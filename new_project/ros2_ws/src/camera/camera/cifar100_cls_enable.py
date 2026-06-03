import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from hobot_vio import libsrcampy as srcampy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension, String


DEFAULT_MODEL_PATH = "/home/sunrise/project/camera/resource/cifar100_cls.onnx"
DEFAULT_NAMES_PATH = "/home/sunrise/project/camera/resource/cifar100_names.txt"


def softmax(logits: np.ndarray) -> np.ndarray:
    logits = logits.astype(np.float32).reshape(-1)
    logits = logits - np.max(logits)
    exp = np.exp(logits)
    return exp / np.sum(exp)


def order_points(points: np.ndarray) -> np.ndarray:
    pts = points.reshape(4, 2).astype(np.float32)
    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)
    return np.array(
        [
            pts[np.argmin(sums)],
            pts[np.argmin(diffs)],
            pts[np.argmax(sums)],
            pts[np.argmax(diffs)],
        ],
        dtype=np.float32,
    )


class Cifar100BpuClassifier:
    def __init__(self, model_path: str, names_path: str) -> None:
        self.names = self._load_names(names_path)
        self.input_h = 224
        self.input_w = 224

        p = Path(model_path)
        onnx_path = p if p.suffix == ".onnx" else p.with_suffix(".onnx")
        if not onnx_path.exists():
            raise FileNotFoundError(f"ONNX model not found: {onnx_path}")

        import os
        import onnxruntime as ort

        os.environ.setdefault("ORT_LOGGING_LEVEL", "3")
        self._ort_session = ort.InferenceSession(
            str(onnx_path), providers=["CPUExecutionProvider"]
        )
        self._ort_input_name = self._ort_session.get_inputs()[0].name
        shape = self._ort_session.get_inputs()[0].shape
        self.input_h, self.input_w = self._parse_hw(shape)

    def _parse_hw(self, shape) -> tuple[int, int]:
        dims = list(shape)
        if len(dims) >= 4 and isinstance(dims[2], int) and isinstance(dims[3], int):
            return int(dims[2]), int(dims[3])
        if len(dims) >= 2 and isinstance(dims[-2], int) and isinstance(dims[-1], int):
            return int(dims[-2]), int(dims[-1])
        return 224, 224

    def _load_names(self, names_path: str) -> list[str]:
        path = Path(names_path)
        if not path.exists():
            return [str(i) for i in range(100)]
        names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        return names if names else [str(i) for i in range(100)]

    def bgr_to_nv12(self, bgr: np.ndarray) -> np.ndarray:
        resized = cv2.resize(bgr, (self.input_w, self.input_h), interpolation=cv2.INTER_AREA)
        area = self.input_h * self.input_w
        yuv420p = cv2.cvtColor(resized, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
        y = yuv420p[:area]
        uv_planar = yuv420p[area:].reshape((2, area // 4))
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
        nv12 = np.empty_like(yuv420p)
        nv12[:area] = y
        nv12[area:] = uv_packed
        return nv12

    def _run_inference(self, bgr: np.ndarray) -> np.ndarray:
        resized = cv2.resize(bgr, (self.input_w, self.input_h), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        inp = rgb.transpose(2, 0, 1)[np.newaxis]
        return self._ort_session.run(None, {self._ort_input_name: inp})[0].reshape(-1)

    def predict_topk(
        self, bgr: np.ndarray, k: int = 5, qr_targets: set | None = None
    ) -> list[tuple[int, str, float]]:
        rotations = [None, cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_180, cv2.ROTATE_90_COUNTERCLOCKWISE]
        best_probs: np.ndarray | None = None
        best_score = -1.0
        for rot_code in rotations:
            img = cv2.rotate(bgr, rot_code) if rot_code is not None else bgr
            probs = self._run_inference(img)
            if qr_targets:
                ranked = np.argsort(-probs)
                for rank, idx in enumerate(ranked[:20], 1):
                    label = self.names[int(idx)].lower() if int(idx) < len(self.names) else ""
                    if label in qr_targets:
                        score = float(probs[idx]) / rank
                        if score > best_score:
                            best_score, best_probs = score, probs
                        # Early exit: QR target already in top-10 with good confidence
                        if rank <= 10 and float(probs[idx]) >= 0.30:
                            top_ids = np.argsort(-best_probs)[:k]
                            return [
                                (int(i), self.names[int(i)] if int(i) < len(self.names) else str(int(i)),
                                 float(best_probs[int(i)]))
                                for i in top_ids
                            ]
                        break
            else:
                s = float(probs.max())
                if s > best_score:
                    best_score, best_probs = s, probs
                # Early exit for non-QR mode: top-1 confidence already high
                if s >= 0.60:
                    break
        if best_probs is None:
            best_probs = self._run_inference(bgr)
        top_ids = np.argsort(-best_probs)[:k]
        return [
            (int(i), self.names[int(i)] if int(i) < len(self.names) else str(int(i)), float(best_probs[int(i)]))
            for i in top_ids
        ]


class TargetCenterCropper:
    def __init__(self, min_area_ratio: float = 0.03) -> None:
        self.min_area_ratio = min_area_ratio

    def crop(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Priority 1: complete white circle
        wc = self._find_white_circle(gray, h * w * 0.01)
        if wc is not None:
            cx, cy, r = wc
            x1, y1 = max(0, cx - r), max(0, cy - r)
            x2, y2 = min(w, cx + r), min(h, cy + r)
            white_crop = frame[y1:y2, x1:x2]
            crop_cx, crop_cy = cx - x1, cy - y1
            inner = self._find_content_in_circle(white_crop, crop_cx, crop_cy, r)
            if inner is not None:
                return inner
            # Fallback: 80% crop centered on actual circle center
            s = int(r * 1.6)
            x1f = max(0, crop_cx - s // 2)
            y1f = max(0, crop_cy - s // 2)
            patch = white_crop[y1f:min(y1f + s, white_crop.shape[0]),
                               x1f:min(x1f + s, white_crop.shape[1])]
            if patch.size > 0:
                return patch

        # Priority 2: dark quad perspective warp → 40% center crop
        min_area = h * w * self.min_area_ratio
        approx = self._find_quad_from_dark_region(gray, min_area)
        if approx is None:
            approx = self._find_quad_from_canny(gray, min_area)
        if approx is not None:
            src = order_points(approx)
            side = 320
            dst = np.array([[0, 0], [side-1, 0], [side-1, side-1], [0, side-1]], dtype=np.float32)
            warped = cv2.warpPerspective(frame, cv2.getPerspectiveTransform(src, dst), (side, side))
            s = int(320 * 0.40)
            return warped[160 - s//2:160 + s//2, 160 - s//2:160 + s//2]

        # Fallback: center crop
        side = int(min(h, w) * 0.40)
        return frame[h//2 - side//2:h//2 + side//2, w//2 - side//2:w//2 + side//2]

    # ── White circle detection ─────────────────────────────────────────────
    def _find_white_circle(self, gray: np.ndarray, min_area: float):
        """Return (cx, cy, r) only when the full circle contour is within the frame."""
        h, w = gray.shape
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        best, best_r = None, 0.0
        for t in range(200, 120, -10):
            _, mask = cv2.threshold(blurred, t, 255, cv2.THRESH_BINARY)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < min_area:
                    continue
                (cx, cy), r = cv2.minEnclosingCircle(c)
                circ = area / (np.pi * r * r) if r > 0 else 0
                if circ < 0.45 or r <= best_r:
                    continue
                margin = int(r * 0.05)
                if (cx - r < margin or cx + r > w - margin or
                        cy - r < margin or cy + r > h - margin):
                    continue
                best, best_r = (int(cx), int(cy), int(r)), r
            if best:
                break
        return best

    # ── Content extraction inside white circle ─────────────────────────────
    def _find_content_in_circle(
        self, white_crop: np.ndarray, crop_cx: int, crop_cy: int, r_white: int
    ) -> np.ndarray | None:
        h, w = white_crop.shape[:2]
        gray = cv2.cvtColor(white_crop, cv2.COLOR_BGR2GRAY)
        min_card_area = (r_white * 0.20) ** 2
        max_card_area = (r_white * 1.6) ** 2

        # Strategy A: Canny rectangle (dark-bordered card)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        best_approx, best_area = None, 0.0
        for lo, hi in [(20, 80), (30, 100), (50, 150)]:
            edges = cv2.Canny(blurred, lo, hi)
            cnts, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < min_card_area or area > max_card_area or area <= best_area:
                    continue
                hull = cv2.convexHull(c)
                peri = cv2.arcLength(hull, True)
                for eps in (0.02, 0.04, 0.06, 0.10):
                    approx = cv2.approxPolyDP(hull, eps * peri, True)
                    if len(approx) == 4:
                        best_approx, best_area = approx, area
                        break
        if best_approx is not None:
            src = order_points(best_approx)
            side = 224
            dst = np.array([[0, 0], [side-1, 0], [side-1, side-1], [0, side-1]], dtype=np.float32)
            return cv2.warpPerspective(white_crop, cv2.getPerspectiveTransform(src, dst), (side, side))

        # Strategy B: non-white content mask
        yy, xx = np.ogrid[:h, :w]
        circ_mask = ((xx - crop_cx)**2 + (yy - crop_cy)**2
                     <= r_white**2).astype(np.uint8) * 255
        hsv = cv2.cvtColor(white_crop, cv2.COLOR_BGR2HSV)
        white_bg = cv2.inRange(hsv, (0, 0, 185), (180, 30, 255))
        content = cv2.bitwise_and(cv2.bitwise_not(white_bg), circ_mask)
        ksize = max(5, r_white // 15)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        content = cv2.morphologyEx(content, cv2.MORPH_CLOSE, k)
        content = cv2.morphologyEx(content, cv2.MORPH_OPEN, k)

        cnts, _ = cv2.findContours(content, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        largest = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(largest) <= min_card_area * 0.2:
            return None

        # Perspective warp only for colored-background cards (fill ≥ 35%)
        if cv2.contourArea(largest) / (np.pi * r_white * r_white) >= 0.35:
            hull = cv2.convexHull(largest)
            peri = cv2.arcLength(hull, True)
            for eps in (0.02, 0.04, 0.06, 0.10):
                approx = cv2.approxPolyDP(hull, eps * peri, True)
                if len(approx) != 4:
                    continue
                pts = order_points(approx)
                sides = [np.linalg.norm(pts[(i+1) % 4] - pts[i]) for i in range(4)]
                if min(sides) < r_white * 0.20:
                    continue
                side = 224
                dst = np.array([[0, 0], [side-1, 0], [side-1, side-1], [0, side-1]], dtype=np.float32)
                return cv2.warpPerspective(white_crop, cv2.getPerspectiveTransform(pts, dst), (side, side))

        # Fallback: axis-aligned bounding box of content
        bx, by, bw, bh = cv2.boundingRect(largest)
        pad = int(max(bw, bh) * 0.15)
        s = max(bw, bh) + 2 * pad
        cx2, cy2 = bx + bw // 2, by + bh // 2
        x1 = max(0, cx2 - s // 2)
        y1 = max(0, cy2 - s // 2)
        return white_crop[y1:min(h, y1 + s), x1:min(w, x1 + s)]

    # ── Dark quad helpers (fallback when no white circle) ──────────────────
    def _find_quad_from_dark_region(self, gray: np.ndarray, min_area: float) -> np.ndarray | None:
        h, w = gray.shape
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        best_approx, best_area = None, 0.0
        for thresh_val in (70, 90, 110, 120, 130, 140, 150):
            _, dark_mask = cv2.threshold(blurred, thresh_val, 255, cv2.THRESH_BINARY_INV)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
            dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel)
            dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < min_area or area > h * w * 0.85 or area <= best_area:
                    continue
                hull = cv2.convexHull(c)
                peri = cv2.arcLength(hull, True)
                for eps in (0.03, 0.05, 0.08, 0.12):
                    approx = cv2.approxPolyDP(hull, eps * peri, True)
                    if len(approx) == 4:
                        best_approx, best_area = approx, area
                        break
        return best_approx

    def _find_quad_from_canny(self, gray: np.ndarray, min_area: float) -> np.ndarray | None:
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        best_approx, best_area = None, 0.0
        for lo, hi in [(20, 80), (40, 120), (60, 160)]:
            edges = cv2.Canny(blurred, lo, hi)
            contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < min_area or area <= best_area:
                    continue
                hull = cv2.convexHull(c)
                peri = cv2.arcLength(hull, True)
                for eps in (0.02, 0.03, 0.05, 0.08, 0.12):
                    approx = cv2.approxPolyDP(hull, eps * peri, True)
                    if len(approx) == 4:
                        best_approx, best_area = approx, area
                        break
        return best_approx


class Cifar100ClsNode(Node):
    def __init__(self) -> None:
        super().__init__("cifar100_cls_node")
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("names_path", DEFAULT_NAMES_PATH)
        self.declare_parameter("camera_index", 1)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("topk", 5)

        model_path = self.get_parameter("model_path").value
        names_path = self.get_parameter("names_path").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        camera_index = int(self.get_parameter("camera_index").value)
        self.topk = int(self.get_parameter("topk").value)

        self.classifier = Cifar100BpuClassifier(model_path, names_path)
        self.cropper = TargetCenterCropper()
        self.camera = srcampy.Camera()
        self.camera.open_cam(camera_index, -1, -1, self.width, self.height, self.height, self.width)

        self.enable = 0
        self.lock = threading.Lock()
        self.enable_sub = self.create_subscription(Int32, "/pic_cls_enable", self.enable_callback, 10)
        self.name_pub = self.create_publisher(String, "/pic_cls_name", 10)
        self.topk_pub = self.create_publisher(Int32MultiArray, "/pic_cls_top5", 10)

        self.thread = threading.Thread(target=self.loop_task, daemon=True)
        self.thread.start()

    def enable_callback(self, msg: Int32) -> None:
        with self.lock:
            self.enable = int(msg.data)

    def loop_task(self) -> None:
        while rclpy.ok():
            with self.lock:
                enabled = self.enable
            if not enabled:
                time.sleep(0.02)
                continue

            nv12_img = self.camera.get_img(2, self.width, self.height)
            if nv12_img is None:
                self.get_logger().warning("failed to get camera image")
                time.sleep(0.05)
                continue

            nv12_data = np.frombuffer(nv12_img, dtype=np.uint8).reshape(self.height * 3 // 2, self.width)
            frame = cv2.cvtColor(nv12_data, cv2.COLOR_YUV2BGR_NV12)
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            crop = self.cropper.crop(frame)
            topk = self.classifier.predict_topk(crop, self.topk)

            best_id, best_name, best_score = topk[0]
            self.name_pub.publish(String(data=f"{best_name}:{best_score:.4f}"))

            msg = Int32MultiArray()
            msg.layout.dim = [MultiArrayDimension(label="topk", size=len(topk), stride=len(topk) * 2)]
            msg.data = []
            for class_id, _, score in topk:
                msg.data.extend([class_id, int(score * 10000)])
            self.topk_pub.publish(msg)
            self.get_logger().info(f"top1 id={best_id} name={best_name} score={best_score:.3f}")

    def destroy_node(self) -> bool:
        self.camera.close_cam()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Cifar100ClsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
