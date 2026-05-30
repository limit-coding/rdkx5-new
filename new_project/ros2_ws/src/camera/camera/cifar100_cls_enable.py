import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from hobot_dnn import pyeasy_dnn as dnn
from hobot_vio import libsrcampy as srcampy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension, String


DEFAULT_MODEL_PATH = "/home/sunrise/ros2/diansai/ws/src/camera/resource/cifar100_cls.bin"
DEFAULT_NAMES_PATH = "/home/sunrise/ros2/diansai/ws/src/camera/resource/cifar100_names.txt"


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
        self.model = dnn.load(model_path)
        self.names = self._load_names(names_path)
        shape = self.model[0].inputs[0].properties.shape
        self.input_h, self.input_w = self._parse_hw(shape)

    def _parse_hw(self, shape) -> tuple[int, int]:
        dims = list(shape)
        if len(dims) >= 4:
            return int(dims[2]), int(dims[3])
        if len(dims) >= 2:
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

    def predict_topk(self, bgr: np.ndarray, k: int = 5) -> list[tuple[int, str, float]]:
        outputs = self.model[0].forward(self.bgr_to_nv12(bgr))
        logits = outputs[0].buffer
        probs = softmax(np.asarray(logits))
        top_ids = np.argsort(-probs)[:k]
        return [(int(i), self.names[int(i)] if int(i) < len(self.names) else str(int(i)), float(probs[int(i)])) for i in top_ids]


class TargetCenterCropper:
    def __init__(self, min_area_ratio: float = 0.03) -> None:
        self.min_area_ratio = min_area_ratio

    def crop(self, frame: np.ndarray) -> np.ndarray:
        warped = self._warp_largest_quad(frame)
        if warped is None:
            warped = self._center_crop(frame)
        return self._center_crop(warped, ratio=0.62)

    def _center_crop(self, frame: np.ndarray, ratio: float = 0.75) -> np.ndarray:
        h, w = frame.shape[:2]
        side = int(min(h, w) * ratio)
        cx, cy = w // 2, h // 2
        x1 = max(0, cx - side // 2)
        y1 = max(0, cy - side // 2)
        return frame[y1 : y1 + side, x1 : x1 + side]

    def _warp_largest_quad(self, frame: np.ndarray) -> np.ndarray | None:
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 60, 160)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = h * w * self.min_area_ratio

        best = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area <= best_area:
                continue
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.03 * peri, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                best = approx
                best_area = area

        if best is None:
            return None

        src = order_points(best)
        side = 320
        dst = np.array([[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]], dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(frame, matrix, (side, side))


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
