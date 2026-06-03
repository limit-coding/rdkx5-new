from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = (
    REPO_ROOT
    / "new_project/ai_training/model_exports/yolo11n_cifar100_cls_full_rtx5060/best.pt"
)


def split_qr_tokens(text: str) -> list[str]:
    normalized = text
    for old, new in {"，": ",", "、": ",", "：": ":", "；": ";"}.items():
        normalized = normalized.replace(old, new)
    for ch in ",;|:=/\\[]{}()\"'":
        normalized = normalized.replace(ch, " ")
    return [token.strip() for token in normalized.split() if token.strip()]


def parse_qr_mission(text: str) -> tuple[str, str, str] | None:
    ignored_keys = {
        "class",
        "class1",
        "class2",
        "target",
        "target1",
        "target2",
        "side",
        "landing",
        "landing_side",
        "land",
        "qr",
        "task",
    }
    classes: list[str] = []
    landing_side = ""
    for token in split_qr_tokens(text):
        lower = token.lower()
        if lower in {"left", "right"}:
            landing_side = lower
        elif lower not in ignored_keys:
            classes.append(token)
    if len(classes) < 2 or not landing_side:
        return None
    return classes[0], classes[1], landing_side


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


class TargetCropper:
    def __init__(self, min_area_ratio: float = 0.03) -> None:
        self.min_area_ratio = min_area_ratio

    def crop(self, frame: np.ndarray) -> tuple[np.ndarray, str]:
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        white_circle = self._find_white_circle(gray, h * w * 0.01)
        if white_circle is not None:
            cx, cy, r = white_circle
            x1, y1 = max(0, cx - r), max(0, cy - r)
            x2, y2 = min(w, cx + r), min(h, cy + r)
            white_crop = frame[y1:y2, x1:x2]
            crop_cx, crop_cy = cx - x1, cy - y1
            inner = self._find_content_in_circle(white_crop, crop_cx, crop_cy, r)
            if inner is not None:
                return inner, "white_circle+content"

            side = int(r * 1.6)
            x1f = max(0, crop_cx - side // 2)
            y1f = max(0, crop_cy - side // 2)
            patch = white_crop[
                y1f : min(y1f + side, white_crop.shape[0]),
                x1f : min(x1f + side, white_crop.shape[1]),
            ]
            if patch.size > 0:
                return patch, "white_circle+center"

        min_area = h * w * self.min_area_ratio
        approx = self._find_quad_from_dark_region(gray, min_area)
        method = "dark_quad"
        if approx is None:
            approx = self._find_quad_from_canny(gray, min_area)
            method = "canny_quad"
        if approx is not None:
            src = order_points(approx)
            side = 320
            dst = np.array(
                [[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]],
                dtype=np.float32,
            )
            warped = cv2.warpPerspective(
                frame, cv2.getPerspectiveTransform(src, dst), (side, side)
            )
            crop_side = int(side * 0.40)
            start = side // 2 - crop_side // 2
            return warped[start : start + crop_side, start : start + crop_side], method

        side = int(min(h, w) * 0.40)
        x1 = w // 2 - side // 2
        y1 = h // 2 - side // 2
        return frame[y1 : y1 + side, x1 : x1 + side], "center"

    def _find_white_circle(self, gray: np.ndarray, min_area: float):
        h, w = gray.shape
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        best, best_r = None, 0.0
        for threshold in range(200, 120, -10):
            _, mask = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area:
                    continue
                (cx, cy), radius = cv2.minEnclosingCircle(contour)
                circularity = area / (np.pi * radius * radius) if radius > 0 else 0.0
                if circularity < 0.45 or radius <= best_r:
                    continue
                margin = int(radius * 0.05)
                if (
                    cx - radius < margin
                    or cx + radius > w - margin
                    or cy - radius < margin
                    or cy + radius > h - margin
                ):
                    continue
                best, best_r = (int(cx), int(cy), int(radius)), radius
            if best is not None:
                break
        return best

    def _find_content_in_circle(
        self, white_crop: np.ndarray, crop_cx: int, crop_cy: int, r_white: int
    ) -> np.ndarray | None:
        h, w = white_crop.shape[:2]
        gray = cv2.cvtColor(white_crop, cv2.COLOR_BGR2GRAY)
        min_card_area = (r_white * 0.20) ** 2
        max_card_area = (r_white * 1.6) ** 2

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        best_approx, best_area = None, 0.0
        for lo, hi in [(20, 80), (30, 100), (50, 150)]:
            edges = cv2.Canny(blurred, lo, hi)
            contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_card_area or area > max_card_area or area <= best_area:
                    continue
                hull = cv2.convexHull(contour)
                peri = cv2.arcLength(hull, True)
                for eps in (0.02, 0.04, 0.06, 0.10):
                    approx = cv2.approxPolyDP(hull, eps * peri, True)
                    if len(approx) == 4:
                        best_approx, best_area = approx, area
                        break
        if best_approx is not None:
            src = order_points(best_approx)
            side = 224
            dst = np.array(
                [[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]],
                dtype=np.float32,
            )
            return cv2.warpPerspective(
                white_crop, cv2.getPerspectiveTransform(src, dst), (side, side)
            )

        yy, xx = np.ogrid[:h, :w]
        circle_mask = (
            (xx - crop_cx) ** 2 + (yy - crop_cy) ** 2 <= r_white**2
        ).astype(np.uint8) * 255
        hsv = cv2.cvtColor(white_crop, cv2.COLOR_BGR2HSV)
        white_bg = cv2.inRange(hsv, (0, 0, 185), (180, 30, 255))
        content = cv2.bitwise_and(cv2.bitwise_not(white_bg), circle_mask)
        ksize = max(5, r_white // 15)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        content = cv2.morphologyEx(content, cv2.MORPH_CLOSE, kernel)
        content = cv2.morphologyEx(content, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(content, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) <= min_card_area * 0.2:
            return None

        if cv2.contourArea(largest) / (np.pi * r_white * r_white) >= 0.35:
            hull = cv2.convexHull(largest)
            peri = cv2.arcLength(hull, True)
            for eps in (0.02, 0.04, 0.06, 0.10):
                approx = cv2.approxPolyDP(hull, eps * peri, True)
                if len(approx) != 4:
                    continue
                pts = order_points(approx)
                sides = [np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)]
                if min(sides) < r_white * 0.20:
                    continue
                side = 224
                dst = np.array(
                    [[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]],
                    dtype=np.float32,
                )
                return cv2.warpPerspective(
                    white_crop, cv2.getPerspectiveTransform(pts, dst), (side, side)
                )

        bx, by, bw, bh = cv2.boundingRect(largest)
        pad = int(max(bw, bh) * 0.15)
        side = max(bw, bh) + 2 * pad
        cx2, cy2 = bx + bw // 2, by + bh // 2
        x1 = max(0, cx2 - side // 2)
        y1 = max(0, cy2 - side // 2)
        return white_crop[y1 : min(h, y1 + side), x1 : min(w, x1 + side)]

    def _find_quad_from_dark_region(
        self, gray: np.ndarray, min_area: float
    ) -> np.ndarray | None:
        h, w = gray.shape
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        best_approx, best_area = None, 0.0
        for thresh_val in (70, 90, 110, 120, 130, 140, 150):
            _, dark_mask = cv2.threshold(blurred, thresh_val, 255, cv2.THRESH_BINARY_INV)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
            dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel)
            dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(
                dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area or area > h * w * 0.85 or area <= best_area:
                    continue
                hull = cv2.convexHull(contour)
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
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area or area <= best_area:
                    continue
                hull = cv2.convexHull(contour)
                peri = cv2.arcLength(hull, True)
                for eps in (0.02, 0.03, 0.05, 0.08, 0.12):
                    approx = cv2.approxPolyDP(hull, eps * peri, True)
                    if len(approx) == 4:
                        best_approx, best_area = approx, area
                        break
        return best_approx


@dataclass
class Classification:
    label: str
    score: float
    topk: list[tuple[str, float]]
    angle: str
    reason: str


class LocalYolo:
    def __init__(self, model_path: Path, imgsz: int, conf: float, rank_k: int) -> None:
        if not model_path.exists():
            raise FileNotFoundError(f"YOLO model not found: {model_path}")
        self.model = YOLO(str(model_path))
        self.imgsz = imgsz
        self.conf = conf
        self.rank_k = rank_k
        self.task = str(getattr(self.model, "task", "") or "")

    def classify(
        self, crop: np.ndarray, qr_targets: set[str] | None, topk: int = 5
    ) -> Classification:
        rotations = [
            ("0", None),
            ("90", cv2.ROTATE_90_CLOCKWISE),
            ("180", cv2.ROTATE_180),
            ("270", cv2.ROTATE_90_COUNTERCLOCKWISE),
        ]
        best_probs = None
        best_angle = "0"
        best_score = -1.0

        for angle, rot_code in rotations:
            image = cv2.rotate(crop, rot_code) if rot_code is not None else crop
            result = self.model(image, imgsz=self.imgsz, verbose=False)[0]
            if result.probs is None:
                raise RuntimeError("Loaded YOLO model is not a classification model")
            probs = result.probs.data.cpu().numpy()
            if qr_targets:
                ranked = np.argsort(-probs)
                for rank, idx in enumerate(ranked[: self.rank_k], 1):
                    label = str(result.names[int(idx)]).strip().lower()
                    if self._match_qr_target(label, qr_targets):
                        score = float(probs[int(idx)]) / rank
                        if score > best_score:
                            best_score = score
                            best_probs = probs
                            best_angle = angle
                        break
            else:
                score = float(probs.max())
                if score > best_score:
                    best_score = score
                    best_probs = probs
                    best_angle = angle

        if best_probs is None:
            result = self.model(crop, imgsz=self.imgsz, verbose=False)[0]
            best_probs = result.probs.data.cpu().numpy()

        names = self.model.names
        ranked_ids = np.argsort(-best_probs)
        top_items = [
            (str(names[int(idx)]), float(best_probs[int(idx)]))
            for idx in ranked_ids[: max(topk, self.rank_k)]
        ]
        label, score, reason = self._choose_label(top_items, qr_targets)
        return Classification(label, score, top_items[:topk], best_angle, reason)

    def detect(self, frame: np.ndarray):
        return self.model(frame, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]

    def _choose_label(
        self, ranked: list[tuple[str, float]], qr_targets: set[str] | None
    ) -> tuple[str, float, str]:
        if qr_targets:
            for rank, (raw_label, score) in enumerate(ranked[: self.rank_k], 1):
                matched = self._match_qr_target(raw_label, qr_targets)
                if matched:
                    return matched, score, f"qr-rank-{rank}"
        if not ranked:
            return "", 0.0, "empty"
        label, score = ranked[0]
        return label, score, "top1"

    @staticmethod
    def _match_qr_target(label: str, qr_targets: set[str]) -> str:
        label_norm = str(label).strip().lower()
        for target in qr_targets:
            target_norm = str(target).strip().lower()
            if target_norm and (
                label_norm == target_norm
                or label_norm in target_norm
                or target_norm in label_norm
            ):
                return target_norm
        return ""


def open_camera(index: int, width: int, height: int, fps: int) -> cv2.VideoCapture:
    backend = cv2.CAP_AVFOUNDATION if hasattr(cv2, "CAP_AVFOUNDATION") else 0
    cap = cv2.VideoCapture(index, backend)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def open_video_source(source: str | None, index: int, width: int, height: int, fps: int):
    if source:
        cap = cv2.VideoCapture(source)
        source_name = source
    else:
        cap = open_camera(index, width, height, fps)
        source_name = f"camera {index}"
    return cap, source_name


def scan_cameras(max_index: int, width: int, height: int, fps: int) -> list[dict[str, object]]:
    cameras: list[dict[str, object]] = []
    for index in range(max_index + 1):
        cap = open_camera(index, width, height, fps)
        if not cap.isOpened():
            cap.release()
            continue

        ok, frame = cap.read()
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(cap.get(cv2.CAP_PROP_FPS))
        cap.release()

        cameras.append(
            {
                "index": index,
                "readable": bool(ok and frame is not None),
                "width": actual_w,
                "height": actual_h,
                "fps": actual_fps,
            }
        )
    return cameras


def print_camera_list(cameras: list[dict[str, object]]) -> None:
    if not cameras:
        print("No camera index could be opened.", flush=True)
        print("If macOS asks for permission, allow Camera access and rerun.", flush=True)
        return

    print("Available OpenCV camera indices:", flush=True)
    for item in cameras:
        readable = "ok" if item["readable"] else "opened/no-frame"
        print(
            f"  [{item['index']}] {readable} "
            f"{item['width']}x{item['height']}@{float(item['fps']):.1f}",
            flush=True,
        )


def choose_camera_index(args: argparse.Namespace) -> int:
    cameras = scan_cameras(args.scan_max, args.width, args.height, args.fps)
    print_camera_list(cameras)
    if args.list_cameras:
        return -1
    if not cameras:
        return args.camera

    readable = [item for item in cameras if item["readable"]]
    default_index = int((readable or cameras)[-1]["index"])
    prompt = f"Choose camera index [{default_index}]: "
    answer = input(prompt).strip()
    if not answer:
        return default_index
    try:
        return int(answer)
    except ValueError:
        print(f"Invalid camera index {answer!r}; using {default_index}", flush=True)
        return default_index


def put_text(frame: np.ndarray, text: str, org: tuple[int, int], color=(255, 255, 255)) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.58
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = org
    cv2.rectangle(frame, (x - 4, y - th - 6), (x + tw + 4, y + baseline + 4), (0, 0, 0), -1)
    cv2.putText(frame, text, org, font, scale, color, thickness, cv2.LINE_AA)


def draw_qr_points(frame: np.ndarray, points) -> None:
    if points is None:
        return
    pts = np.asarray(points, dtype=np.int32).reshape(-1, 2)
    if pts.shape[0] >= 4:
        cv2.polylines(frame, [pts.reshape(-1, 1, 2)], True, (0, 255, 0), 2)


def draw_crop_preview(frame: np.ndarray, crop: np.ndarray) -> None:
    if crop.size == 0:
        return
    preview_h = min(150, frame.shape[0] // 3)
    preview_w = preview_h
    preview = cv2.resize(crop, (preview_w, preview_h), interpolation=cv2.INTER_AREA)
    y1 = frame.shape[0] - preview_h - 12
    x1 = frame.shape[1] - preview_w - 12
    frame[y1 : y1 + preview_h, x1 : x1 + preview_w] = preview
    cv2.rectangle(frame, (x1, y1), (x1 + preview_w, y1 + preview_h), (0, 255, 255), 2)


def run(args: argparse.Namespace) -> int:
    if args.list_cameras or args.select_camera:
        selected = choose_camera_index(args)
        if args.list_cameras:
            return 0
        args.camera = selected

    model_path = Path(args.model).expanduser().resolve()
    yolo = LocalYolo(model_path, args.imgsz, args.conf, args.target_rank_k)
    cropper = TargetCropper()
    qr_detector = cv2.QRCodeDetector()

    cap, source_name = open_video_source(args.source, args.camera, args.width, args.height, args.fps)
    if not cap.isOpened():
        print(f"Failed to open video source: {source_name}", flush=True)
        return 1

    print(
        f"Opened {source_name}. "
        "QR first, then YOLO. Press q to quit, r to reset QR, y to force YOLO.",
        flush=True,
    )
    print(f"YOLO model: {model_path} (task={yolo.task or 'unknown'})", flush=True)

    phase = "qr"
    last_qr_text = ""
    qr_stable_frames = 0
    qr_targets: set[str] = set()
    target_candidate = ""
    target_stable_frames = 0
    last_print = 0.0
    frame_count = 0

    try:
        while args.max_frames <= 0 or frame_count < args.max_frames:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("Failed to read frame", flush=True)
                time.sleep(0.05)
                continue
            frame_count += 1

            if args.rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            view = frame.copy()
            now = time.time()

            if phase == "qr":
                text, points, _ = qr_detector.detectAndDecode(frame)
                text = text.strip()
                draw_qr_points(view, points)
                mission = parse_qr_mission(text) if text else None
                if mission is None:
                    last_qr_text = ""
                    qr_stable_frames = 0
                    if now - last_print >= args.print_interval:
                        msg = "No valid QR" if not text else f"QR format not mission: {text!r}"
                        print(msg, flush=True)
                        last_print = now
                else:
                    if text == last_qr_text:
                        qr_stable_frames += 1
                    else:
                        last_qr_text = text
                        qr_stable_frames = 1
                    class1, class2, landing_side = mission
                    put_text(
                        view,
                        f"QR {qr_stable_frames}/{args.qr_confirm_frames}: {class1}, {class2}, {landing_side}",
                        (12, 28),
                        (0, 255, 0),
                    )
                    if qr_stable_frames >= args.qr_confirm_frames:
                        qr_targets = {class1.lower(), class2.lower()}
                        payload = {
                            "valid": True,
                            "confirmed": True,
                            "class1": class1,
                            "class2": class2,
                            "landing_side": landing_side,
                        }
                        print("QR confirmed: " + json.dumps(payload, ensure_ascii=False), flush=True)
                        phase = "yolo"
                        target_candidate = ""
                        target_stable_frames = 0
                if mission is None:
                    put_text(view, "PHASE QR: show mission QR", (12, 28), (0, 255, 255))

            else:
                if yolo.task == "detect":
                    result = yolo.detect(frame)
                    view = result.plot()
                    put_text(view, "PHASE YOLO DETECT", (12, 28), (0, 255, 255))
                else:
                    crop, crop_method = cropper.crop(frame)
                    cls = yolo.classify(crop, qr_targets or None, topk=args.topk)
                    draw_crop_preview(view, crop)
                    put_text(
                        view,
                        f"PHASE YOLO CLS: {cls.label} {cls.score:.2%} {cls.reason} rot={cls.angle}",
                        (12, 28),
                        (0, 255, 255),
                    )
                    put_text(view, f"crop={crop_method}", (12, 55), (255, 255, 255))
                    for idx, (name, score) in enumerate(cls.topk[:3], 1):
                        put_text(view, f"#{idx} {name} {score:.1%}", (12, 55 + idx * 27))

                    if cls.score >= args.conf:
                        if cls.label == target_candidate:
                            target_stable_frames += 1
                        else:
                            target_candidate = cls.label
                            target_stable_frames = 1
                        if target_stable_frames >= args.target_confirm_frames:
                            print(
                                f"Target confirmed: {cls.label} "
                                f"score={cls.score:.4f} reason={cls.reason}",
                                flush=True,
                            )
                            target_stable_frames = 0
                    else:
                        target_candidate = ""
                        target_stable_frames = 0

            if args.no_window:
                continue

            cv2.imshow("Local Mission Vision", view)
            key = cv2.waitKey(1) & 0xFF
            if key in {ord("q"), 27}:
                break
            if key == ord("r"):
                phase = "qr"
                last_qr_text = ""
                qr_stable_frames = 0
                qr_targets = set()
                print("Reset to QR phase", flush=True)
            if key == ord("y"):
                phase = "yolo"
                print("Forced YOLO phase", flush=True)

    except KeyboardInterrupt:
        print("Stopping", flush=True)
    finally:
        cap.release()
        if not args.no_window:
            cv2.destroyAllWindows()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run QR-first, then YOLO recognition from a local Mac camera."
    )
    parser.add_argument("--camera", type=int, default=1, help="Mac camera index. Default: 1")
    parser.add_argument(
        "--source",
        default="",
        help="Video source URL/file, for example http://192.168.66.2:8080/stream",
    )
    parser.add_argument("--list-cameras", action="store_true", help="Scan usable OpenCV camera indices and exit")
    parser.add_argument("--select-camera", action="store_true", help="Scan cameras and choose one interactively")
    parser.add_argument("--scan-max", type=int, default=8, help="Highest camera index to scan")
    parser.add_argument("--model", default=str(DEFAULT_MODEL), help="Ultralytics .pt model path")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--imgsz", type=int, default=224)
    parser.add_argument("--conf", type=float, default=0.30)
    parser.add_argument("--qr-confirm-frames", type=int, default=3)
    parser.add_argument("--target-confirm-frames", type=int, default=3)
    parser.add_argument("--target-rank-k", type=int, default=20)
    parser.add_argument("--topk", type=int, default=5)
    parser.add_argument("--rotate-180", action="store_true")
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--print-interval", type=float, default=1.0)
    return parser


def main() -> None:
    raise SystemExit(run(build_parser().parse_args()))


if __name__ == "__main__":
    main()
