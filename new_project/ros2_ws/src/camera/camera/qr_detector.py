import argparse
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class QrDetection:
    text: str
    points: np.ndarray
    center_x: int
    center_y: int
    offset_x: int
    offset_y: int
    area: int


class QrDetector:
    def __init__(self, min_area: int = 100, require_text: bool = True) -> None:
        self.detector = cv2.QRCodeDetector()
        self.min_area = min_area
        self.require_text = require_text

    def detect(self, frame: np.ndarray) -> List[QrDetection]:
        if frame is None or frame.size == 0:
            return []

        height, width = frame.shape[:2]
        frame_center_x = width // 2
        frame_center_y = height // 2
        detections: List[QrDetection] = []

        multi_result = self._detect_multi(frame)
        if multi_result is not None:
            decoded_texts, points_list = multi_result
            for text, points in zip(decoded_texts, points_list):
                detection = self._build_detection(
                    text=text,
                    points=points,
                    frame_center_x=frame_center_x,
                    frame_center_y=frame_center_y,
                )
                if detection is not None:
                    detections.append(detection)

        if detections:
            return detections

        text, points, _ = self.detector.detectAndDecode(frame)
        detection = self._build_detection(
            text=text,
            points=points,
            frame_center_x=frame_center_x,
            frame_center_y=frame_center_y,
        )
        return [detection] if detection is not None else []

    def draw(self, frame: np.ndarray, detections: Sequence[QrDetection]) -> np.ndarray:
        output = frame.copy()
        height, width = output.shape[:2]
        cv2.drawMarker(
            output,
            (width // 2, height // 2),
            (0, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
        )

        for detection in detections:
            pts = detection.points.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(output, [pts], True, (0, 255, 0), 2)
            cv2.circle(output, (detection.center_x, detection.center_y), 5, (0, 0, 255), -1)
            label = detection.text if detection.text else "QR"
            cv2.putText(
                output,
                label[:40],
                (detection.center_x + 8, detection.center_y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        return output

    def _detect_multi(
        self, frame: np.ndarray
    ) -> Optional[Tuple[Sequence[str], Sequence[np.ndarray]]]:
        if not hasattr(self.detector, "detectAndDecodeMulti"):
            return None

        ok, decoded_texts, points, _ = self.detector.detectAndDecodeMulti(frame)
        if not ok or points is None:
            return None
        return decoded_texts, points

    def _build_detection(
        self,
        text: str,
        points: Optional[np.ndarray],
        frame_center_x: int,
        frame_center_y: int,
    ) -> Optional[QrDetection]:
        if points is None:
            return None
        if self.require_text and not text:
            return None

        pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        if pts.shape[0] < 4:
            return None

        area = int(abs(cv2.contourArea(pts)))
        if area < self.min_area:
            return None

        center = pts.mean(axis=0)
        center_x = int(round(float(center[0])))
        center_y = int(round(float(center[1])))

        return QrDetection(
            text=text or "",
            points=pts,
            center_x=center_x,
            center_y=center_y,
            offset_x=center_x - frame_center_x,
            offset_y=center_y - frame_center_y,
            area=area,
        )


def decode_jpeg_bytes(data: bytes) -> Optional[np.ndarray]:
    buffer = np.frombuffer(data, dtype=np.uint8)
    if buffer.size == 0:
        return None
    return cv2.imdecode(buffer, cv2.IMREAD_COLOR)


def open_capture(device: str, width: int, height: int, fps: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def format_text_for_log(text: str, show_text: bool = False) -> str:
    if show_text:
        return repr(text)
    if not text:
        return "<empty>"
    return f"<decoded length={len(text)}>"


def print_detection(prefix: str, detection: QrDetection, show_text: bool = False) -> None:
    print(
        f"{prefix} text={format_text_for_log(detection.text, show_text)} "
        f"center=({detection.center_x},{detection.center_y}) "
        f"offset=({detection.offset_x},{detection.offset_y}) "
        f"area={detection.area}",
        flush=True,
    )


def run_device(args: argparse.Namespace) -> int:
    detector = QrDetector(min_area=args.min_area, require_text=not args.allow_empty)
    cap = open_capture(args.device, args.width, args.height, args.fps)
    if not cap.isOpened():
        print(f"Failed to open camera device: {args.device}", flush=True)
        return 1

    print(
        f"Reading {args.device} at {args.width}x{args.height}@{args.fps}. "
        "Press Ctrl+C to stop.",
        flush=True,
    )
    last_text = None
    last_print_time = 0.0
    frame_count = 0

    try:
        while args.max_frames <= 0 or frame_count < args.max_frames:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame", flush=True)
                time.sleep(0.1)
                continue

            frame_count += 1
            detections = detector.detect(frame)
            now = time.time()
            if detections:
                for detection in detections:
                    if detection.text != last_text or now - last_print_time >= args.print_interval:
                        print_detection("QR", detection, show_text=args.show_text)
                        last_text = detection.text
                        last_print_time = now
            elif now - last_print_time >= args.print_interval:
                print("No QR code detected", flush=True)
                last_print_time = now

            if args.output and detections:
                cv2.imwrite(args.output, detector.draw(frame, detections))

    except KeyboardInterrupt:
        print("Stopping", flush=True)
    finally:
        cap.release()
    return 0


def run_image(args: argparse.Namespace) -> int:
    frame = cv2.imread(args.image)
    if frame is None:
        print(f"Failed to read image: {args.image}", flush=True)
        return 1

    detector = QrDetector(min_area=args.min_area, require_text=not args.allow_empty)
    detections = detector.detect(frame)
    if detections:
        for detection in detections:
            print_detection("QR", detection, show_text=args.show_text)
    else:
        print("No QR code detected", flush=True)

    if args.output:
        cv2.imwrite(args.output, detector.draw(frame, detections))
        print(f"Wrote annotated image: {args.output}", flush=True)
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detect QR codes from an image or USB camera.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--device", help="V4L2 device, for example /dev/video0")
    source.add_argument("--image", help="Image file to test")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--min-area", type=int, default=100)
    parser.add_argument("--allow-empty", action="store_true")
    parser.add_argument("--show-text", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--print-interval", type=float, default=1.0)
    parser.add_argument("--output", help="Write annotated image when available")
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.image:
        raise SystemExit(run_image(args))
    raise SystemExit(run_device(args))


if __name__ == "__main__":
    main()
