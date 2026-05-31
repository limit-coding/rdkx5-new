from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
from hobot_dnn import pyeasy_dnn as dnn
from hobot_vio import libsrcampy as srcampy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension, String


DEFAULT_MODEL_PATH = "/home/sunrise/project/camera/resource/yolo11_det.bin"
DEFAULT_NAMES = [
    "picture_target",
    "special_target",
    "ring",
    "obstacle",
    "landing_h",
    "red_light",
    "blue_light",
]

COLORS = [
    (56, 56, 255),
    (31, 112, 255),
    (49, 210, 207),
    (10, 249, 72),
    (187, 212, 0),
    (236, 24, 0),
    (255, 56, 203),
    (200, 149, 255),
]


@dataclass(frozen=True)
class Detection:
    class_id: int
    score: float
    bbox: tuple[int, int, int, int]


@dataclass(frozen=True)
class LetterboxInfo:
    scale: float
    pad_x: int
    pad_y: int
    source_w: int
    source_h: int


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    x = x.astype(np.float32)
    x = x - np.max(x, axis=axis, keepdims=True)
    exp = np.exp(x)
    return exp / np.sum(exp, axis=axis, keepdims=True)


def parse_names(raw: str) -> list[str]:
    names = [name.strip() for name in raw.split(",") if name.strip()]
    return names or DEFAULT_NAMES


class Yolo11BpuDetector:
    def __init__(self, model_path: str, class_names: list[str], conf: float, iou: float) -> None:
        self.model_path = model_path
        self.class_names = class_names
        self.num_classes = len(class_names)
        self.conf = conf
        self.iou = iou
        self.conf_logit = -np.log(1.0 / conf - 1.0)

        begin = time.time()
        self.model = dnn.load(model_path)
        load_ms = (time.time() - begin) * 1000.0

        input_shape = list(self.model[0].inputs[0].properties.shape)
        self.input_h, self.input_w = self._parse_hw(input_shape)
        self.strides = (8, 16, 32)
        self.anchors = {stride: self._make_anchors(stride) for stride in self.strides}
        self.dfl_weights = np.arange(16, dtype=np.float32)[np.newaxis, np.newaxis, :]
        self.letterbox = LetterboxInfo(1.0, 0, 0, self.input_w, self.input_h)

        print(f"[RDK_YOLO] loaded {model_path} in {load_ms:.1f} ms")
        print(f"[RDK_YOLO] input shape={input_shape}, input_hw={self.input_h}x{self.input_w}")
        for idx, output in enumerate(self.model[0].outputs):
            print(f"[RDK_YOLO] output[{idx}] shape={output.properties.shape} dtype={output.properties.dtype}")

    def _parse_hw(self, shape: list[int]) -> tuple[int, int]:
        if len(shape) >= 4:
            return int(shape[2]), int(shape[3])
        if len(shape) >= 2:
            return int(shape[-2]), int(shape[-1])
        return 640, 640

    def _make_anchors(self, stride: int) -> np.ndarray:
        grid_w = self.input_w // stride
        grid_h = self.input_h // stride
        xs = np.tile(np.arange(0.5, grid_w + 0.5, 1.0), reps=grid_h)
        ys = np.repeat(np.arange(0.5, grid_h + 0.5, 1.0), grid_w)
        return np.stack([xs, ys], axis=1).astype(np.float32)

    def bgr_to_nv12(self, frame: np.ndarray) -> np.ndarray:
        resized = self._letterbox(frame)
        area = self.input_h * self.input_w
        yuv420p = cv2.cvtColor(resized, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
        y = yuv420p[:area]
        uv_planar = yuv420p[area:].reshape((2, area // 4))
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
        nv12 = np.empty_like(yuv420p)
        nv12[:area] = y
        nv12[area:] = uv_packed
        return nv12

    def _letterbox(self, frame: np.ndarray) -> np.ndarray:
        source_h, source_w = frame.shape[:2]
        scale = min(self.input_w / source_w, self.input_h / source_h)
        new_w = max(1, int(round(source_w * scale)))
        new_h = max(1, int(round(source_h * scale)))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        canvas = np.zeros((self.input_h, self.input_w, 3), dtype=np.uint8)
        pad_x = (self.input_w - new_w) // 2
        pad_y = (self.input_h - new_h) // 2
        canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = resized
        self.letterbox = LetterboxInfo(scale, pad_x, pad_y, source_w, source_h)
        return canvas

    def predict(self, frame: np.ndarray) -> list[Detection]:
        outputs = self.model[0].forward(self.bgr_to_nv12(frame))
        tensors = [np.asarray(output.buffer) for output in outputs]
        return self._postprocess(tensors)

    def _postprocess(self, outputs: list[np.ndarray]) -> list[Detection]:
        if len(outputs) < 6:
            raise RuntimeError(f"YOLO11 detector expects at least 6 outputs, got {len(outputs)}")

        all_boxes: list[np.ndarray] = []
        all_scores: list[np.ndarray] = []
        all_ids: list[np.ndarray] = []

        for level, stride in enumerate(self.strides):
            cls = outputs[level * 2].reshape(-1, self.num_classes)
            bbox = outputs[level * 2 + 1].reshape(-1, 64)
            max_logits = np.max(cls, axis=1)
            valid = np.flatnonzero(max_logits >= self.conf_logit)
            if valid.size == 0:
                continue

            ids = np.argmax(cls[valid], axis=1)
            scores = sigmoid(max_logits[valid])
            dist = np.sum(
                softmax(bbox[valid].reshape(-1, 4, 16), axis=2) * self.dfl_weights,
                axis=2,
            )
            anchors = self.anchors[stride][valid]
            x1y1 = anchors - dist[:, 0:2]
            x2y2 = anchors + dist[:, 2:4]
            boxes = np.hstack([x1y1, x2y2]) * stride

            all_boxes.append(boxes.astype(np.float32))
            all_scores.append(scores.astype(np.float32))
            all_ids.append(ids.astype(np.int32))

        if not all_boxes:
            return []

        boxes = np.concatenate(all_boxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        ids = np.concatenate(all_ids, axis=0)
        nms_boxes = boxes.copy()
        nms_boxes[:, 2] = boxes[:, 2] - boxes[:, 0]
        nms_boxes[:, 3] = boxes[:, 3] - boxes[:, 1]
        keep = cv2.dnn.NMSBoxes(nms_boxes.tolist(), scores.tolist(), self.conf, self.iou)
        if len(keep) == 0:
            return []

        detections: list[Detection] = []
        for idx in np.asarray(keep).reshape(-1):
            bbox = self._restore_box(boxes[int(idx)])
            detections.append(Detection(int(ids[int(idx)]), float(scores[int(idx)]), bbox))
        return detections

    def _restore_box(self, box: np.ndarray) -> tuple[int, int, int, int]:
        info = self.letterbox
        x1, y1, x2, y2 = box.astype(np.float32)
        x1 = (x1 - info.pad_x) / info.scale
        x2 = (x2 - info.pad_x) / info.scale
        y1 = (y1 - info.pad_y) / info.scale
        y2 = (y2 - info.pad_y) / info.scale
        x1 = int(np.clip(round(x1), 0, info.source_w - 1))
        x2 = int(np.clip(round(x2), 0, info.source_w - 1))
        y1 = int(np.clip(round(y1), 0, info.source_h - 1))
        y2 = int(np.clip(round(y2), 0, info.source_h - 1))
        return x1, y1, x2, y2


def draw_detection(frame: np.ndarray, det: Detection, class_names: list[str]) -> None:
    x1, y1, x2, y2 = det.bbox
    color = COLORS[det.class_id % len(COLORS)]
    label_name = class_names[det.class_id] if det.class_id < len(class_names) else str(det.class_id)
    label = f"{label_name}: {det.score:.2f}"
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
    label_y = y1 - 8 if y1 - 8 > label_h else y1 + label_h + 8
    cv2.rectangle(frame, (x1, label_y - label_h - 4), (x1 + label_w + 4, label_y + 4), color, cv2.FILLED)
    cv2.putText(frame, label, (x1 + 2, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1, cv2.LINE_AA)


class AnimalDetectNode(Node):
    def __init__(self) -> None:
        super().__init__("animal_detect_node")
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("class_names", ",".join(DEFAULT_NAMES))
        self.declare_parameter("camera_index", 1)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("crop_size", 0)
        self.declare_parameter("conf", 0.3)
        self.declare_parameter("iou", 0.5)
        self.declare_parameter("rotate_180", True)
        self.declare_parameter("show", True)
        self.declare_parameter("publish_empty", False)
        self.declare_parameter("enable_topic", "/pic_enable")
        self.declare_parameter("count_topic", "/pic_cnt")
        self.declare_parameter("detections_topic", "/vision/detections")

        model_path = str(self.get_parameter("model_path").value)
        class_names = parse_names(str(self.get_parameter("class_names").value))
        if not Path(model_path).exists():
            self.get_logger().warning(f"model file does not exist yet: {model_path}")

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.crop_size = int(self.get_parameter("crop_size").value)
        self.rotate_180 = bool(self.get_parameter("rotate_180").value)
        self.show = bool(self.get_parameter("show").value)
        self.publish_empty = bool(self.get_parameter("publish_empty").value)
        camera_index = int(self.get_parameter("camera_index").value)
        enable_topic = str(self.get_parameter("enable_topic").value)
        count_topic = str(self.get_parameter("count_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)

        self.detector = Yolo11BpuDetector(
            model_path=model_path,
            class_names=class_names,
            conf=float(self.get_parameter("conf").value),
            iou=float(self.get_parameter("iou").value),
        )
        self.class_names = class_names
        self.camera = srcampy.Camera()
        self.camera.open_cam(camera_index, -1, -1, self.width, self.height, self.height, self.width)

        self.enable = 0
        self.lock = threading.Lock()
        self.picture_pub = self.create_publisher(Int32MultiArray, count_topic, 10)
        self.detections_pub = self.create_publisher(String, detections_topic, 10)
        self.enable_sub = self.create_subscription(Int32, enable_topic, self.enable_callback, 10)
        self.thread = threading.Thread(target=self.loop_task, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"YOLO11 BPU detection ready: classes={self.class_names}, "
            f"publishing detections to {detections_topic}"
        )

    def enable_callback(self, msg: Int32) -> None:
        with self.lock:
            self.enable = int(msg.data)
        self.get_logger().info(f"vision inference {'enabled' if msg.data else 'disabled'}")

    def loop_task(self) -> None:
        while rclpy.ok():
            with self.lock:
                enabled = self.enable
            if not enabled:
                time.sleep(0.02)
                continue

            frame = self._read_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            crop = self._center_crop(frame)
            detections = self.detector.predict(crop)
            counts = [0] * len(self.class_names)
            detection_items = []
            crop_h, crop_w = crop.shape[:2]
            frame_center_x = crop_w // 2
            frame_center_y = crop_h // 2
            for det in detections:
                if 0 <= det.class_id < len(counts):
                    counts[det.class_id] += 1
                x1, y1, x2, y2 = det.bbox
                center_x = int(round((x1 + x2) / 2.0))
                center_y = int(round((y1 + y2) / 2.0))
                class_name = (
                    self.class_names[det.class_id]
                    if 0 <= det.class_id < len(self.class_names)
                    else str(det.class_id)
                )
                detection_items.append(
                    {
                        "class_id": det.class_id,
                        "class_name": class_name,
                        "score": det.score,
                        "bbox": [x1, y1, x2, y2],
                        "center": [center_x, center_y],
                        "offset": [center_x - frame_center_x, center_y - frame_center_y],
                        "area": max(0, x2 - x1) * max(0, y2 - y1),
                    }
                )
                if self.show:
                    draw_detection(crop, det, self.class_names)

            if self.publish_empty or any(counts):
                msg = Int32MultiArray()
                msg.layout.dim = [MultiArrayDimension(label="classes", size=len(counts), stride=len(counts))]
                msg.data = counts
                self.picture_pub.publish(msg)

            if self.publish_empty or detection_items:
                msg = String()
                msg.data = json.dumps(
                    {
                        "found": bool(detection_items),
                        "frame_size": [crop_w, crop_h],
                        "count": len(detection_items),
                        "detections": detection_items,
                    },
                    ensure_ascii=False,
                )
                self.detections_pub.publish(msg)

            if detections:
                summary = ", ".join(f"{self.class_names[i]}={count}" for i, count in enumerate(counts) if count)
                self.get_logger().info(f"detections: {summary}")

            if self.show:
                cv2.imshow("RDK YOLO11", crop)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    def _read_frame(self) -> np.ndarray | None:
        nv12_img = self.camera.get_img(2, self.width, self.height)
        if nv12_img is None:
            self.get_logger().warning("failed to get camera image")
            return None
        nv12_data = np.frombuffer(nv12_img, dtype=np.uint8).reshape(self.height * 3 // 2, self.width)
        frame = cv2.cvtColor(nv12_data, cv2.COLOR_YUV2BGR_NV12)
        if self.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def _center_crop(self, frame: np.ndarray) -> np.ndarray:
        if self.crop_size <= 0:
            return frame
        h, w = frame.shape[:2]
        side = min(self.crop_size, h, w)
        cx, cy = w // 2, h // 2
        x1 = max(0, cx - side // 2)
        y1 = max(0, cy - side // 2)
        return frame[y1 : y1 + side, x1 : x1 + side]

    def destroy_node(self) -> bool:
        self.camera.close_cam()
        if self.show:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AnimalDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
