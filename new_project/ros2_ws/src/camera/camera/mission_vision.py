from __future__ import annotations

import json
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
try:
    from hobot_vio import libsrcampy as srcampy
    HAS_HOBOT = True
except ImportError:
    srcampy = None
    HAS_HOBOT = False
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String

from camera.cifar100_cls_enable import Cifar100BpuClassifier, TargetCenterCropper
from camera.qr_detector import decode_jpeg_bytes

DEFAULT_CLS_MODEL_PATH = "/home/sunrise/project/camera/resource/cifar100_cls.onnx"
DEFAULT_CLS_NAMES_PATH = "/home/sunrise/project/camera/resource/cifar100_names.txt"


def _split_qr_tokens(text: str) -> list[str]:
    normalized = text
    for old, new in {"，": ",", "、": ",", "：": ":", "；": ";"}.items():
        normalized = normalized.replace(old, new)
    for ch in ",;|:=/\\[]{}()\"'":
        normalized = normalized.replace(ch, " ")
    return [token.strip() for token in normalized.split() if token.strip()]


def parse_qr_mission(text: str) -> tuple[str, str, str] | None:
    ignored_keys = {
        "class", "class1", "class2", "target", "target1", "target2",
        "side", "landing", "landing_side", "land", "qr", "task",
    }
    classes: list[str] = []
    landing_side = ""
    for token in _split_qr_tokens(text):
        lower = token.lower()
        if lower in {"left", "right"}:
            landing_side = lower
        elif lower not in ignored_keys:
            classes.append(token)
    if len(classes) < 2 or not landing_side:
        return None
    return classes[0], classes[1], landing_side


class MissionVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_vision_node")
        self.declare_parameter("cls_model_path", DEFAULT_CLS_MODEL_PATH)
        self.declare_parameter("cls_names_path", DEFAULT_CLS_NAMES_PATH)
        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_index", 1)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("rotate_180", True)
        self.declare_parameter("show", False)
        self.declare_parameter("cls_conf", 0.2)
        self.declare_parameter("qr_confirm_frames", 3)
        self.declare_parameter("target_confirm_frames", 3)
        self.declare_parameter("target_release_frames", 5)
        self.declare_parameter("qr_task_topic", "/qr_task")
        self.declare_parameter("target_event_topic", "/target_event")
        self.declare_parameter("skip_qr", False)
        self.declare_parameter("wait_ring_after_qr", False)
        self.declare_parameter("ring_confirm_frames", 3)
        self.declare_parameter("ring_min_radius_ratio", 0.06)
        self.declare_parameter("ring_max_radius_ratio", 0.45)
        self.declare_parameter("ignored_target_labels", ["cloud", "keyboard"])
        self.declare_parameter("target_rank_k", 20)
        self.declare_parameter("target_rank_min_score", 0.004)
        self.declare_parameter("target_collect_frames", 5)
        self.declare_parameter("qr_target_min_score", 0.1)
        self.declare_parameter("require_ring", True)
        self.declare_parameter("ring_hough_param2", 28)
        self.declare_parameter("ring_white_s_max", 55)
        self.declare_parameter("ring_white_v_min", 160)

        cls_model_path = str(self.get_parameter("cls_model_path").value)
        cls_names_path = str(self.get_parameter("cls_names_path").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.rotate_180 = bool(self.get_parameter("rotate_180").value)
        self.show = bool(self.get_parameter("show").value)
        self.cls_conf = float(self.get_parameter("cls_conf").value)
        self.qr_confirm_frames = max(1, int(self.get_parameter("qr_confirm_frames").value))
        self.target_confirm_frames = max(1, int(self.get_parameter("target_confirm_frames").value))
        self.target_release_frames = max(1, int(self.get_parameter("target_release_frames").value))
        self.wait_ring_after_qr = bool(self.get_parameter("wait_ring_after_qr").value)
        self.ring_confirm_frames = max(1, int(self.get_parameter("ring_confirm_frames").value))
        self.ring_min_radius_ratio = float(self.get_parameter("ring_min_radius_ratio").value)
        self.ring_max_radius_ratio = float(self.get_parameter("ring_max_radius_ratio").value)
        self.ignored_target_labels = {
            str(label).strip().lower()
            for label in self.get_parameter("ignored_target_labels").value
            if str(label).strip()
        }
        self.target_rank_k = max(1, int(self.get_parameter("target_rank_k").value))
        self.target_rank_min_score = max(0.0, float(self.get_parameter("target_rank_min_score").value))
        self.target_collect_frames = max(1, int(self.get_parameter("target_collect_frames").value))
        self.qr_target_min_score = max(0.0, float(self.get_parameter("qr_target_min_score").value))
        self.require_ring = bool(self.get_parameter("require_ring").value)
        self.ring_hough_param2 = int(self.get_parameter("ring_hough_param2").value)
        self.ring_white_s_max = int(self.get_parameter("ring_white_s_max").value)
        self.ring_white_v_min = int(self.get_parameter("ring_white_v_min").value)

        if not Path(cls_model_path).exists():
            self.get_logger().warning(f"cls model not found: {cls_model_path}")
        self.classifier = Cifar100BpuClassifier(cls_model_path, cls_names_path)
        self.cropper = TargetCenterCropper()
        self.qr_detector = cv2.QRCodeDetector()

        self.image_topic = str(self.get_parameter("image_topic").value).strip()
        self.camera = None
        if self.image_topic:
            self.create_subscription(CompressedImage, self.image_topic, self.image_callback, 10)
        else:
            camera_index = int(self.get_parameter("camera_index").value)
            self.camera = srcampy.Camera()
            self.camera.open_cam(camera_index, -1, -1, self.width, self.height, self.height, self.width)

        self.qr_task_pub = self.create_publisher(
            String, str(self.get_parameter("qr_task_topic").value), 10
        )
        self.target_event_pub = self.create_publisher(
            String, str(self.get_parameter("target_event_topic").value), 10
        )
        self.debug_pub = self.create_publisher(String, "/vision/debug", 1)

        skip_qr = bool(self.get_parameter("skip_qr").value)
        self.phase = "cls" if skip_qr else "qr"
        self.last_qr_text = ""
        self.qr_stable_frames = 0
        self.target_candidate = ""
        self.target_candidate_frames = 0
        self.latched_target = ""
        self.target_empty_frames = 0
        self.ring_seen_frames = 0
        self.last_ring_log_time = 0.0
        self.qr_target_classes: set[str] = set()
        self._pic_enabled = False
        self._cached_label = ""
        self._cached_score = 0.0
        # 滑动窗口：持续缓存最近10帧里QR匹配的结果，02到来时直接取
        self._recent_qr_results: deque[tuple[str, float]] = deque(maxlen=10)
        self._collect_count = 0
        self._collect_results: list[tuple[str, float]] = []
        self._collect_fallback_results: list[tuple[str, float]] = []
        self.create_subscription(Int32, "/pic_enable", self._on_pic_enable, 10)

        src = "direct camera" if self.camera is not None else f"topic {self.image_topic}"
        self.get_logger().info(f"mission vision ready: {self.phase.upper()} phase ({src})")

        if self.camera is not None:
            t = threading.Thread(target=self._loop_direct_camera, daemon=True)
            t.start()

    def _loop_direct_camera(self) -> None:
        while rclpy.ok():
            nv12 = self.camera.get_img(2, self.width, self.height)
            if nv12 is None:
                time.sleep(0.05)
                continue
            data = np.frombuffer(nv12, dtype=np.uint8).reshape(self.height * 3 // 2, self.width)
            frame = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_NV12)
            if self.rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            self._process_frame(frame)

    def image_callback(self, msg: CompressedImage) -> None:
        frame = decode_jpeg_bytes(bytes(msg.data))
        if frame is None:
            return
        self._process_frame(frame)

    def _process_frame(self, frame: np.ndarray) -> None:
        if self.phase == "qr":
            self._process_qr(frame)
        elif self.phase == "wait_ring":
            self._process_ring_gate(frame)
        else:
            self._process_cls(frame)
        if self.show:
            cv2.imshow("Mission Vision", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rclpy.shutdown()

    def _on_pic_enable(self, msg: Int32) -> None:
        enabled = bool(int(msg.data))
        if enabled and not self._pic_enabled:
            self.latched_target = ""
            self.target_candidate = ""
            self.target_candidate_frames = 0
            self.target_empty_frames = 0
            self._collect_results = []
            self._collect_fallback_results = []
            self._cached_label = ""
            self._cached_score = 0.0
            self._recent_qr_results.clear()
            self._collect_count = self.target_collect_frames
            self.get_logger().info(
                f"pic_enable=1 -> 忽略触发前缓存，开始采集触发后{self.target_collect_frames}帧..."
            )
        self._pic_enabled = enabled

    def _process_qr(self, frame: np.ndarray) -> None:
        text, _, _ = self.qr_detector.detectAndDecode(frame)
        text = text.strip()
        mission = parse_qr_mission(text) if text else None

        if mission is None:
            self.last_qr_text = ""
            self.qr_stable_frames = 0
            return

        if text == self.last_qr_text:
            self.qr_stable_frames += 1
        else:
            self.last_qr_text = text
            self.qr_stable_frames = 1

        if self.qr_stable_frames < self.qr_confirm_frames:
            return

        class1, class2, landing_side = mission
        self.qr_target_classes = {class1.strip().lower(), class2.strip().lower()}
        self._cached_label = ""
        self._cached_score = 0.0
        self._recent_qr_results.clear()
        payload = {
            "valid": True,
            "confirmed": True,
            "stable_frames": self.qr_stable_frames,
            "class1": class1,
            "class2": class2,
            "landing_side": landing_side,
        }
        self.qr_task_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if self.wait_ring_after_qr:
            self.phase = "wait_ring"
            self.ring_seen_frames = 0
            self.get_logger().info(
                f"QR confirmed: class1={class1} class2={class2} landing={landing_side}; "
                "waiting for ring before CLS"
            )
        else:
            self.phase = "cls"
            self.get_logger().info(
                f"QR confirmed: class1={class1} class2={class2} landing={landing_side}; switching to CLS"
            )

    def _process_ring_gate(self, frame: np.ndarray) -> None:
        found_ring = self._detect_ring_marker(frame)
        if found_ring:
            self.ring_seen_frames += 1
        else:
            self.ring_seen_frames = 0

        if self.ring_seen_frames >= self.ring_confirm_frames:
            self.phase = "cls"
            self.target_candidate = ""
            self.target_candidate_frames = 0
            self.latched_target = ""
            self.target_empty_frames = 0
            self.get_logger().info("Ring confirmed; switching to CLS")
            return

        now = time.time()
        if now - self.last_ring_log_time >= 1.0:
            self.last_ring_log_time = now
            self.get_logger().info(
                f"waiting for ring: {self.ring_seen_frames}/{self.ring_confirm_frames}"
            )

    def _detect_ring_marker(self, frame: np.ndarray) -> bool:
        if frame.size == 0:
            return False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        h, w = gray.shape
        min_dim = min(h, w)
        min_radius = max(12, int(min_dim * self.ring_min_radius_ratio))
        max_radius = max(min_radius + 1, int(min_dim * self.ring_max_radius_ratio))

        if self._detect_ring_by_hough(blurred, gray, min_radius, max_radius):
            return True
        return self._detect_ring_by_contours(blurred, gray, min_radius, max_radius)

    def _detect_ring_by_hough(
        self,
        blurred: np.ndarray,
        gray: np.ndarray,
        min_radius: int,
        max_radius: int,
    ) -> bool:
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(30, min(gray.shape) // 3),
            param1=90,
            param2=28,
            minRadius=min_radius,
            maxRadius=max_radius,
        )
        if circles is None:
            return False
        for cx, cy, radius in np.round(circles[0, :]).astype(int):
            if self._has_ring_contrast(gray, cx, cy, radius):
                return True
        return False

    def _detect_ring_by_contours(
        self,
        blurred: np.ndarray,
        gray: np.ndarray,
        min_radius: int,
        max_radius: int,
    ) -> bool:
        edges = cv2.Canny(blurred, 40, 130)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        min_area = np.pi * min_radius * min_radius * 0.25

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            if radius < min_radius or radius > max_radius:
                continue
            circularity = area / (np.pi * radius * radius) if radius > 0 else 0.0
            if circularity < 0.35:
                continue
            if self._has_ring_contrast(gray, int(cx), int(cy), int(radius)):
                return True
        return False

    def _has_ring_contrast(self, gray: np.ndarray, cx: int, cy: int, radius: int) -> bool:
        h, w = gray.shape
        if radius <= 0:
            return False
        margin = int(radius * 1.15)
        if cx - margin < 0 or cy - margin < 0 or cx + margin >= w or cy + margin >= h:
            return False

        yy, xx = np.ogrid[:h, :w]
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        inner_mask = dist <= radius * 0.55
        ring_mask = (dist >= radius * 0.70) & (dist <= radius * 1.05)
        outer_mask = (dist >= radius * 1.10) & (dist <= radius * 1.35)

        if not inner_mask.any() or not ring_mask.any() or not outer_mask.any():
            return False

        inner_mean = float(gray[inner_mask].mean())
        ring_mean = float(gray[ring_mask].mean())
        outer_mean = float(gray[outer_mask].mean())
        contrast = max(abs(ring_mean - inner_mean), abs(ring_mean - outer_mean))
        return contrast >= 18.0

    def _detect_ring(self, frame: np.ndarray) -> bool:
        """Return True when a white circular ring is visible in the frame."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, self.ring_white_v_min]),
            np.array([180, self.ring_white_s_max, 255]),
        )
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        masked = cv2.bitwise_and(gray, gray, mask=white_mask)
        blurred = cv2.GaussianBlur(masked, (9, 9), 0)
        h, w = gray.shape
        min_dim = min(h, w)
        min_r = max(12, int(min_dim * self.ring_min_radius_ratio))
        max_r = max(min_r + 1, int(min_dim * self.ring_max_radius_ratio))
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(30, min_dim // 3),
            param1=90, param2=self.ring_hough_param2,
            minRadius=min_r, maxRadius=max_r,
        )
        return circles is not None

    def _process_cls(self, frame: np.ndarray) -> None:
        if self.require_ring and not self._detect_ring(frame):
            return
        crop = self.cropper.crop(frame)
        qr_targets = self.qr_target_classes if self.qr_target_classes else None
        topk = self.classifier.predict_topk(crop, k=max(self.target_rank_k, 5), qr_targets=qr_targets)

        label, score, reason = self._choose_target_label(topk)
        is_qr_target = reason.startswith("qr-target-rank=")
        # Publish debug info (top-3 + QR target rank) for external monitoring
        top3 = "  ".join(f"{n}:{s*100:.0f}%" for _, n, s in topk[:3])
        qr_info = f"  QR={label}:{score*100:.0f}%@{reason}" if is_qr_target else "  no-qr-match"
        self.debug_pub.publish(String(data=top3 + qr_info))
        # 滑动窗口：持续记录QR匹配结果，供02到来时使用
        if is_qr_target:
            self._recent_qr_results.append((label, score))
        # 缓存历史最高分（只用于日志）
        if is_qr_target and score > self._cached_score:
            self._cached_label = label
            self._cached_score = score
        # 每2秒打一次中间识别结果，方便调试
        now = time.time()
        if not hasattr(self, '_last_cls_log_t') or now - self._last_cls_log_t >= 2.0:
            self._last_cls_log_t = now
            cache_hint = f"  cache={self._cached_label}:{self._cached_score*100:.0f}%" if self._cached_label else "  cache=none"
            self.get_logger().info(f"CLS  {top3}{cache_hint}")
        # 飞控未下达识别命令时不发结果
        if not self._pic_enabled:
            return
        # 02到来后采集最近5帧，从中选最优QR匹配结果发布
        if self._collect_count > 0:
            frame_idx = self.target_collect_frames - self._collect_count + 1
            if is_qr_target:
                self._collect_results.append((label, score))
            elif label:
                self._collect_fallback_results.append((label, score))
            qr_hint = f"  QR={label}:{score*100:.0f}%" if is_qr_target else "  no-qr-match"
            self.get_logger().info(
                f"采集[{frame_idx}/{self.target_collect_frames}] top1={topk[0][1]}:{topk[0][2]*100:.0f}%{qr_hint}"
            )
            self._collect_count -= 1
            if self._collect_count == 0:
                if self._collect_results:
                    best_label, best_score = max(self._collect_results, key=lambda x: x[1])
                    self.target_event_pub.publish(String(data=best_label))
                    self.latched_target = best_label
                    self.get_logger().info(
                        f"{self.target_collect_frames}帧采集完 -> 有QR匹配，发布: {best_label} ({best_score:.3f})"
                    )
                elif self._collect_fallback_results:
                    best_label, best_score = max(self._collect_fallback_results, key=lambda x: x[1])
                    self.target_event_pub.publish(String(data=best_label))
                    self.latched_target = best_label
                    self.get_logger().info(
                        f"{self.target_collect_frames}帧采集完 -> 无QR匹配，发布非匹配结果: {best_label} ({best_score:.3f})"
                    )
                else:
                    self.target_event_pub.publish(String(data="no_match"))
                    self.latched_target = "no_match"
                    self.get_logger().info(
                        f"{self.target_collect_frames}帧采集完 -> 无有效分类，发布: no_match"
                    )
            return
        # When QR targets are known, only confirm QR-matched labels
        if self.qr_target_classes and not is_qr_target:
            self.target_candidate = ""
            self.target_candidate_frames = 0
            self.target_empty_frames += 1
            if self.target_empty_frames >= self.target_release_frames:
                self.latched_target = ""
            return
        if not label or (not is_qr_target and score < self.cls_conf):
            self.target_candidate = ""
            self.target_candidate_frames = 0
            self.target_empty_frames += 1
            if self.target_empty_frames >= self.target_release_frames:
                self.latched_target = ""
            return

        if label.lower() in self.ignored_target_labels:
            self.target_candidate = ""
            self.target_candidate_frames = 0
            self.target_empty_frames += 1
            if self.target_empty_frames >= self.target_release_frames:
                self.latched_target = ""
            return

        self.target_empty_frames = 0

        if label == self.target_candidate:
            self.target_candidate_frames += 1
        else:
            self.target_candidate = label
            self.target_candidate_frames = 1

        if self.target_candidate_frames < self.target_confirm_frames:
            return
        if label == self.latched_target:
            return

        self.target_event_pub.publish(String(data=label))
        self.latched_target = label
        self.get_logger().info(f"target event published: {label} (score={score:.3f}, {reason})")

    def _choose_target_label(self, topk: list[tuple[int, str, float]]) -> tuple[str, float, str]:
        if not topk:
            return "", 0.0, "empty"

        for rank, (_, raw_label, raw_score) in enumerate(topk[: self.target_rank_k], 1):
            label = str(raw_label).strip()
            score = float(raw_score)
            if score < max(self.target_rank_min_score, self.qr_target_min_score):
                continue
            matched = self._match_qr_target(label)
            if matched:
                return matched, score, f"qr-target-rank={rank}"

        for _, raw_label, raw_score in topk:
            label = str(raw_label).strip()
            if label.lower() in self.ignored_target_labels:
                continue
            return label, float(raw_score), "top1-filtered"

        return "", 0.0, "ignored"

    def _match_qr_target(self, label: str) -> str:
        label_norm = str(label).strip().lower()
        if not label_norm:
            return ""
        for target in self.qr_target_classes:
            if target and (target == label_norm or target in label_norm or label_norm in target):
                return target
        return ""

    def destroy_node(self) -> bool:
        if self.camera is not None:
            self.camera.close_cam()
        if self.show:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MissionVisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
