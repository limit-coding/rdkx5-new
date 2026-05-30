"""
mission_vision_v2.py — 替换 /home/sunrise/project/camera/camera/mission_vision.py
变更：
  1. YOLO模型换成 yolo11n-cls trained on CIFAR-100 (100类动物/物体)
  2. 推理前自动检测靶板白色圆心，裁剪到目标图区域再送模型
  3. 多帧确认机制（同一标签连续 cls_confirm_frames 帧才发布）
"""

import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO

MODEL_PATH = "/home/sunrise/model/yolo11n_cifar100_cls.pt"
QR_CONFIRM  = 3
CLS_CONFIRM = 5
IMGSZ       = 224
CONF_THRESH = 0.30  # 最低置信度


class MissionVision(Node):
    def __init__(self):
        super().__init__("mission_vision")

        self.model = YOLO(MODEL_PATH)

        self.pub_qr     = self.create_publisher(String, "/qr_task",      10)
        self.pub_target = self.create_publisher(String, "/target_event", 10)

        self.sub = self.create_subscription(
            CompressedImage, "/image", self._cb, 1
        )

        # 状态机
        self.phase = "qr"         # "qr" → "classify"
        self.qr_data     = None
        self.qr_count    = 0
        self.cls_label   = None
        self.cls_count   = 0

        self.get_logger().info("MissionVision v2 ready")

    # ------------------------------------------------------------------
    def _cb(self, msg: CompressedImage):
        arr   = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        if self.phase == "qr":
            self._phase_qr(frame)
        else:
            self._phase_classify(frame)

    # ------------------------------------------------------------------
    def _phase_qr(self, frame):
        detector = cv2.QRCodeDetector()
        data, _, _ = detector.detectAndDecode(frame)
        if not data:
            self.qr_count = 0
            return

        if data == self.qr_data:
            self.qr_count += 1
        else:
            self.qr_data  = data
            self.qr_count = 1

        if self.qr_count >= QR_CONFIRM:
            self.get_logger().info(f"QR confirmed: {data}")
            msg = String()
            msg.data = data
            self.pub_qr.publish(msg)
            self.phase = "classify"

    # ------------------------------------------------------------------
    def _phase_classify(self, frame):
        crop = self._crop_target(frame)
        if crop is None:
            # 找不到靶板，用全图中心裁剪兜底
            h, w = frame.shape[:2]
            s = min(h, w) // 3
            cx, cy = w // 2, h // 2
            crop = frame[cy - s:cy + s, cx - s:cx + s]

        result = self.model(crop, imgsz=IMGSZ, verbose=False)[0]
        conf   = float(result.probs.top1conf)
        label  = result.names[result.probs.top1]

        if conf < CONF_THRESH:
            return

        if label == self.cls_label:
            self.cls_count += 1
        else:
            self.cls_label = label
            self.cls_count = 1

        if self.cls_count >= CLS_CONFIRM:
            self.get_logger().info(f"Target confirmed: {label} ({conf:.1%})")
            msg = String()
            msg.data = label
            self.pub_target.publish(msg)
            # 重置，允许识别下一个目标
            self.cls_count = 0

    # ------------------------------------------------------------------
    def _crop_target(self, frame):
        """
        在靶板图像中找到白色内圆，返回裁剪后的目标区域。
        失败返回 None。
        """
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 白色：低饱和度 + 高亮度
        mask = cv2.inRange(hsv, (0, 0, 170), (180, 45, 255))

        k    = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best, best_score = None, 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            (_, _), r = cv2.minEnclosingCircle(cnt)
            circularity = area / (np.pi * r * r + 1e-6)
            score = area * circularity
            if score > best_score:
                best, best_score = cnt, score

        if best is None:
            return None

        M  = cv2.moments(best)
        if M["m00"] < 1:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        r  = int(np.sqrt(cv2.contourArea(best) / np.pi))

        # CIFAR图大约占白圆半径的80%
        crop_r = max(32, int(r * 0.85))
        h, w   = frame.shape[:2]
        x1 = max(0, cx - crop_r)
        y1 = max(0, cy - crop_r)
        x2 = min(w, cx + crop_r)
        y2 = min(h, cy + crop_r)

        if (x2 - x1) < 32 or (y2 - y1) < 32:
            return None
        return frame[y1:y2, x1:x2]


def main():
    rclpy.init()
    node = MissionVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
