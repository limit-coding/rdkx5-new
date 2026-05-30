import json
from typing import List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, Int32MultiArray, String

from camera.qr_detector import (
    QrDetection,
    QrDetector,
    decode_jpeg_bytes,
    format_text_for_log,
)


class QrDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("qr_detector")

        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("text_topic", "/qr_code/text")
        self.declare_parameter("offset_topic", "/qr_code/offset")
        self.declare_parameter("json_topic", "/qr_code/result")
        self.declare_parameter("enable_topic", "/qr_enable")
        self.declare_parameter("min_area", 100)
        self.declare_parameter("allow_empty", False)
        self.declare_parameter("log_text", False)
        self.declare_parameter("log_every_frame", False)

        image_topic = self.get_parameter("image_topic").value
        text_topic = self.get_parameter("text_topic").value
        offset_topic = self.get_parameter("offset_topic").value
        json_topic = self.get_parameter("json_topic").value
        enable_topic = self.get_parameter("enable_topic").value
        min_area = int(self.get_parameter("min_area").value)
        allow_empty = bool(self.get_parameter("allow_empty").value)
        self.log_text = bool(self.get_parameter("log_text").value)

        self.detector = QrDetector(min_area=min_area, require_text=not allow_empty)
        self.log_every_frame = bool(self.get_parameter("log_every_frame").value)
        self.frame_count = 0
        self.last_text = ""
        self.enabled = True

        self.text_pub = self.create_publisher(String, text_topic, 10)
        self.offset_pub = self.create_publisher(Int32MultiArray, offset_topic, 10)
        self.json_pub = self.create_publisher(String, json_topic, 10)
        self.image_sub = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_callback,
            10,
        )
        self.enable_sub = self.create_subscription(
            Int32,
            enable_topic,
            self.enable_callback,
            10,
        )

        self.get_logger().info(
            f"QR detector subscribed to {image_topic}; publishing {text_topic}, "
            f"{offset_topic}, {json_topic}"
        )

    def enable_callback(self, msg: Int32) -> None:
        enabled = msg.data != 0
        if self.enabled != enabled:
            self.enabled = enabled
            self.get_logger().info(f"QR detector enabled={self.enabled}")

    def image_callback(self, msg: CompressedImage) -> None:
        if not self.enabled:
            return

        self.frame_count += 1
        frame = decode_jpeg_bytes(bytes(msg.data))
        if frame is None:
            self.get_logger().warn("Failed to decode compressed image")
            return

        detections = self.detector.detect(frame)
        if not detections:
            self._publish_not_found()
            if self.log_every_frame and self.frame_count % 30 == 0:
                self.get_logger().info("No QR code detected")
            return

        detection = self._pick_best_detection(detections)
        self._publish_detection(detection, len(detections))

    def _pick_best_detection(self, detections: List[QrDetection]) -> QrDetection:
        return max(detections, key=lambda item: item.area)

    def _publish_not_found(self) -> None:
        offset_msg = Int32MultiArray()
        offset_msg.data = [0, 0, 0, 0, 0, 0]
        self.offset_pub.publish(offset_msg)

    def _publish_detection(self, detection: QrDetection, count: int) -> None:
        text_msg = String()
        text_msg.data = detection.text
        self.text_pub.publish(text_msg)

        offset_msg = Int32MultiArray()
        offset_msg.data = [
            1,
            detection.center_x,
            detection.center_y,
            detection.offset_x,
            detection.offset_y,
            detection.area,
        ]
        self.offset_pub.publish(offset_msg)

        result_msg = String()
        result_msg.data = json.dumps(
            {
                "found": True,
                "count": count,
                "text": detection.text,
                "center": [detection.center_x, detection.center_y],
                "offset": [detection.offset_x, detection.offset_y],
                "area": detection.area,
                "points": detection.points.astype(int).tolist(),
            },
            ensure_ascii=False,
        )
        self.json_pub.publish(result_msg)

        if self.log_every_frame or detection.text != self.last_text:
            self.get_logger().info(
                f"QR text={format_text_for_log(detection.text, self.log_text)} "
                f"center=({detection.center_x},"
                f"{detection.center_y}) offset=({detection.offset_x},"
                f"{detection.offset_y}) area={detection.area}"
            )
            self.last_text = detection.text


def main() -> None:
    rclpy.init()
    node = QrDetectorNode()
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
