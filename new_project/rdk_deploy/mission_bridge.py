#!/usr/bin/env python3
"""
mission_vision → task_state_machine 桥接节点
/qr_task (JSON)     → /qr_code/text (每2秒重发直到收到QR)
/target_event (str) → /vision/detections (JSON)
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionBridge(Node):
    def __init__(self):
        super().__init__("mission_bridge")
        self.qr_text_pub    = self.create_publisher(String, "/qr_code/text", 10)
        self.detections_pub = self.create_publisher(String, "/vision/detections", 10)

        self.create_subscription(String, "/qr_task",      self.qr_task_cb,     10)
        self.create_subscription(String, "/target_event", self.target_event_cb, 10)

        self._last_qr_text = ""
        self.create_timer(2.0, self._republish_qr)

        self.get_logger().info("mission_bridge ready")

    def qr_task_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if not data.get("confirmed"):
                return
            c1   = data.get("class1", "")
            c2   = data.get("class2", "")
            side = data.get("landing_side", "")
            text = f"{c1},{c2},{side}"
            self._last_qr_text = text
            self.qr_text_pub.publish(String(data=text))
            self.get_logger().info(f"bridge QR → {text}")
        except Exception as e:
            self.get_logger().error(f"qr_task parse error: {e}")

    def _republish_qr(self):
        if self._last_qr_text:
            self.qr_text_pub.publish(String(data=self._last_qr_text))

    def target_event_cb(self, msg):
        label = msg.data.strip()
        if not label:
            return
        payload = json.dumps({
            "found": True, "count": 1,
            "frame_size": [640, 480],
            "detections": [{
                "class_id": 0, "class_name": label,
                "score": 0.95,
                "bbox": [100, 100, 300, 300],
                "center": [200, 200],
                "offset": [0, 0],
                "area": 40000,
            }]
        })
        self.detections_pub.publish(String(data=payload))
        self.get_logger().info(f"bridge target → {label}")


def main():
    rclpy.init()
    node = MissionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
