#!/usr/bin/env python3
"""
Monitor mission_vision output — no model loading, instant start.
Usage: python3 vision_monitor.py
Stop:  Ctrl+C
"""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node("vision_monitor")

    def on_qr(msg):
        try:
            d = json.loads(msg.data)
            if d.get("confirmed"):
                print(f"\nQR  targets=[{d['class1']}, {d['class2']}]  landing={d['landing_side']}")
        except Exception:
            print(f"QR: {msg.data}")

    def on_target(msg):
        print(f"TARGET: {msg.data}  -> LAND")

    def on_debug(msg):
        print(f"  {msg.data}")

    node.create_subscription(String, "/qr_task",      on_qr,    10)
    node.create_subscription(String, "/target_event", on_target, 10)
    node.create_subscription(String, "/vision/debug", on_debug,  1)

    print("vision_monitor ready  |  Ctrl+C to stop\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
