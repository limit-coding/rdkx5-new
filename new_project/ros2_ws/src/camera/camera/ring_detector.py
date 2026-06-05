#!/usr/bin/env python3
"""
圆环检测节点 —— 使用第二个摄像头检测白色圆环并发布偏移和距离

发布话题:
  /ring/detected   std_msgs/Bool        是否检测到圆环
  /ring/offset     geometry_msgs/Point  圆心像素偏移(x=左右, y=上下, z=半径px)
  /ring/distance   std_msgs/Float32     估算距离(米), 未标定时为 -1.0

参数:
  camera_index       摄像头编号 (默认 0)
  focal_length_px    焦距像素值 (默认 0=未标定)
  ring_diameter_m    圆环真实直径米 (默认 1.2)
  min_radius_ratio   最小半径/画面短边 (默认 0.06)
  max_radius_ratio   最大半径/画面短边 (默认 0.48)
  hough_param2       HoughCircles灵敏度，越小越灵敏 (默认 28)
  white_s_max        HSV饱和度上限，用于白色过滤 (默认 55)
  white_v_min        HSV亮度下限，用于白色过滤 (默认 160)
  white_fill_ratio   圆环区域中白色像素占比阈值 (默认 0.25)
  confirm_frames     连续N帧才确认检测到 (默认 2)
  publish_rate       发布频率 Hz (默认 20)
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point

try:
    from hobot_vio import libsrcampy as srcampy
    HAS_HOBOT = True
except ImportError:
    srcampy = None
    HAS_HOBOT = False


class RingDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("ring_detector")

        self.declare_parameter("enabled", False)
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("focal_length_px", 0.0)
        self.declare_parameter("ring_diameter_m", 1.2)
        self.declare_parameter("min_radius_ratio", 0.06)
        self.declare_parameter("max_radius_ratio", 0.48)
        self.declare_parameter("hough_param2", 28)
        self.declare_parameter("white_s_max", 55)
        self.declare_parameter("white_v_min", 160)
        self.declare_parameter("white_fill_ratio", 0.25)
        self.declare_parameter("confirm_frames", 2)
        self.declare_parameter("publish_rate", 20.0)

        if not bool(self.get_parameter("enabled").value):
            self.get_logger().info("ring_detector disabled (enabled=False), 节点空转")
            return

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.focal_length_px = float(self.get_parameter("focal_length_px").value)
        self.ring_diameter_m = float(self.get_parameter("ring_diameter_m").value)
        self.min_radius_ratio = float(self.get_parameter("min_radius_ratio").value)
        self.max_radius_ratio = float(self.get_parameter("max_radius_ratio").value)
        self.hough_param2 = int(self.get_parameter("hough_param2").value)
        self.white_s_max = int(self.get_parameter("white_s_max").value)
        self.white_v_min = int(self.get_parameter("white_v_min").value)
        self.white_fill_ratio = float(self.get_parameter("white_fill_ratio").value)
        self.confirm_frames = max(1, int(self.get_parameter("confirm_frames").value))
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.detected_pub = self.create_publisher(Bool, "/ring/detected", 10)
        self.offset_pub = self.create_publisher(Point, "/ring/offset", 10)
        self.distance_pub = self.create_publisher(Float32, "/ring/distance", 10)

        self._lock = threading.Lock()
        self._latest: tuple[int, int, int] | None = None  # cx, cy, r
        self._seen_frames = 0

        camera_index = int(self.get_parameter("camera_index").value)
        if not HAS_HOBOT:
            self.get_logger().error("hobot_vio 未安装，节点无法打开摄像头")
            return

        self.camera = srcampy.Camera()
        self.camera.open_cam(camera_index, -1, -1, self.width, self.height, self.height, self.width)
        self.get_logger().info(
            f"ring_detector ready: cam={camera_index} {self.width}x{self.height}"
            f"  focal={self.focal_length_px}px  ring_d={self.ring_diameter_m}m"
        )

        t = threading.Thread(target=self._capture_loop, daemon=True)
        t.start()

        self.create_timer(1.0 / publish_rate, self._publish)

    def _capture_loop(self) -> None:
        while rclpy.ok():
            nv12 = self.camera.get_img(2, self.width, self.height)
            if nv12 is None:
                time.sleep(0.02)
                continue
            data = np.frombuffer(nv12, dtype=np.uint8).reshape(self.height * 3 // 2, self.width)
            frame = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_NV12)
            result = self._detect(frame)
            with self._lock:
                if result is not None:
                    self._seen_frames += 1
                    if self._seen_frames >= self.confirm_frames:
                        self._latest = result
                else:
                    self._seen_frames = 0
                    self._latest = None

    def _detect(self, frame: np.ndarray) -> tuple[int, int, int] | None:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, self.white_v_min]),
            np.array([180, self.white_s_max, 255]),
        )
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        masked = cv2.bitwise_and(gray, gray, mask=white_mask)
        blurred = cv2.GaussianBlur(masked, (9, 9), 0)

        h, w = gray.shape
        min_dim = min(h, w)
        min_r = max(12, int(min_dim * self.min_radius_ratio))
        max_r = max(min_r + 1, int(min_dim * self.max_radius_ratio))

        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(30, min_dim // 3),
            param1=90,
            param2=self.hough_param2,
            minRadius=min_r,
            maxRadius=max_r,
        )
        if circles is None:
            return None

        best = None
        best_r = 0
        for cx, cy, r in np.round(circles[0]).astype(int):
            thickness = max(3, r // 6)
            ring_mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.circle(ring_mask, (cx, cy), r, 255, thickness)
            ring_px = int(np.count_nonzero(ring_mask))
            if ring_px == 0:
                continue
            white_px = int(np.count_nonzero(cv2.bitwise_and(white_mask, ring_mask)))
            if white_px / ring_px >= self.white_fill_ratio and r > best_r:
                best = (cx, cy, r)
                best_r = r
        return best

    def _publish(self) -> None:
        with self._lock:
            result = self._latest

        detected = result is not None
        self.detected_pub.publish(Bool(data=detected))

        if not detected:
            return

        cx, cy, r = result
        center_x = self.width // 2
        center_y = self.height // 2
        offset_x = float(cx - center_x)
        offset_y = float(cy - center_y)

        pt = Point()
        pt.x = offset_x   # 正值=圆在画面右边
        pt.y = offset_y   # 正值=圆在画面下边
        pt.z = float(r)   # 像素半径
        self.offset_pub.publish(pt)

        dist = -1.0
        if self.focal_length_px > 0 and r > 0:
            dist = (self.ring_diameter_m / 2.0 * self.focal_length_px) / r
        self.distance_pub.publish(Float32(data=dist))

        self.get_logger().info(
            f"ring  offset=({offset_x:+.0f}, {offset_y:+.0f})px  r={r}px  dist={'未标定' if dist < 0 else f'{dist:.2f}m'}",
            throttle_duration_sec=1.0,
        )

    def destroy_node(self) -> bool:
        if hasattr(self, "camera"):
            self.camera.close_cam()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RingDetectorNode()
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
