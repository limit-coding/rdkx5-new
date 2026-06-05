#!/usr/bin/env python3
"""
圆环激光雷达检测节点

订阅 /mid360/xy_points (PointCloud2)，用RANSAC圆拟合找环形结构。

发布:
  /ring/detected   std_msgs/Bool
  /ring/offset     geometry_msgs/Point  x=前方距离(m) y=左右偏移(m) z=上下偏移(m)
  /ring/distance   std_msgs/Float32     前方距离(m)

参数:
  pc_topic          点云话题 (默认 /mid360/xy_points)
  ring_diameter_m   圆环直径米 (默认 1.2)
  diameter_tol      直径容差米 (默认 0.3, 即 0.9~1.5m 都接受)
  dist_min          检测最近距离m (默认 0.5)
  dist_max          检测最远距离m (默认 8.0)
  height_min        点云高度下限m，相对雷达 (默认 -1.5)
  height_max        点云高度上限m，相对雷达 (默认  1.5)
  ransac_iter       RANSAC迭代次数 (默认 80)
  ransac_thr        RANSAC内点距离阈值m (默认 0.06)
  min_inliers       最少内点数确认 (默认 8)
  confirm_frames    连续N帧才发布 (默认 2)
  publish_rate      发布频率Hz (默认 10.0)
"""

import threading
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point


# ── 点云解析 ─────────────────────────────────────────────────────────────────

def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """把 PointCloud2 解成 (N,3) float32 数组，只取 x/y/z。"""
    fields = {f.name: f.offset for f in msg.fields}
    ox, oy, oz = fields.get('x', 0), fields.get('y', 4), fields.get('z', 8)

    step = msg.point_step
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

    x = raw[:, ox:ox+4].copy().view(np.float32).ravel()
    y = raw[:, oy:oy+4].copy().view(np.float32).ravel()
    z = raw[:, oz:oz+4].copy().view(np.float32).ravel()

    pts = np.column_stack([x, y, z])
    return pts[np.isfinite(pts).all(axis=1)]


# ── RANSAC 圆拟合 ─────────────────────────────────────────────────────────────

def _circle_from_3pts(p1, p2, p3):
    """过三点求圆，返回 (cy, cz, r) 或 None。"""
    ax, ay = p1;  bx, by = p2;  cx, cy = p3
    D = 2.0 * (ax*(by-cy) + bx*(cy-ay) + cx*(ay-by))
    if abs(D) < 1e-9:
        return None
    s1 = ax*ax + ay*ay
    s2 = bx*bx + by*by
    s3 = cx*cx + cy*cy
    ux = (s1*(by-cy) + s2*(cy-ay) + s3*(ay-by)) / D
    uy = (s1*(cx-bx) + s2*(ax-cx) + s3*(bx-ax)) / D
    r  = np.sqrt((ax-ux)**2 + (ay-uy)**2)
    return ux, uy, r


def _ransac_circle(pts2d, n_iter, threshold, r_min, r_max):
    """
    对 (N,2) 点集跑RANSAC圆拟合。
    返回 (cy, cz, r, inlier_count) 或 None。
    """
    n = len(pts2d)
    if n < 8:
        return None

    best = None
    best_k = 0
    rng = np.random.default_rng()

    for _ in range(n_iter):
        idx = rng.choice(n, 3, replace=False)
        res = _circle_from_3pts(pts2d[idx[0]], pts2d[idx[1]], pts2d[idx[2]])
        if res is None:
            continue
        cy, cz, r = res
        if not (r_min <= r <= r_max):
            continue

        d = np.abs(np.sqrt((pts2d[:, 0]-cy)**2 + (pts2d[:, 1]-cz)**2) - r)
        k = int((d < threshold).sum())
        if k > best_k:
            best_k = k
            best = (cy, cz, r, k)

    return best


# ── ROS2 节点 ─────────────────────────────────────────────────────────────────

class RingLidarDetector(Node):
    def __init__(self):
        super().__init__('ring_lidar_detector')

        self.declare_parameter('pc_topic',        '/mid360/xy_points')
        self.declare_parameter('ring_diameter_m',  1.2)
        self.declare_parameter('diameter_tol',     0.3)
        self.declare_parameter('dist_min',         0.5)
        self.declare_parameter('dist_max',         8.0)
        self.declare_parameter('height_min',      -1.5)
        self.declare_parameter('height_max',       1.5)
        self.declare_parameter('ransac_iter',      80)
        self.declare_parameter('ransac_thr',       0.06)
        self.declare_parameter('min_inliers',      8)
        self.declare_parameter('confirm_frames',   2)
        self.declare_parameter('publish_rate',     10.0)

        r_d   = float(self.get_parameter('ring_diameter_m').value)
        tol   = float(self.get_parameter('diameter_tol').value)
        self._r_min = (r_d - tol) / 2.0
        self._r_max = (r_d + tol) / 2.0
        self._dist_min    = float(self.get_parameter('dist_min').value)
        self._dist_max    = float(self.get_parameter('dist_max').value)
        self._h_min       = float(self.get_parameter('height_min').value)
        self._h_max       = float(self.get_parameter('height_max').value)
        self._n_iter      = int(self.get_parameter('ransac_iter').value)
        self._thr         = float(self.get_parameter('ransac_thr').value)
        self._min_inliers = int(self.get_parameter('min_inliers').value)
        self._confirm     = max(1, int(self.get_parameter('confirm_frames').value))

        pc_topic = str(self.get_parameter('pc_topic').value)
        self.create_subscription(PointCloud2, pc_topic, self._on_cloud, 5)

        self._det_pub  = self.create_publisher(Bool,    '/ring/detected',  10)
        self._off_pub  = self.create_publisher(Point,   '/ring/offset',    10)
        self._dist_pub = self.create_publisher(Float32, '/ring/distance',  10)

        self._lock   = threading.Lock()
        self._latest = None   # (dist_x, offset_y, offset_z, inliers)
        self._streak = 0      # 连续检测到的帧数

        pub_rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / pub_rate, self._publish)

        self.get_logger().info(
            f'ring_lidar_detector ready  topic={pc_topic}  '
            f'r=[{self._r_min:.2f},{self._r_max:.2f}]m  '
            f'dist=[{self._dist_min},{self._dist_max}]m'
        )

    # ── 点云回调 ──────────────────────────────────────────────────────────────

    def _on_cloud(self, msg: PointCloud2):
        pts = _pc2_to_xyz(msg)
        if len(pts) == 0:
            with self._lock:
                self._latest = None
                self._streak = 0
            return

        # 1. 高度过滤
        mask_z = (pts[:, 2] >= self._h_min) & (pts[:, 2] <= self._h_max)
        pts = pts[mask_z]

        # 2. 距离过滤（只看前方 x>0）
        dist_xy = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
        mask_d = (pts[:, 0] > 0) & (dist_xy >= self._dist_min) & (dist_xy <= self._dist_max)
        pts = pts[mask_d]

        if len(pts) < 8:
            with self._lock:
                self._latest = None
                self._streak = 0
            return

        # 3. 每5个点取1个，降低计算量
        pts = pts[::5]

        # 4. RANSAC: 在 YZ 平面（环正面）拟合圆
        pts2d = pts[:, 1:3]   # y, z
        result = _ransac_circle(pts2d, self._n_iter, self._thr,
                                self._r_min, self._r_max)

        with self._lock:
            if result is None or result[3] < self._min_inliers:
                self._streak = 0
                self._latest = None
            else:
                cy, cz, r, k = result
                # x方向：取内点的平均前向距离
                inlier_mask = np.abs(
                    np.sqrt((pts2d[:, 0]-cy)**2 + (pts2d[:, 1]-cz)**2) - r
                ) < self._thr
                dist_x = float(pts[inlier_mask, 0].mean()) if inlier_mask.any() else float(pts[:, 0].mean())

                self._streak += 1
                if self._streak >= self._confirm:
                    self._latest = (dist_x, float(cy), float(cz), k)

    # ── 发布 ──────────────────────────────────────────────────────────────────

    def _publish(self):
        with self._lock:
            data = self._latest

        detected = data is not None
        self._det_pub.publish(Bool(data=detected))

        if not detected:
            return

        dist_x, oy, oz, k = data
        pt = Point(x=dist_x, y=oy, z=oz)
        self._off_pub.publish(pt)
        self._dist_pub.publish(Float32(data=float(dist_x)))

        self.get_logger().info(
            f'ring  dist={dist_x:.2f}m  offset=({oy:+.2f},{oz:+.2f})m  inliers={k}',
            throttle_duration_sec=1.0,
        )


def main():
    rclpy.init()
    node = RingLidarDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
