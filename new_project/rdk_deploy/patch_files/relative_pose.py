import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, Header
import math
import time


class RelativePoseNode(Node):
    def __init__(self):
        super().__init__('relative_pose_node')

        # 参数
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.0)
        self.declare_parameter('print_freq', 1.0)
        self.declare_parameter('max_position_jump', 5.0)       # 允许的最大单次跳变(m)
        self.declare_parameter('max_consecutive_errors', 5)    # 连续异常多少次后重置原点
        self.declare_parameter('init_max_meters', 50.0)        # 首次设定原点允许的最大绝对值(m)
        self.declare_parameter('max_relative_meters', 5.0)     # 相对原点最大允许距离(m)
        self.declare_parameter('valid_after_sec', 2.0)         # 原点设定后稳定多久才认为定位有效
        self.declare_parameter('odom_timeout_sec', 1.0)        # 多久没收到 Odometry 判为失效
        self.declare_parameter('use_initial_heading_frame', True)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('swap_xy', False)
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_z = self.get_parameter('target_z').value
        print_freq = self.get_parameter('print_freq').value
        self.max_position_jump = self.get_parameter('max_position_jump').value
        self.max_consecutive_errors = self.get_parameter('max_consecutive_errors').value
        self.init_max_meters = self.get_parameter('init_max_meters').value
        self.max_relative_meters = self.get_parameter('max_relative_meters').value
        self.valid_after_sec = self.get_parameter('valid_after_sec').value
        self.odom_timeout_sec = self.get_parameter('odom_timeout_sec').value
        self.use_initial_heading_frame = self.get_parameter('use_initial_heading_frame').value
        self.yaw_offset = math.radians(self.get_parameter('yaw_offset_deg').value)
        self.swap_xy = self.get_parameter('swap_xy').value
        self.invert_x = self.get_parameter('invert_x').value
        self.invert_y = self.get_parameter('invert_y').value

        # 原点（启动时的位置）
        self.origin = None
        self.origin_yaw = 0.0
        self.origin_set = False

        # 当前相对位姿
        self.rel_x = 0.0
        self.rel_y = 0.0
        self.map_rel_x = 0.0
        self.map_rel_y = 0.0
        self.rel_z = 0.0
        self.yaw = 0.0

        # 异常检测状态
        self.last_pos = None          # [x, y, z]
        self.error_count = 0
        self._last_error_log_time = 0.0
        self.origin_set_time = None
        self.localization_valid = False
        self.last_odom_time = None

        # 订阅 Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10)

        # 发布相对位姿
        self.rel_pose_pub = self.create_publisher(PoseStamped, '/relative_pose', 10)

        # 发布到目标点的误差
        self.error_pub = self.create_publisher(Point, '/position_error', 10)

        # 发布定位有效状态，fc_bridge 用它决定是否允许发送坐标到飞控
        self.valid_pub = self.create_publisher(Bool, '/localization_valid', 10)

        # 定时打印
        self.timer = self.create_timer(1.0 / print_freq, self.timer_callback)

        self.get_logger().info('相对定位节点已启动')
        self.get_logger().info('等待 FAST_LIO /Odometry 数据以设定原点...')
        self.get_logger().info(f'当前目标点: target=({self.target_x}, {self.target_y}, {self.target_z})')
        self.get_logger().info(
            '坐标模式: 开机位置为原点，'
            f'{"开机机头方向为 +X" if self.use_initial_heading_frame else "FAST_LIO map 坐标原样输出"}; '
            f'swap_xy={self.swap_xy}, invert_x={self.invert_x}, invert_y={self.invert_y}, '
            f'yaw_offset={math.degrees(self.yaw_offset):.1f}deg'
        )

    def _yaw_from_quaternion(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _apply_axis_options(self, x, y):
        if self.swap_xy:
            x, y = y, x
        if self.invert_x:
            x = -x
        if self.invert_y:
            y = -y
        return x, y

    def _publish_valid(self, valid: bool):
        if self.localization_valid != valid:
            if valid:
                self.get_logger().info('定位状态恢复有效，允许飞控桥接发送坐标')
            else:
                self.get_logger().warn('定位状态无效，飞控桥接应停止发送真实坐标')
        self.localization_valid = valid
        msg = Bool()
        msg.data = valid
        self.valid_pub.publish(msg)

    def _log_error_throttle(self, msg: str):
        """带节流的错误日志（每秒最多一次）"""
        now = time.time()
        if now - self._last_error_log_time >= 1.0:
            self._last_error_log_time = now
            self.get_logger().warn(msg)

    def odom_callback(self, msg: Odometry):
        self.last_odom_time = time.time()
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        current_yaw = self._yaw_from_quaternion(q)

        # 1. 基础合法性检查 (NaN / Inf)
        if not all(math.isfinite(v) for v in [pos.x, pos.y, pos.z]):
            self._log_error_throttle(f'Odometry 包含非法值，跳过: x={pos.x}, y={pos.y}, z={pos.z}')
            self._handle_error()
            self._publish_valid(False)
            return

        # 2. 首次设定原点时，防止 FAST_LIO 初始化异常导致原点错误
        if not self.origin_set:
            if abs(pos.x) > self.init_max_meters or abs(pos.y) > self.init_max_meters:
                self._log_error_throttle(
                    f'首次 Odometry 值过大，拒绝设定原点: x={pos.x:.2f}, y={pos.y:.2f}'
                )
                self._publish_valid(False)
                return
            self.origin = [pos.x, pos.y, pos.z]
            self.origin_yaw = current_yaw + self.yaw_offset
            self.origin_set = True
            self.last_pos = [pos.x, pos.y, pos.z]
            self.error_count = 0
            self.origin_set_time = time.time()
            self._publish_valid(False)
            self.get_logger().info(
                f'原点已设定: ({self.origin[0]:.3f}, {self.origin[1]:.3f}), '
                f'开机yaw={math.degrees(self.origin_yaw):.1f}°'
            )
        else:
            # 3. 检查与上一帧的跳变（防止雷达断线/插拔后 FAST_LIO 发散）
            jump = math.hypot(pos.x - self.last_pos[0], pos.y - self.last_pos[1])
            if jump > self.max_position_jump:
                self.error_count += 1
                self._log_error_throttle(
                    f'Odometry 跳变过大: {jump:.2f}m (阈值 {self.max_position_jump}m), '
                    f'连续异常 {self.error_count}/{self.max_consecutive_errors}'
                )
                if self.error_count >= self.max_consecutive_errors:
                    self.get_logger().error(
                        '连续异常次数过多，判定为定位失效，重置原点等待恢复...'
                    )
                    self.origin_set = False
                    self.origin = None
                    self.origin_yaw = 0.0
                    self.last_pos = None
                    self.origin_set_time = None
                    self.error_count = 0
                    self._publish_valid(False)
                return
            else:
                # 正常帧，清零错误计数
                if self.error_count > 0:
                    self.get_logger().info('Odometry 恢复正常')
                self.error_count = 0
                self.last_pos = [pos.x, pos.y, pos.z]

        # 4. 计算相对位移。先在 FAST_LIO map 中做差，再按开机 yaw 转到开机机头坐标系。
        self.map_rel_x = pos.x - self.origin[0]
        self.map_rel_y = pos.y - self.origin[1]
        if self.use_initial_heading_frame:
            cos_yaw = math.cos(self.origin_yaw)
            sin_yaw = math.sin(self.origin_yaw)
            rel_x = self.map_rel_x * cos_yaw + self.map_rel_y * sin_yaw
            rel_y = -self.map_rel_x * sin_yaw + self.map_rel_y * cos_yaw
        else:
            rel_x = self.map_rel_x
            rel_y = self.map_rel_y
        self.rel_x, self.rel_y = self._apply_axis_options(rel_x, rel_y)
        rel_dist = math.hypot(self.rel_x, self.rel_y)
        if rel_dist > self.max_relative_meters:
            self._log_error_throttle(
                f'相对位移超出安全范围，跳过发布: {rel_dist:.2f}m '
                f'(max={self.max_relative_meters}m)'
            )
            self._handle_error()
            self._publish_valid(False)
            return

        self.yaw = self._normalize_angle(current_yaw - self.origin_yaw)

        stable_sec = time.time() - self.origin_set_time if self.origin_set_time else 0.0
        self._publish_valid(stable_sec >= self.valid_after_sec)

        # 发布相对位姿
        rel_pose = PoseStamped()
        rel_pose.header = msg.header
        rel_pose.header.frame_id = 'relative_origin'
        rel_pose.pose.position.x = self.rel_x
        rel_pose.pose.position.y = self.rel_y
        rel_pose.pose.position.z = 0.0
        rel_pose.pose.orientation = q
        self.rel_pose_pub.publish(rel_pose)

        # 发布到目标点的误差
        error = Point()
        error.x = self.target_x - self.rel_x
        error.y = self.target_y - self.rel_y
        error.z = 0.0
        self.error_pub.publish(error)

    def _handle_error(self):
        """记录连续错误，必要时重置原点"""
        self.error_count += 1
        if self.error_count >= self.max_consecutive_errors and self.origin_set:
            self.get_logger().error('连续收到非法 Odometry，重置原点等待恢复...')
            self.origin_set = False
            self.origin = None
            self.origin_yaw = 0.0
            self.last_pos = None
            self.origin_set_time = None
            self.error_count = 0
            self._publish_valid(False)

    def timer_callback(self):
        if not self.origin_set:
            self._publish_valid(False)
            self.get_logger().warn('尚未收到 /Odometry，请确认 FAST_LIO 已启动')
            return

        if self.last_odom_time is None or time.time() - self.last_odom_time > self.odom_timeout_sec:
            self._publish_valid(False)
            age = time.time() - self.last_odom_time if self.last_odom_time else -1.0
            self.get_logger().warn(
                f'/Odometry 超时 {age:.1f}s，定位状态无效，请检查 FAST_LIO 是否仍在发布'
            )
            return

        # 计算到目标点的平面距离
        dist = math.sqrt(
            (self.target_x - self.rel_x) ** 2 +
            (self.target_y - self.rel_y) ** 2
        )

        self.get_logger().info(
            f'相对位移: Δx={self.rel_x:+.3f}m, Δy={self.rel_y:+.3f}m | '
            f'map=({self.map_rel_x:+.3f},{self.map_rel_y:+.3f})m | '
            f'yaw={math.degrees(self.yaw):.1f}° | 到目标点距离={dist:.3f}m'
        )

        if dist < 0.05:
            self.get_logger().info('>>> 已到达目标点！<<<')


def main(args=None):
    rclpy.init(args=args)
    node = RelativePoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
