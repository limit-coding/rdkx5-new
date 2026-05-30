import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import serial
import struct
import time
import math


class FcBridgeNode(Node):
    """
    凌霄飞控串口桥接节点
    订阅 /relative_pose，按协议通过串口发送 XY 坐标到飞控

    通信协议:
    [0xAA] [0xFF] [0x01] [LEN] [X低] [X高] [Y低] [Y高] [CHECKSUM]
    - LEN = 4 (X占2字节 + Y占2字节)
    - X, Y: int16, 小端模式, 单位: cm
    - CHECKSUM: 从 LEN 开始到 Y高 的累加和, 取低8位
    """

    def __init__(self):
        super().__init__('fc_bridge')

        # 参数
        self.declare_parameter('serial_port', '/dev/ttyFC')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_freq', 20.0)  # 发送频率 Hz
        self.declare_parameter('max_xy_meters', 2.0)  # 单次发送最大允许绝对值(m)
        self.declare_parameter('valid_timeout_sec', 0.5)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        send_freq = self.get_parameter('send_freq').value
        self.max_xy_meters = self.get_parameter('max_xy_meters').value
        self.valid_timeout_sec = self.get_parameter('valid_timeout_sec').value

        # 串口对象
        self.ser = None
        self._try_open_serial()

        # 重连定时器（仅在断开时启用）
        self.reconnect_timer = None

        # 订阅相对位姿
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/relative_pose',
            self.pose_callback,
            10)

        self.valid_sub = self.create_subscription(
            Bool,
            '/localization_valid',
            self.valid_callback,
            10)

        self.current_pose = None
        self.localization_valid = False
        self.last_valid_time = 0.0
        self._last_invalid_log_time = 0.0
        self._last_zero_reason = ''

        # 定时发送
        self.timer = self.create_timer(1.0 / send_freq, self.send_callback)

        self.get_logger().info('飞控桥接节点已启动，等待 /relative_pose 数据...')

    def _try_open_serial(self):
        """尝试打开串口，成功返回 True"""
        if self.ser is not None and self.ser.is_open:
            return True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            self.ser.dtr = True
            self.ser.rts = True
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f'串口已打开: {self.port} @ {self.baud}bps')
            return True
        except serial.SerialException as e:
            if self.ser is not None:
                self.ser = None
            # 仅在首次或重连时打印一次 error，避免刷屏
            self.get_logger().debug(f'串口打开失败 {self.port}: {e}')
            return False

    def _close_serial(self):
        """安全关闭串口"""
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            finally:
                self.ser = None

    def _start_reconnect_timer(self):
        """启动重连定时器（如果未启动）"""
        if self.reconnect_timer is not None:
            return
        self.get_logger().warn(f'串口断开，每 2 秒尝试重连 {self.port}...')
        self.reconnect_timer = self.create_timer(2.0, self._reconnect_callback)

    def _stop_reconnect_timer(self):
        """停止重连定时器"""
        if self.reconnect_timer is not None:
            self.reconnect_timer.cancel()
            self.reconnect_timer = None

    def _reconnect_callback(self):
        """定时重连回调"""
        if self._try_open_serial():
            self.get_logger().info('串口重连成功！')
            self._stop_reconnect_timer()

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def valid_callback(self, msg: Bool):
        self.localization_valid = bool(msg.data)
        if self.localization_valid:
            self.last_valid_time = time.time()

    def send_callback(self):
        if self.ser is None or not self.ser.is_open:
            self._start_reconnect_timer()
            return

        x = 0.0
        y = 0.0
        zero_reason = ''

        valid_recent = (
            self.localization_valid and
            time.time() - self.last_valid_time <= self.valid_timeout_sec
        )

        if self.current_pose is None:
            zero_reason = '尚未收到 /relative_pose，发送 0cm 心跳帧'
        elif not valid_recent:
            zero_reason = '定位状态无效，发送 0cm 心跳帧'
        else:
            x = self.current_pose.pose.position.x
            y = self.current_pose.pose.position.y

        # 异常值过滤：防止 FAST_LIO 发散或 relative_pose 异常时把错误数据发给飞控
        if not (math.isfinite(x) and math.isfinite(y)):
            zero_reason = f'收到非法坐标 (NaN/Inf)，发送 0cm 心跳帧: x={x}, y={y}'
            x = 0.0
            y = 0.0

        if abs(x) > self.max_xy_meters or abs(y) > self.max_xy_meters:
            zero_reason = (
                f'坐标超出安全范围，发送 0cm 心跳帧: x={x:.2f}m, y={y:.2f}m '
                f'(max={self.max_xy_meters}m)'
            )
            x = 0.0
            y = 0.0

        # 提取 XY，米 -> 厘米，转 int16
        x_cm = int(x * 100.0)
        y_cm = int(y * 100.0)

        # 限制在 int16 范围 (-32768 ~ 32767)
        x_cm = max(-32768, min(32767, x_cm))
        y_cm = max(-32768, min(32767, y_cm))

        # 打包为小端 int16
        x_bytes = struct.pack('<h', x_cm)
        y_bytes = struct.pack('<h', y_cm)

        # 构造数据帧
        frame = bytearray()
        frame.append(0xAA)
        frame.append(0xFF)
        frame.append(0x01)
        frame.append(0x04)
        frame.extend(x_bytes)
        frame.extend(y_bytes)

        # 计算校验和：CHECKSUM 之前所有字节累加和，取低8位
        checksum = sum(frame) & 0xFF
        frame.append(checksum)

        # 发送
        try:
            self.ser.write(bytes(frame))
        except serial.SerialException as e:
            self.get_logger().warn(f'串口写入失败: {e}')
            self._close_serial()
            self._start_reconnect_timer()
            return

        # 调试打印（每1秒打印一次）
        now = time.time()
        if zero_reason and (
            zero_reason != self._last_zero_reason or
            now - self._last_invalid_log_time >= 1.0
        ):
            self._last_invalid_log_time = now
            self._last_zero_reason = zero_reason
            self.get_logger().warn(zero_reason)

        if not hasattr(self, '_last_log_time') or now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.get_logger().info(
                f'串口发送 -> X={x_cm}cm, Y={y_cm}cm | '
                f'hex: {" ".join(f"{b:02X}" for b in frame)}'
            )

    def destroy_node(self):
        self._stop_reconnect_timer()
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FcBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
