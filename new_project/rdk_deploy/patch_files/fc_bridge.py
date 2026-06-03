#!/usr/bin/env python3
"""
飞控串口统一桥接节点
合并原 fc_bridge（发位置帧）和 uart（收飞控数据 + 发任务状态）的全部功能
单进程独占 /dev/ttyFC，彻底消除多进程串口冲突
"""

import math
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Header, Int32, Int32MultiArray, String


DROP_COMMANDS = {0x0B, 0x0C, 0x0D}
DROP_ACK_TYPE = 0x0B


class FcSerialBridge(Node):

    def __init__(self):
        super().__init__('fc_bridge')

        self.declare_parameter('serial_port', '/dev/ttyFC')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_freq', 20.0)
        self.declare_parameter('max_xy_meters', 10.0)
        self.declare_parameter('valid_timeout_sec', 0.5)

        self._port = self.get_parameter('serial_port').value
        self._baud = int(self.get_parameter('baudrate').value)
        send_freq = float(self.get_parameter('send_freq').value)
        self._max_xy = float(self.get_parameter('max_xy_meters').value)
        self._valid_timeout = float(self.get_parameter('valid_timeout_sec').value)

        # 串口对象，写操作用锁保护（读在独立线程，不与写竞争）
        self._ser = None
        self._write_lock = threading.Lock()
        self._reconnect_timer = None
        self._try_open()

        # ── 发布：飞控 → 电脑 ────────────────────
        self._height_pub   = self.create_publisher(Int32, '/height', 10)
        self._unlock_pub   = self.create_publisher(String, '/unlock', 10)
        self._imu_pub      = self.create_publisher(Imu, '/IMU', 10)
        self._prselect_pub = self.create_publisher(Int32, '/pr_select', 10)
        self._drop_pub     = self.create_publisher(Bool, '/drop_payload', 10)

        # ── 订阅：电脑 → 飞控 ────────────────────
        # 激光雷达位置（FAST-LIO 输出）
        self._pose = None
        self._loc_valid = False
        self._last_valid_t = 0.0
        self._last_warn_reason = ''
        self._last_warn_t = 0.0
        self.create_subscription(PoseStamped, '/relative_pose', self._on_pose, 10)
        self.create_subscription(Bool, '/localization_valid', self._on_valid, 10)

        # 任务状态
        self._task_state   = 0x01
        self._landing_state = 0x01
        self._task_lock    = threading.Lock()
        self.create_subscription(Int32MultiArray, '/task_status', self._on_task_status, 10)

        # 上锁
        self.create_subscription(String, '/lock', self._on_lock, 10)

        # IMU 初始 yaw
        self._init_yaw = None

        # ── 接收缓冲 ─────────────────────────────
        self._rx_buf = bytearray()
        self._rx_types = {1, 4, 6, 18}   # 飞控会发的 type 值（十进制）

        # ── 定时发送 ─────────────────────────────
        self.create_timer(1.0 / send_freq, self._send_position)   # 位置帧 20 Hz
        self.create_timer(0.1, self._send_task_status)            # 任务状态帧 10 Hz

        # ── 接收线程 ─────────────────────────────
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info(f'串口已打开: {self._port} @ {self._baud}bps')

    # ═══════════════════════════════════════════════
    # 串口管理
    # ═══════════════════════════════════════════════

    def _try_open(self):
        import serial
        if self._ser is not None and self._ser.is_open:
            return True
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.1)
            self._ser.dtr = True
            self._ser.rts = True
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            return True
        except serial.SerialException as e:
            self._ser = None
            self.get_logger().debug(f'串口打开失败: {e}')
            return False

    def _close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _start_reconnect(self):
        if self._reconnect_timer is not None:
            return
        self.get_logger().warn(f'串口断开，每2秒重连 {self._port}...')
        self._reconnect_timer = self.create_timer(2.0, self._reconnect_cb)

    def _reconnect_cb(self):
        if self._try_open():
            self.get_logger().info('串口重连成功')
            self._reconnect_timer.cancel()
            self._reconnect_timer = None

    def _write(self, frame: bytes) -> bool:
        with self._write_lock:
            if self._ser is None or not self._ser.is_open:
                return False
            try:
                self._ser.write(frame)
                return True
            except Exception as e:
                self.get_logger().warn(f'串口写入失败: {e}')
                self._close()
                return False

    # ═══════════════════════════════════════════════
    # 接收线程（飞控 → 电脑）
    # ═══════════════════════════════════════════════

    def _rx_loop(self):
        import serial
        while rclpy.ok():
            if self._ser is None or not self._ser.is_open:
                time.sleep(0.1)
                continue
            try:
                data = self._ser.read(self._ser.in_waiting or 1)
                if data:
                    self._rx_buf += data
                    self._parse()
            except serial.SerialException as e:
                self.get_logger().error(f'串口读取错误: {e}')
                self._close()
                self._start_reconnect()
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'接收异常: {e}')
                time.sleep(0.05)

    def _parse(self):
        while True:
            idx = self._find_header()
            if idx < 0:
                if len(self._rx_buf) > 100:
                    self._rx_buf = self._rx_buf[-3:]
                return
            if idx > 0:
                self._rx_buf = self._rx_buf[idx:]
                continue
            if len(self._rx_buf) < 5:
                return
            # 帧格式: AA FF [type] [data_len] [data...] [checksum]
            data_len = self._rx_buf[3]
            total = 4 + data_len + 1
            if len(self._rx_buf) < total:
                return
            pkt = self._rx_buf[:total]
            calc = sum(pkt[:-1]) & 0xFF
            if calc != pkt[-1]:
                self.get_logger().error(
                    f'校验失败: 计算={calc:02X} 收到={pkt[-1]:02X}'
                )
                self._rx_buf = self._rx_buf[total:]
                continue
            self._handle(pkt[:-1])
            self._rx_buf = self._rx_buf[total:]

    def _find_header(self) -> int:
        for i in range(len(self._rx_buf) - 2):
            if (self._rx_buf[i] == 0xAA
                    and self._rx_buf[i + 1] == 0xFF
                    and self._rx_buf[i + 2] in self._rx_types):
                return i
        return -1

    def _handle(self, pkt: bytes):
        typ = pkt[2]
        body = pkt[4:]  # byte3=data_len, byte4+ = data

        if typ == 4 and len(body) == 4:
            height = struct.unpack('>i', body)[0]
            msg = Int32(data=height)
            self._height_pub.publish(msg)

        elif typ == 6 and len(body) == 6:
            try:
                text = body.decode('ascii').strip('\x00')
                self._unlock_pub.publish(String(data=text))
                self.get_logger().info(f'解锁信号: {text}')
            except Exception as e:
                self.get_logger().error(f'解锁解析失败: {e}')

        elif typ == 18 and len(body) == 18:
            try:
                v = struct.unpack('<9h', body)
                msg = Imu()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'
                msg.linear_acceleration = Vector3(
                    x=v[0] * 0.001,
                    y=v[1] * 0.001,
                    z=v[2] * 0.001,
                )
                dps2rps = (180.0 / 2000.0) * 0.0174533
                msg.angular_velocity = Vector3(
                    x=v[3] * dps2rps,
                    y=v[4] * dps2rps,
                    z=v[5] * dps2rps,
                )
                pit, rol = v[6] / 100.0, v[7] / 100.0
                if self._init_yaw is None:
                    self._init_yaw = v[8] / 100.0
                yaw = v[8] / 100.0 - self._init_yaw
                msg.orientation = self._euler_to_quat(rol, pit, yaw)
                self._imu_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'IMU 解析失败: {e}')

        elif typ == 1 and len(body) >= 1:
            cmd = body[0]
            self._prselect_pub.publish(Int32(data=cmd))
            self.get_logger().info(f'飞控命令: 0x{cmd:02X} ({cmd})')
            if cmd in DROP_COMMANDS:
                self._handle_drop_command(cmd)

    # ═══════════════════════════════════════════════
    # 发送：位置帧 AA FF 01 04 [x_lo x_hi y_lo y_hi] [sum]
    # ═══════════════════════════════════════════════

    def _on_pose(self, msg: PoseStamped):
        self._pose = msg

    def _on_valid(self, msg: Bool):
        self._loc_valid = bool(msg.data)
        if self._loc_valid:
            self._last_valid_t = time.time()

    def _send_position(self):
        if self._ser is None or not self._ser.is_open:
            self._start_reconnect()
            return

        x = y = 0.0
        yaw_deg = 0.0
        warn = ''

        valid = self._loc_valid and (time.time() - self._last_valid_t <= self._valid_timeout)

        if self._pose is None:
            warn = '尚未收到 /relative_pose，发送 0cm 心跳帧'
        elif not valid:
            warn = '定位无效，发送 0cm 心跳帧'
        else:
            x = self._pose.pose.position.x
            y = self._pose.pose.position.y
            q = self._pose.pose.orientation
            yaw_deg = math.degrees(
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            )

        if not (math.isfinite(x) and math.isfinite(y)):
            warn = '非法坐标 NaN/Inf，发送 0cm 心跳帧'
            x = y = yaw_deg = 0.0

        if abs(x) > self._max_xy or abs(y) > self._max_xy:
            warn = f'坐标越界 x={x:.2f}m y={y:.2f}m，发送 0cm 心跳帧'
            x = y = yaw_deg = 0.0

        xc   = max(-32768, min(32767, int(x * 100)))
        yc   = max(-32768, min(32767, int(y * 100)))
        yawc = max(-32768, min(32767, round(yaw_deg)))

        # AA FF 01 06 [X int16 BE][Y int16 BE][YAW int16 BE °][sum]
        frame = bytearray([0xAA, 0xFF, 0x01, 0x06])
        frame += struct.pack('>hhh', xc, yc, yawc)
        frame.append(sum(frame) & 0xFF)

        if not self._write(bytes(frame)):
            self._start_reconnect()
            return

        now = time.time()
        if warn and (warn != self._last_warn_reason or now - self._last_warn_t >= 1.0):
            self._last_warn_reason = warn
            self._last_warn_t = now
            self.get_logger().warn(warn)

        if not hasattr(self, '_last_pos_log') or now - self._last_pos_log >= 1.0:
            self._last_pos_log = now
            self.get_logger().info(
                f'位置帧 -> X={xc}cm Y={yc}cm YAW={yaw_deg:.1f}° | '
                f'{" ".join(f"{b:02X}" for b in frame)}'
            )

    # ═══════════════════════════════════════════════
    # 发送：任务状态帧 AA FF 02 02 [task] [landing] [sum]
    # ═══════════════════════════════════════════════

    def _on_task_status(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            return
        with self._task_lock:
            self._task_state    = int(msg.data[0]) & 0xFF
            self._landing_state = int(msg.data[1]) & 0xFF

    def _send_task_status(self):
        with self._task_lock:
            ts = self._task_state
            ls = self._landing_state
        frame = bytes([0xAA, 0xFF, 0x02, 0x02, ts, ls])
        self._write(frame + bytes([sum(frame) & 0xFF]))

    # ═══════════════════════════════════════════════
    # 接收投放命令 0x0B 后立即 ACK，并触发本机投放
    # 飞控 → 电脑: AA FF 01 [0B/0C/0D] checksum
    # 电脑 → 飞控: AA FF 0B 01 [0B/0C/0D] checksum
    # ═══════════════════════════════════════════════

    def _handle_drop_command(self, cmd: int):
        ack = self._make_frame(DROP_ACK_TYPE, bytes([cmd & 0xFF]))
        if self._write(ack):
            self.get_logger().info(
                f'投放命令 ACK -> {" ".join(f"{b:02X}" for b in ack)}'
            )
        else:
            self._start_reconnect()
            self.get_logger().warn('投放命令 ACK 发送失败，等待串口重连')

        self._drop_pub.publish(Bool(data=True))
        self.get_logger().info('已发布 /drop_payload=True')

    # ═══════════════════════════════════════════════
    # 发送：上锁帧 AA FF 81 04 "lock" [sum]
    # ═══════════════════════════════════════════════

    def _on_lock(self, msg: String):
        if msg.data.strip().lower() != 'lock':
            return
        data = b'lock'
        self._write(self._make_frame(0x81, data))
        self.get_logger().info('发送上锁帧')

    # ═══════════════════════════════════════════════
    # 工具
    # ═══════════════════════════════════════════════

    def _make_frame(self, frame_type: int, payload: bytes = b'') -> bytes:
        base = bytes([0xAA, 0xFF, frame_type & 0xFF, len(payload) & 0xFF]) + payload
        return base + bytes([sum(base) & 0xFF])

    def _euler_to_quat(self, roll_deg, pitch_deg, yaw_deg) -> Quaternion:
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)
        cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
        cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
        cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
        return Quaternion(
            w=cr * cp * cy + sr * sp * sy,
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
        )

    def destroy_node(self):
        self._close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FcSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
