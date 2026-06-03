#!/usr/bin/env python3
"""
舵机投放控制
订阅 /pr_select，收到飞控命令后控制舵机到对应位置
用法: python3 servo_drop.py
停止: Ctrl+C
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

try:
    import Hobot.GPIO as GPIO
except ImportError:
    GPIO = None

SERVO_PIN  = 32    # 物理引脚 BOARD 编号
SERVO_FREQ = 50    # Hz（标准舵机）

# 占空比：脉宽(us) / 20000us * 100
DUTY = {
    "begin":  2.0,   # 400us  启动初始位置
    "first":  5.0,   # 1000us 0x0B
    "second": 7.0,   # 1400us 0x0C
    "third":  9.5,   # 1900us 0x0D
}

DROP_MAP = {
    0x0B: "first",
    0x0C: "second",
    0x0D: "third",
}


class ServoDropNode(Node):
    def __init__(self):
        super().__init__("servo_drop")
        self._pwm = None
        self._init_gpio()
        self.create_subscription(Int32, "/pr_select", self._on_cmd, 10)
        self.get_logger().info("舵机就绪，等待投放命令 0x0B/0x0C/0x0D ...")

    def _init_gpio(self):
        if GPIO is None:
            self.get_logger().error("Hobot.GPIO 未安装")
            return
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self._pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
        self._pwm.start(DUTY["begin"])
        self.get_logger().info(
            f"GPIO pin={SERVO_PIN}  初始位置 begin  duty={DUTY['begin']}%  (400us)"
        )

    def _on_cmd(self, msg: Int32):
        cmd = int(msg.data)
        if cmd not in DROP_MAP:
            return
        pos  = DROP_MAP[cmd]
        duty = DUTY[pos]
        if self._pwm is None:
            self.get_logger().error("PWM 未初始化，无法执行投放")
            return
        self._pwm.ChangeDutyCycle(duty)
        pulse_us = int(duty * 200)   # duty% × 20000us / 100
        self.get_logger().info(
            f"投放  cmd=0x{cmd:02X}  → {pos}  duty={duty}%  ({pulse_us}us)"
        )

    def destroy_node(self):
        if self._pwm is not None:
            self._pwm.stop()
        if GPIO is not None:
            GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = ServoDropNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
