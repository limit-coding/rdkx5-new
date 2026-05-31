"""
Servo dropper node — controls a PWM servo to release/lock a payload.

Topic interface:
  Subscribe: /drop_payload  (std_msgs/Bool)
      True  → execute drop sequence (open → hold → close)
      False → force servo to locked position

  Publish:   /drop_status  (std_msgs/String)
      "locked" | "dropping" | "done"

PWM backend priority:
  1. Hobot.GPIO  (RDK native, preferred)
  2. sysfs PWM   (/sys/class/pwm/pwmchipN/)
"""

import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# ── PWM parameters ────────────────────────────────────────────────────────────
SERVO_FREQ_HZ   = 50        # standard servo frequency

DUTY_BEGIN      = 2.0       #  400 us → home / reset position
DUTY_FIRST      = 5.0       # 1000 us
DUTY_SECOND     = 7.0       # 1400 us
DUTY_THIRD      = 9.5       # 1900 us

DUTY_LOCKED     = DUTY_BEGIN  # reset/init always returns to begin
DUTY_RELEASE    = DUTY_THIRD  # open → third position
HOLD_OPEN_SEC   = 1.5         # time to hold open before returning

# ── GPIO / sysfs config ───────────────────────────────────────────────────────
BOARD_PIN       = 32        # physical pin on 40-pin header (Hobot.GPIO BOARD mode)
# sysfs fallback — adjust pwmchipN and channel index for your kernel
SYSFS_CHIP      = "pwmchip0"   # Board Pin 32 = pwmchip0/pwm0 (confirmed on RDK X5)
SYSFS_CHANNEL   = 0
PERIOD_NS       = 20_000_000            # 20 ms → 50 Hz
DUTY_LOCKED_NS  = int(PERIOD_NS * DUTY_LOCKED  / 100)   #  400 000 ns
DUTY_RELEASE_NS = int(PERIOD_NS * DUTY_RELEASE / 100)   # 1 900 000 ns


class _HobotPWM:
    """Thin wrapper around Hobot.GPIO PWM."""

    def __init__(self, pin: int, freq: float):
        import Hobot.GPIO as GPIO
        self._GPIO = GPIO
        GPIO.setmode(GPIO.BOARD)
        self._pwm = GPIO.PWM(pin, freq)
        self._pwm.start(DUTY_LOCKED)

    def set_duty(self, duty: float):
        self._pwm.ChangeDutyCycle(duty)

    def close(self):
        self._pwm.stop()
        self._GPIO.cleanup()


class _SysfsPWM:
    """sysfs PWM backend — works without Hobot.GPIO."""

    def __init__(self, chip: str, channel: int):
        import os
        self._base = f"/sys/class/pwm/{chip}/pwm{channel}"
        export_path = f"/sys/class/pwm/{chip}/export"

        if not os.path.exists(self._base):
            with open(export_path, "w") as f:
                f.write(str(channel))
            time.sleep(0.1)

        self._write("period", PERIOD_NS)
        self._write("duty_cycle", DUTY_LOCKED_NS)
        self._write("enable", 1)

    def _write(self, attr: str, value):
        with open(f"{self._base}/{attr}", "w") as f:
            f.write(str(value))

    def set_duty(self, duty: float):
        ns = int(PERIOD_NS * duty / 100)
        # duty_cycle must not exceed period
        ns = max(0, min(ns, PERIOD_NS))
        self._write("duty_cycle", ns)

    def close(self):
        self._write("enable", 0)


def _make_pwm_backend():
    try:
        backend = _HobotPWM(BOARD_PIN, SERVO_FREQ_HZ)
        return backend, "Hobot.GPIO"
    except Exception:
        pass
    try:
        backend = _SysfsPWM(SYSFS_CHIP, SYSFS_CHANNEL)
        return backend, "sysfs"
    except Exception as e:
        raise RuntimeError(f"No PWM backend available: {e}")


class ServoDropperNode(Node):

    def __init__(self):
        super().__init__("servo_dropper")

        self._lock = threading.Lock()
        self._dropping = False

        try:
            self._pwm, backend = _make_pwm_backend()
            self.get_logger().info(f"PWM backend: {backend}")
        except RuntimeError as e:
            self.get_logger().error(str(e))
            self._pwm = None

        self._status_pub = self.create_publisher(String, "/drop_status", 10)
        self.create_subscription(Bool, "/drop_payload", self._drop_callback, 10)

        self._publish_status("locked")
        self.get_logger().info("servo_dropper ready — waiting on /drop_payload")

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _drop_callback(self, msg: Bool):
        if msg.data:
            self._trigger_drop()
        else:
            self._force_lock()

    # ── Servo actions ─────────────────────────────────────────────────────────

    def _trigger_drop(self):
        with self._lock:
            if self._dropping:
                self.get_logger().warn("drop already in progress, ignoring")
                return
            self._dropping = True

        t = threading.Thread(target=self._drop_sequence, daemon=True)
        t.start()

    def _drop_sequence(self):
        self.get_logger().info("drop sequence start")
        self._publish_status("dropping")
        self._set_servo(DUTY_RELEASE)
        time.sleep(HOLD_OPEN_SEC)
        self._set_servo(DUTY_LOCKED)
        self._publish_status("done")
        self.get_logger().info("drop sequence complete")
        with self._lock:
            self._dropping = False

    def _force_lock(self):
        self.get_logger().info("force lock")
        self._set_servo(DUTY_LOCKED)
        self._publish_status("locked")

    def _set_servo(self, duty: float):
        if self._pwm is None:
            self.get_logger().warn(f"PWM not available, would set duty={duty:.1f}%")
            return
        self._pwm.set_duty(duty)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self._pwm is not None:
            self._set_servo(DUTY_LOCKED)
            self._pwm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDropperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
