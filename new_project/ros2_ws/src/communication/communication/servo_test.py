"""
Servo wiring test — uses Hobot.GPIO to configure pinmux (pin 32 → PWM mode),
then manually forces enable=1 to work around the Hobot.GPIO start() bug.
"""

import time, signal, sys
import Hobot.GPIO as GPIO

PIN      = 32
FREQ     = 50
LOCKED   = 1.25  # 250 us  → one end
RELEASE  = 12.5  # 2500 us → other end
INTERVAL = 1.5

CHIP    = "/sys/class/pwm/pwmchip0"
CHANNEL = 0
PERIOD  = 20_000_000
PWM_PATH = f"{CHIP}/pwm{CHANNEL}"

def sysfs_write(path, val):
    with open(path, "w") as f:
        f.write(str(val))

# Step 1: use Hobot.GPIO to configure pinmux (pin 32 → PWM mode)
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
pwm = GPIO.PWM(PIN, FREQ)
pwm.start(LOCKED)  # Hobot.GPIO sets up pinmux here, even if enable stays buggy

# Step 2: force enable=1 via sysfs to work around the bug
time.sleep(0.1)
sysfs_write(f"{PWM_PATH}/enable", 1)
print(f"PWM started on pin {PIN}  (pinmux via Hobot.GPIO + enable forced via sysfs)")

def set_duty(duty_pct):
    ns = int(PERIOD * duty_pct / 100)
    sysfs_write(f"{PWM_PATH}/duty_cycle", ns)

def cleanup(*_):
    print("\nstopping")
    set_duty(LOCKED)
    time.sleep(0.3)
    sysfs_write(f"{PWM_PATH}/enable", 0)
    pwm.stop()
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)

print(f"Swinging every {INTERVAL}s  (Ctrl-C to stop)")
state = False
while True:
    duty = RELEASE if state else LOCKED
    label = "RELEASE" if state else "LOCKED "
    print(f"  → {label}  ({duty:.1f}%)", flush=True)
    set_duty(duty)
    state = not state
    time.sleep(INTERVAL)
