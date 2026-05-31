"""
Servo sweep calibration — interactive, supports forward and backward stepping.

Run on board:
    python3 servo_sweep.py

Controls:
    Enter  → step forward  (+100 us)
    b      → step backward (-100 us)
    r      → mark current position as key point
    q      → quit and print summary
"""

import time, signal, sys

CHIP     = "/sys/class/pwm/pwmchip0"
CHANNEL  = 0
PERIOD   = 20_000_000
STEP_US  = 100
MIN_US   = 500
MAX_US   = 2500

PWM = f"{CHIP}/pwm{CHANNEL}"

def w(path, val):
    with open(path, "w") as f:
        f.write(str(val))

try:
    w(f"{CHIP}/export", CHANNEL)
    time.sleep(0.1)
except OSError:
    pass

w(f"{PWM}/period",     PERIOD)
w(f"{PWM}/duty_cycle", MIN_US * 1000)
w(f"{PWM}/enable",     1)

def cleanup(*_):
    w(f"{PWM}/enable", 0)
    w(f"{CHIP}/unexport", CHANNEL)
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)

print("=" * 52)
print("Servo sweep  |  Enter=前进  b=后退  r=标记  q=结束")
print("=" * 52)

marked = {}
current = MIN_US

while True:
    pct = current / (PERIOD // 1000) * 100
    key = input(f"  {current:4d} us  ({pct:.1f}%)  > ").strip().lower()

    if key == "q":
        break
    elif key == "b":
        current = max(MIN_US, current - STEP_US)
        w(f"{PWM}/duty_cycle", current * 1000)
    elif key == "r":
        label = input(f"    标注 {current} us 为: ").strip()
        marked[current] = label
        print(f"    ✓ {current} us → {label}")
    else:
        current = min(MAX_US, current + STEP_US)
        w(f"{PWM}/duty_cycle", current * 1000)

print("\n" + "=" * 52)
print("汇总:")
for us, label in sorted(marked.items()):
    pct = us / (PERIOD // 1000) * 100
    print(f"  {label:10s} → {us} us  ({pct:.1f}%)")
print("=" * 52)

w(f"{PWM}/enable", 0)
w(f"{CHIP}/unexport", CHANNEL)
