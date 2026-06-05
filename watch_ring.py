#!/usr/bin/env python3
"""
圆环检测实时监测 —— 后台输出
用法:  python3 watch_ring.py
停止:  Ctrl+C

检测白色圆环，每秒打印：
  - 是否检测到
  - 距离提示（太近/合适/太远）
  - 圆心偏移（X左右 Y上下）
  - 当前距离估算（需先标定焦距）
"""

import cv2
import numpy as np
import time
import sys

# ── 参数（改这里）──────────────────────────────
CAMERA_INDEX       = 0
WIDTH, HEIGHT      = 1920, 1080
FOCAL_LENGTH_PX    = 0       # 0 = 未标定；标定后填入
RING_DIAMETER_M    = 1.2     # 圆环真实直径（米）

MIN_RADIUS_RATIO   = 0.06    # 最小半径/画面短边
MAX_RADIUS_RATIO   = 0.48    # 最大半径/画面短边
HOUGH_PARAM2       = 28      # 越小越灵敏

WHITE_S_MAX        = 55      # HSV 饱和度上限（白色过滤）
WHITE_V_MIN        = 160     # HSV 亮度下限（白色过滤）
WHITE_FILL_RATIO   = 0.25    # 圆环区域白色像素占比阈值

PRINT_INTERVAL     = 0.8     # 打印间隔（秒）
# ────────────────────────────────────────────


def detect_white_ring(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv,
        np.array([0, 0, WHITE_V_MIN]),
        np.array([180, WHITE_S_MAX, 255]))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    masked = cv2.bitwise_and(gray, gray, mask=white_mask)
    blurred = cv2.GaussianBlur(masked, (9, 9), 0)

    h, w = gray.shape
    min_dim = min(h, w)
    min_r = max(12, int(min_dim * MIN_RADIUS_RATIO))
    max_r = max(min_r + 1, int(min_dim * MAX_RADIUS_RATIO))

    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2,
        minDist=max(30, min_dim // 3),
        param1=90, param2=HOUGH_PARAM2,
        minRadius=min_r, maxRadius=max_r,
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
        if white_px / ring_px >= WHITE_FILL_RATIO and r > best_r:
            best = (cx, cy, r)
            best_r = r
    return best


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"摄像头 {CAMERA_INDEX} 打不开", flush=True)
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"摄像头已开: {w}x{h}  按 Ctrl+C 停止", flush=True)
    print("-" * 50, flush=True)

    last_print = 0
    seen = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.02)
                continue

            result = detect_white_ring(frame)
            now = time.time()
            if now - last_print < PRINT_INTERVAL:
                continue
            last_print = now

            ts = time.strftime("%H:%M:%S")

            if result is None:
                seen = 0
                print(f"[{ts}] 未检测到白色圆环", flush=True)
            else:
                seen += 1
                cx, cy, r = result
                ox = cx - w // 2
                oy = cy - h // 2
                min_dim = min(h, w)
                ratio = r / min_dim

                if ratio < 0.10:
                    dist_hint = "太远 → 往前"
                elif ratio > 0.35:
                    dist_hint = "太近 → 往后退"
                else:
                    dist_hint = "距离合适 ✓"

                if FOCAL_LENGTH_PX > 0:
                    dist_m = (RING_DIAMETER_M / 2.0 * FOCAL_LENGTH_PX) / r
                    dist_str = f"{dist_m:.2f}m"
                else:
                    dist_str = "未标定"

                align = "对准 ✓" if abs(ox) < 40 and abs(oy) < 40 else \
                        f"偏移 X={ox:+d}px {'←左' if ox>0 else '右→'}  Y={oy:+d}px {'↑升' if oy>0 else '降↓'}"

                print(f"[{ts}] 圆环 r={r}px({ratio*100:.0f}%)  {dist_hint}  {align}  距离={dist_str}", flush=True)

    except KeyboardInterrupt:
        print("\n停止", flush=True)
    finally:
        cap.release()


if __name__ == "__main__":
    main()
