#!/usr/bin/env python3
"""
圆环检测测试脚本
按键操作:
  0/1/2/3  - 切换摄像头
  q        - 退出
  s        - 保存当前帧
  +/-      - 调整检测灵敏度
"""

import cv2
import numpy as np
import sys

# ── 标定参数（测好后填这里）────────────────────
RING_REAL_DIAMETER_M = 1.2   # 圆环真实直径(米)
FOCAL_LENGTH_PX = 0          # 0=未标定，标定后填入

# ── 检测参数 ─────────────────────────────────
MIN_RADIUS_RATIO = 0.06      # 最小半径/画面短边
MAX_RADIUS_RATIO = 0.48      # 最大半径/画面短边
SENSITIVITY = 28             # HoughCircles param2，越小越灵敏


def detect_ring(frame, sensitivity):
    h, w = frame.shape[:2]
    min_dim = min(h, w)
    min_r = max(12, int(min_dim * MIN_RADIUS_RATIO))
    max_r = max(min_r + 1, int(min_dim * MAX_RADIUS_RATIO))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(30, min_dim // 3),
        param1=90,
        param2=sensitivity,
        minRadius=min_r,
        maxRadius=max_r,
    )

    if circles is None:
        return None

    circles = np.round(circles[0]).astype(int)
    # 选最大的圆
    best = max(circles, key=lambda c: c[2])
    cx, cy, r = best
    return cx, cy, r


def draw_info(frame, cx, cy, r, sensitivity):
    h, w = frame.shape[:2]
    center_x, center_y = w // 2, h // 2

    # 画圆
    cv2.circle(frame, (cx, cy), r, (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

    # 画画面中心
    cv2.drawMarker(frame, (center_x, center_y), (255, 255, 0),
                   cv2.MARKER_CROSS, 20, 2)

    # 画偏移线
    cv2.arrowedLine(frame, (center_x, center_y), (cx, cy),
                    (0, 165, 255), 2)

    # 偏差计算
    offset_x = cx - center_x
    offset_y = cy - center_y  # 正值=圆在画面下方=飞机需要升高

    # 距离估算
    dist_str = "未标定"
    if FOCAL_LENGTH_PX > 0:
        dist_m = (RING_REAL_DIAMETER_M * FOCAL_LENGTH_PX) / (2 * r)
        dist_str = f"{dist_m:.2f}m"

    # 文字信息
    texts = [
        f"圆心偏移: X={offset_x:+d}px  Y={offset_y:+d}px",
        f"圆半径: {r}px",
        f"距离: {dist_str}",
        f"灵敏度(param2): {sensitivity}  (+/-调整)",
    ]
    for i, t in enumerate(texts):
        cv2.putText(frame, t, (10, 30 + i * 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # 对齐提示
    aligned = abs(offset_x) < 20 and abs(offset_y) < 20
    status = ">>> 对齐! <<<" if aligned else f"X:{'左移' if offset_x>0 else '右移'} Y:{'降高' if offset_y>0 else '升高'}"
    color = (0, 255, 0) if aligned else (0, 100, 255)
    cv2.putText(frame, status, (w // 2 - 100, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    return offset_x, offset_y


def open_camera(idx):
    cap = cv2.VideoCapture(idx)
    if cap.isOpened():
        print(f"摄像头 {idx}: {int(cap.get(3))}x{int(cap.get(4))}")
        return cap
    print(f"摄像头 {idx} 打不开")
    return None


def main():
    # ── 启动前选择摄像头 ──────────────────────────
    print("=" * 40)
    print("可用摄像头:")
    available = []
    for i in range(4):
        cap_test = cv2.VideoCapture(i)
        if cap_test.isOpened():
            w = int(cap_test.get(3))
            h = int(cap_test.get(4))
            print(f"  [{i}] camera {i}  {w}x{h}")
            available.append(i)
            cap_test.release()
    print("=" * 40)

    cam_idx = None
    while cam_idx not in available:
        try:
            cam_idx = int(input(f"选择摄像头编号 {available}: "))
        except ValueError:
            pass

    cap = open_camera(cam_idx)
    sensitivity = SENSITIVITY
    save_count = 0

    print("操作: 0/1/2/3 切换摄像头 | +/- 调灵敏度 | s 保存 | q 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("读取帧失败")
            break

        display = frame.copy()
        result = detect_ring(frame, sensitivity)

        if result:
            cx, cy, r = result
            draw_info(display, cx, cy, r, sensitivity)
        else:
            h, w = display.shape[:2]
            cv2.putText(display, "未检测到圆环", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            cv2.putText(display, f"灵敏度: {sensitivity}  (+/-调整)",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 0), 2)

        cv2.putText(display, f"CAM:{cam_idx}", (display.shape[1]-80, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("Ring Detector", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key in [ord('0'), ord('1'), ord('2'), ord('3')]:
            new_idx = key - ord('0')
            new_cap = open_camera(new_idx)
            if new_cap:
                cap.release()
                cap = new_cap
                cam_idx = new_idx
        elif key in [ord('+'), ord('=')]:
            sensitivity = max(5, sensitivity - 3)
            print(f"灵敏度: {sensitivity}")
        elif key in [ord('-'), ord('_')]:
            sensitivity = min(80, sensitivity + 3)
            print(f"灵敏度: {sensitivity}")
        elif key == ord('s'):
            fname = f"/tmp/ring_{save_count}.jpg"
            cv2.imwrite(fname, frame)
            save_count += 1
            print(f"保存: {fname}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
