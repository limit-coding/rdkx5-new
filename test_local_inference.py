"""
Local inference test on a target image.
Usage: python3 test_local_inference.py <image_path>
"""
import sys
import cv2
import numpy as np
from pathlib import Path

MODEL_PT   = "new_project/ai_training/models_A/best.pt"
NAMES_FILE = "new_project/ros2_ws/src/camera/resource/cifar100_names.txt"


def order_points(points):
    pts = points.reshape(4, 2).astype(np.float32)
    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)
    return np.array([
        pts[np.argmin(sums)], pts[np.argmin(diffs)],
        pts[np.argmax(sums)], pts[np.argmax(diffs)],
    ], dtype=np.float32)


def find_quad_from_dark(gray, min_area):
    h, w = gray.shape
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    best_approx, best_area = None, 0.0
    for thresh_val in (70, 90, 110):
        _, dark = cv2.threshold(blurred, thresh_val, 255, cv2.THRESH_BINARY_INV)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, k)
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, k)
        cnts, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area or area > h * w * 0.85 or area <= best_area:
                continue
            hull = cv2.convexHull(c)
            peri = cv2.arcLength(hull, True)
            for eps in (0.03, 0.05, 0.08, 0.12):
                approx = cv2.approxPolyDP(hull, eps * peri, True)
                if len(approx) == 4:
                    best_approx, best_area = approx, area
                    break
    return best_approx


def find_quad_from_canny(gray, min_area):
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    best_approx, best_area = None, 0.0
    for lo, hi in [(20, 80), (40, 120), (60, 160)]:
        edges = cv2.Canny(blurred, lo, hi)
        cnts, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area or area <= best_area:
                continue
            hull = cv2.convexHull(c)
            peri = cv2.arcLength(hull, True)
            for eps in (0.02, 0.03, 0.05, 0.08, 0.12):
                approx = cv2.approxPolyDP(hull, eps * peri, True)
                if len(approx) == 4:
                    best_approx, best_area = approx, area
                    break
    return best_approx


def _find_white_circle(gray, min_area):
    """
    Find the white circle. Only returns a result when the circle is FULLY
    within the frame (complete contour, not touching any edge).
    Returns (cx, cy, r) or None.
    """
    h, w = gray.shape
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    best, best_r = None, 0.0
    for t in range(200, 120, -10):
        _, mask = cv2.threshold(blurred, t, 255, cv2.THRESH_BINARY)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            (cx, cy), r = cv2.minEnclosingCircle(c)
            circ = area / (np.pi * r * r) if r > 0 else 0
            if circ < 0.45 or r <= best_r:
                continue
            # ── Completeness check: circle must not touch any frame edge ──
            margin = int(r * 0.05)
            if (cx - r < margin or cx + r > w - margin or
                    cy - r < margin or cy + r > h - margin):
                continue
            best, best_r = (int(cx), int(cy), int(r)), r
        if best:
            break
    return best


def _find_content_in_circle(white_crop, crop_cx, crop_cy, r_white):
    """
    Within the white circle area, find the non-white content (card / fish sticker).
    Uses the EXACT circle center (crop_cx, crop_cy) in crop coordinates so the
    circle mask is correctly placed even when the fish is off-center.

    Strategy A: Canny rectangle detection (dark-bordered card).
    Strategy B: non-white content bounding box (light card or fish sticker).
    Returns (cropped_content, method_name) or (None, None).
    """
    h, w = white_crop.shape[:2]
    gray = cv2.cvtColor(white_crop, cv2.COLOR_BGR2GRAY)
    min_card_area = (r_white * 0.20) ** 2
    max_card_area = (r_white * 1.6) ** 2

    # ── Strategy A: Canny rectangle ────────────────────────────────────────
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    best_approx, best_area = None, 0.0
    for lo, hi in [(20, 80), (30, 100), (50, 150)]:
        edges = cv2.Canny(blurred, lo, hi)
        cnts, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_card_area or area > max_card_area or area <= best_area:
                continue
            hull = cv2.convexHull(c)
            peri = cv2.arcLength(hull, True)
            for eps in (0.02, 0.04, 0.06, 0.10):
                approx = cv2.approxPolyDP(hull, eps * peri, True)
                if len(approx) == 4:
                    best_approx, best_area = approx, area
                    break
    if best_approx is not None:
        src = order_points(best_approx)
        side = 224
        dst = np.array([[0,0],[side-1,0],[side-1,side-1],[0,side-1]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(white_crop, M, (side, side)), "card_rect"

    # ── Strategy B: non-white content bounding box ─────────────────────────
    # Use EXACT circle center for the mask (critical for off-center content)
    yy, xx = np.ogrid[:h, :w]
    circ_mask = ((xx - crop_cx)**2 + (yy - crop_cy)**2
                 <= (r_white * 1.0)**2).astype(np.uint8) * 255

    hsv = cv2.cvtColor(white_crop, cv2.COLOR_BGR2HSV)
    white_bg = cv2.inRange(hsv, (0, 0, 185), (180, 30, 255))
    content = cv2.bitwise_and(cv2.bitwise_not(white_bg), circ_mask)

    ksize = max(5, r_white // 15)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    content = cv2.morphologyEx(content, cv2.MORPH_CLOSE, k)
    content = cv2.morphologyEx(content, cv2.MORPH_OPEN, k)

    cnts, _ = cv2.findContours(content, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cnts:
        largest = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(largest) > min_card_area * 0.2:
            # Perspective warp: only when content fills ≥35% of circle area
            # (colored-background cards like motorcycle).
            # White-background cards (fish) have fill ~21% → skip to bbox.
            circle_area = np.pi * r_white * r_white
            content_area = cv2.contourArea(largest)
            if content_area / circle_area >= 0.35:
                hull = cv2.convexHull(largest)
                peri = cv2.arcLength(hull, True)
                for eps in (0.02, 0.04, 0.06, 0.10):
                    approx = cv2.approxPolyDP(hull, eps * peri, True)
                    if len(approx) != 4:
                        continue
                    pts = order_points(approx)
                    # Reject degenerate quads: min side must be > 20% of circle radius
                    sides = [np.linalg.norm(pts[(i+1)%4] - pts[i]) for i in range(4)]
                    if min(sides) < r_white * 0.20:
                        continue
                    side = 224
                    dst = np.array([[0,0],[side-1,0],[side-1,side-1],[0,side-1]], dtype=np.float32)
                    M = cv2.getPerspectiveTransform(pts, dst)
                    return cv2.warpPerspective(white_crop, M, (side, side)), "content_warp"
            # Fallback to axis-aligned bounding box
            bx, by, bw, bh = cv2.boundingRect(largest)
            pad = int(max(bw, bh) * 0.15)
            s = max(bw, bh) + 2 * pad
            cx2, cy2 = bx + bw // 2, by + bh // 2
            x1 = max(0, cx2 - s // 2)
            y1 = max(0, cy2 - s // 2)
            return white_crop[y1:min(h, y1+s), x1:min(w, x1+s)], "content_bbox"

    return None, None


def crop_target(frame):
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ── Step 1: find white circle — ONLY if the full contour is visible ────
    wc = _find_white_circle(gray, h * w * 0.01)
    if wc is not None:
        cx, cy, r = wc
        # Crop to exact white circle interior
        x1, y1 = max(0, cx - r), max(0, cy - r)
        x2, y2 = min(w, cx + r), min(h, cy + r)
        white_crop = frame[y1:y2, x1:x2]

        # Circle center in crop coordinates
        crop_cx = cx - x1
        crop_cy = cy - y1

        # ── Step 2: find content using exact circle center ─────────────────
        inner, inner_method = _find_content_in_circle(white_crop, crop_cx, crop_cy, r)
        if inner is not None:
            return inner, f"white_circle+{inner_method}"

        # Fallback: 80 % center crop of white circle
        s = int(r * 1.6)
        x1f = max(0, crop_cx - s // 2)
        y1f = max(0, crop_cy - s // 2)
        return white_crop[y1f:min(y1f+s, white_crop.shape[0]),
                          x1f:min(x1f+s, white_crop.shape[1])], "white_circle+center"

    # ── Fallback: dark quad perspective warp ──────────────────────────────
    approx = find_quad_from_dark(gray, h * w * 0.03)
    method = "dark_thresh"
    if approx is None:
        approx = find_quad_from_canny(gray, h * w * 0.03)
        method = "canny"
    if approx is not None:
        src = order_points(approx)
        side = 320
        dst = np.array([[0,0],[side-1,0],[side-1,side-1],[0,side-1]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(frame, M, (side, side))
        s = int(320 * 0.40)
        return warped[160-s//2:160+s//2, 160-s//2:160+s//2], method

    s = int(min(h, w) * 0.40)
    return frame[h//2-s//2:h//2+s//2, w//2-s//2:w//2+s//2], "full_frame"


ROTATIONS = [
    (0,   "0°",   None),
    (1,   "90°",  cv2.ROTATE_90_CLOCKWISE),
    (2,   "180°", cv2.ROTATE_180),
    (3,   "270°", cv2.ROTATE_90_COUNTERCLOCKWISE),
]


def predict_with_rotation(model, crop, qr_targets: set | None = None, topk: int = 20):
    """
    Run inference on all 4 rotations of crop.
    If qr_targets is given (set of label strings), return the rotation where
    the highest-ranked QR target appears earliest.
    Otherwise return the rotation with the highest top-1 confidence.
    Returns (best_probs_array, best_angle_label, best_rotation_code).
    """
    best_probs, best_angle, best_rot_code = None, "0°", None
    best_score = -1.0

    for _, angle, rot_code in ROTATIONS:
        rotated = cv2.rotate(crop, rot_code) if rot_code is not None else crop
        results = model(rotated, verbose=False)
        probs = results[0].probs.data.cpu().numpy()

        if qr_targets:
            ranked = sorted(enumerate(probs), key=lambda x: -x[1])
            for rank, (idx, p) in enumerate(ranked[:topk], 1):
                if model.names[idx].lower() in qr_targets:
                    # lower rank index = better; use negative rank as score
                    score = p / rank
                    if score > best_score:
                        best_score, best_probs, best_angle, best_rot_code = score, probs, angle, rot_code
                    break
        else:
            top1_conf = float(probs.max())
            if top1_conf > best_score:
                best_score, best_probs, best_angle, best_rot_code = top1_conf, probs, angle, rot_code

    if best_probs is None:
        best_probs = model(crop, verbose=False)[0].probs.data.cpu().numpy()
    return best_probs, best_angle


def main():
    img_path = sys.argv[1] if len(sys.argv) > 1 else None
    if not img_path or not Path(img_path).exists():
        print("Usage: python3 test_local_inference.py <image_path>")
        sys.exit(1)

    frame = cv2.imread(img_path)
    if frame is None:
        print(f"Cannot read image: {img_path}")
        sys.exit(1)

    print(f"Image size: {frame.shape[1]}x{frame.shape[0]}")
    crop, method = crop_target(frame)
    print(f"Crop method: {method}, crop size: {crop.shape[1]}x{crop.shape[0]}")

    # Save crops for inspection
    cv2.imwrite("/tmp/crop_result.jpg", crop)
    print("Saved crop to /tmp/crop_result.jpg")

    # Run YOLO classification with rotation
    try:
        from ultralytics import YOLO
        model = YOLO(MODEL_PT)
        probs, best_angle = predict_with_rotation(model, crop)
        ranked = sorted(enumerate(probs), key=lambda x: -x[1])
        names = model.names
        print(f"\n=== YOLO Classification (models_A/best.pt) — best rotation: {best_angle} ===")
        for rank, (idx, conf) in enumerate(ranked[:5], 1):
            print(f"  #{rank}  {names[idx]:<25} {conf*100:.1f}%")
    except Exception as e:
        print(f"YOLO inference failed: {e}")

    # Also run on full frame for comparison
    try:
        from ultralytics import YOLO
        model = YOLO(MODEL_PT)
        results_full = model(frame, verbose=False)
        probs_full = results_full[0].probs
        top5_ids_f = probs_full.top5
        top5_conf_f = probs_full.top5conf.tolist()
        names_f = results_full[0].names
        print("\n=== YOLO Classification on FULL FRAME ===")
        for rank, (idx, conf) in enumerate(zip(top5_ids_f, top5_conf_f)):
            print(f"  #{rank+1}  {names_f[idx]:<25} {conf*100:.1f}%")
    except Exception as e:
        print(f"Full frame inference failed: {e}")


if __name__ == "__main__":
    main()
