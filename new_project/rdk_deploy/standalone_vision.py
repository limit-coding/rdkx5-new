#!/usr/bin/env python3
"""
Standalone vision debug script — reads /image topic, no FC needed.

Usage:
  python3 standalone_vision.py

Stop:
  Ctrl+C
"""

import os
import sys
import time

import cv2
import numpy as np

# ── Config ──────────────────────────────────────────────────────────────────
IMAGE_TOPIC = "/image"
ROTATE_180  = True

MODEL_PATH = "/home/sunrise/project/camera/resource/cifar100_cls.onnx"
NAMES_PATH = "/home/sunrise/project/camera/resource/cifar100_names.txt"

QR_CONFIRM_FRAMES  = 3    # consecutive frames to lock QR
CLS_CONFIRM_FRAMES = 3    # consecutive frames with target in top-10 → LAND
CLS_REJECT_FRAMES  = 5    # consecutive frames without target in top-10 → FLY AWAY
TARGET_RANK_K      = 10   # scan top-10 for QR target
TARGET_MIN_SCORE   = 0.005
EARLY_EXIT_RANK    = 10   # stop rotating if QR target already in top-N
EARLY_EXIT_CONF    = 0.30 # … and confidence ≥ this


# ── QR parsing ───────────────────────────────────────────────────────────────
def parse_qr_mission(text: str):
    """Return (class1, class2, landing_side) or None."""
    ignored = {"class","class1","class2","target","target1","target2",
               "side","landing","landing_side","land","qr","task"}
    for ch in "，、：；,;|:=/\\[]{}()\"'":
        text = text.replace(ch, " ")
    tokens = text.split()
    classes, side = [], ""
    for t in tokens:
        lo = t.lower()
        if lo in ("left", "right"):
            side = lo
        elif lo not in ignored:
            classes.append(t)
    if len(classes) >= 2 and side:
        return classes[0], classes[1], side
    return None


# ── Crop helpers ─────────────────────────────────────────────────────────────
def _order_points(points: np.ndarray) -> np.ndarray:
    pts = points.reshape(4, 2).astype(np.float32)
    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)
    return np.array([pts[np.argmin(sums)], pts[np.argmin(diffs)],
                     pts[np.argmax(sums)], pts[np.argmax(diffs)]], dtype=np.float32)


def _find_white_circle(gray: np.ndarray, min_area: float):
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
            margin = int(r * 0.05)
            if (cx - r < margin or cx + r > w - margin or
                    cy - r < margin or cy + r > h - margin):
                continue
            best, best_r = (int(cx), int(cy), int(r)), r
        if best:
            break
    return best


def _find_content_in_circle(white_crop: np.ndarray, crop_cx: int,
                             crop_cy: int, r_white: int):
    h, w = white_crop.shape[:2]
    gray = cv2.cvtColor(white_crop, cv2.COLOR_BGR2GRAY)
    min_card = (r_white * 0.20) ** 2
    max_card = (r_white * 1.6) ** 2

    # Strategy A: Canny rectangle
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    best_approx, best_area = None, 0.0
    for lo, hi in [(20, 80), (30, 100), (50, 150)]:
        edges = cv2.Canny(blurred, lo, hi)
        cnts, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_card or area > max_card or area <= best_area:
                continue
            hull = cv2.convexHull(c)
            peri = cv2.arcLength(hull, True)
            for eps in (0.02, 0.04, 0.06, 0.10):
                approx = cv2.approxPolyDP(hull, eps * peri, True)
                if len(approx) == 4:
                    best_approx, best_area = approx, area
                    break
    if best_approx is not None:
        src = _order_points(best_approx)
        side = 224
        dst = np.array([[0,0],[side-1,0],[side-1,side-1],[0,side-1]], dtype=np.float32)
        return cv2.warpPerspective(white_crop, cv2.getPerspectiveTransform(src, dst), (side, side))

    # Strategy B: non-white content mask
    yy, xx = np.ogrid[:h, :w]
    circ_mask = ((xx - crop_cx)**2 + (yy - crop_cy)**2 <= r_white**2).astype(np.uint8) * 255
    hsv = cv2.cvtColor(white_crop, cv2.COLOR_BGR2HSV)
    white_bg = cv2.inRange(hsv, (0, 0, 185), (180, 30, 255))
    content = cv2.bitwise_and(cv2.bitwise_not(white_bg), circ_mask)
    ksize = max(5, r_white // 15)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    content = cv2.morphologyEx(content, cv2.MORPH_CLOSE, k)
    content = cv2.morphologyEx(content, cv2.MORPH_OPEN, k)
    cnts, _ = cv2.findContours(content, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    largest = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(largest) <= min_card * 0.2:
        return None

    # Perspective warp for colored-background cards (fill ≥ 35%)
    if cv2.contourArea(largest) / (np.pi * r_white * r_white) >= 0.35:
        hull = cv2.convexHull(largest)
        peri = cv2.arcLength(hull, True)
        for eps in (0.02, 0.04, 0.06, 0.10):
            approx = cv2.approxPolyDP(hull, eps * peri, True)
            if len(approx) != 4:
                continue
            pts = _order_points(approx)
            sides = [np.linalg.norm(pts[(i+1)%4] - pts[i]) for i in range(4)]
            if min(sides) < r_white * 0.20:
                continue
            side = 224
            dst = np.array([[0,0],[side-1,0],[side-1,side-1],[0,side-1]], dtype=np.float32)
            return cv2.warpPerspective(white_crop, cv2.getPerspectiveTransform(pts, dst), (side, side))

    # Bbox fallback
    bx, by, bw, bh = cv2.boundingRect(largest)
    pad = int(max(bw, bh) * 0.15)
    s = max(bw, bh) + 2 * pad
    cx2, cy2 = bx + bw // 2, by + bh // 2
    x1, y1 = max(0, cx2 - s // 2), max(0, cy2 - s // 2)
    return white_crop[y1:min(h, y1+s), x1:min(w, x1+s)]


def _find_quad_dark(gray, min_area):
    h, w = gray.shape
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    best_approx, best_area = None, 0.0
    for thresh in (70, 90, 110, 130, 150):
        _, dark = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY_INV)
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


def crop_target(frame: np.ndarray) -> np.ndarray:
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    wc = _find_white_circle(gray, h * w * 0.01)
    if wc is not None:
        cx, cy, r = wc
        x1, y1 = max(0, cx-r), max(0, cy-r)
        x2, y2 = min(w, cx+r), min(h, cy+r)
        white_crop = frame[y1:y2, x1:x2]
        inner = _find_content_in_circle(white_crop, cx-x1, cy-y1, r)
        if inner is not None:
            return inner
        s = int(r * 1.6)
        crop_cx, crop_cy = cx-x1, cy-y1
        x1f = max(0, crop_cx - s//2)
        y1f = max(0, crop_cy - s//2)
        patch = white_crop[y1f:min(y1f+s, white_crop.shape[0]),
                           x1f:min(x1f+s, white_crop.shape[1])]
        if patch.size > 0:
            return patch

    approx = _find_quad_dark(gray, h * w * 0.03)
    if approx is not None:
        src = _order_points(approx)
        side = 320
        dst = np.array([[0,0],[side-1,0],[side-1,side-1],[0,side-1]], dtype=np.float32)
        warped = cv2.warpPerspective(frame, cv2.getPerspectiveTransform(src, dst), (side, side))
        s = int(320 * 0.40)
        return warped[160-s//2:160+s//2, 160-s//2:160+s//2]

    side = int(min(h, w) * 0.40)
    return frame[h//2-side//2:h//2+side//2, w//2-side//2:w//2+side//2]


# ── Classifier ────────────────────────────────────────────────────────────────
class Classifier:
    def __init__(self, model_path: str, names_path: str):
        import onnxruntime as ort
        os.environ.setdefault("ORT_LOGGING_LEVEL", "3")
        try:
            lines = open(names_path, encoding="utf-8").read().splitlines()
            self.names = [l.strip() for l in lines if l.strip()] or [str(i) for i in range(100)]
        except Exception:
            self.names = [str(i) for i in range(100)]
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 2
        opts.inter_op_num_threads = 1
        sess = ort.InferenceSession(model_path, sess_options=opts, providers=["CPUExecutionProvider"])
        self._sess = sess
        self._in = sess.get_inputs()[0].name
        shape = sess.get_inputs()[0].shape
        self.ih = int(shape[2]) if isinstance(shape[2], int) else 224
        self.iw = int(shape[3]) if isinstance(shape[3], int) else 224

    def _infer(self, bgr: np.ndarray) -> np.ndarray:
        img = cv2.resize(bgr, (self.iw, self.ih), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        return self._sess.run(None, {self._in: rgb.transpose(2,0,1)[np.newaxis]})[0].reshape(-1)

    def predict(self, bgr: np.ndarray, qr_targets: set | None = None):
        """Return (probs, angle_str). Uses lazy 4-rotation TTA."""
        rotations = [
            (None,                          "0°"),
            (cv2.ROTATE_90_CLOCKWISE,       "90°"),
            (cv2.ROTATE_180,                "180°"),
            (cv2.ROTATE_90_COUNTERCLOCKWISE,"270°"),
        ]
        best_probs, best_angle, best_score = None, "0°", -1.0
        for rot_code, angle in rotations:
            img = cv2.rotate(bgr, rot_code) if rot_code is not None else bgr
            probs = self._infer(img)
            if qr_targets:
                for rank, idx in enumerate(np.argsort(-probs)[:20], 1):
                    label = self.names[int(idx)].lower() if int(idx) < len(self.names) else ""
                    if label in qr_targets:
                        score = float(probs[idx]) / rank
                        if score > best_score:
                            best_score, best_probs, best_angle = score, probs, angle
                        if rank <= EARLY_EXIT_RANK and float(probs[idx]) >= EARLY_EXIT_CONF:
                            return best_probs, best_angle
                        break
            else:
                s = float(probs.max())
                if s > best_score:
                    best_score, best_probs, best_angle = s, probs, angle
                if s >= 0.60:
                    break
        return (best_probs if best_probs is not None else self._infer(bgr)), best_angle


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    os.environ["ORT_LOGGING_LEVEL"] = "3"

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage

    clf = Classifier(MODEL_PATH, NAMES_PATH)
    qr_det = cv2.QRCodeDetector()

    phase = "qr"
    qr_text, qr_count = "", 0
    qr_targets: set = set()
    landing_side = ""
    hit_count = 0
    miss_count = 0

    def on_image(msg: CompressedImage):
        nonlocal phase, qr_text, qr_count, qr_targets, landing_side, hit_count, miss_count

        arr = np.frombuffer(bytes(msg.data), np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        if ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        # ── QR phase ────────────────────────────────────────────────────────
        if phase == "qr":
            text, _, _ = qr_det.detectAndDecode(frame)
            text = text.strip() if text else ""
            if not text:
                return
            mission = parse_qr_mission(text)
            if mission is None:
                return
            if text == qr_text:
                qr_count += 1
            else:
                qr_text, qr_count = text, 1
            if qr_count >= QR_CONFIRM_FRAMES:
                c1, c2, landing_side = mission
                qr_targets = {c1.strip().lower(), c2.strip().lower()}
                print(f"QR OK  targets={sorted(qr_targets)}  landing={landing_side}")
                phase = "cls"
                hit_count = miss_count = 0

        # ── CLS phase ────────────────────────────────────────────────────────
        else:
            crop = crop_target(frame)
            probs, _ = clf.predict(crop, qr_targets=qr_targets)
            ranked = np.argsort(-probs)

            tgt_rank, tgt_label, tgt_conf = None, "", 0.0
            for rank, idx in enumerate(ranked[:TARGET_RANK_K], 1):
                name = clf.names[int(idx)].lower() if int(idx) < len(clf.names) else ""
                if name in qr_targets:
                    tgt_rank, tgt_label, tgt_conf = rank, name, float(probs[idx])
                    break

            if tgt_rank is not None and tgt_conf >= TARGET_MIN_SCORE:
                hit_count += 1
                miss_count = 0
                if hit_count >= CLS_CONFIRM_FRAMES:
                    print(f"TARGET: {tgt_label} {tgt_conf*100:.1f}%  landing={landing_side}  -> LAND")
                    hit_count = miss_count = 0
            else:
                miss_count += 1
                hit_count = 0
                if miss_count >= CLS_REJECT_FRAMES:
                    print(f"NOT TARGET  -> FLY AWAY")
                    miss_count = hit_count = 0

    rclpy.init()
    node = Node("standalone_vision")
    node.create_subscription(CompressedImage, IMAGE_TOPIC, on_image, 1)
    print("Standalone Vision  |  Ctrl+C to stop")
    print("Scanning QR...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
