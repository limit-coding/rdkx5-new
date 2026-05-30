#!/usr/bin/env python3
"""
把 CIFAR-100 图片合成到仿真靶板上，生成接近无人机俯拍真实场景的训练数据。

靶板结构（参考 IMG_4101~4104）：
  - 黑色方形底板
  - 深灰色圆环
  - 白色内圆
  - 中心贴 CIFAR-100 小图（模拟打印模糊）

用法：
  python3 prepare_board_aug.py
  python3 prepare_board_aug.py --help
"""
from __future__ import annotations

import argparse
import pickle
import random
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RAW_DIR = ROOT / "datasets" / "raw" / "cifar-100-python"
DEFAULT_OUT_DIR = ROOT / "datasets" / "cifar100_board_aug"

# 合成靶板尺寸（像素），最终会缩放到 image_size
BOARD_SIZE = 512


def load_pickle(path: Path) -> dict:
    with path.open("rb") as f:
        return pickle.load(f, encoding="latin1")


def load_cifar100(raw_dir: Path):
    meta = load_pickle(raw_dir / "meta")
    train_d = load_pickle(raw_dir / "train")
    test_d = load_pickle(raw_dir / "test")
    names = [n.decode() if isinstance(n, bytes) else n for n in meta["fine_label_names"]]

    def decode(d):
        imgs = d["data"].reshape(-1, 3, 32, 32).transpose(0, 2, 3, 1)
        return imgs, list(d["fine_labels"])

    return names, *decode(train_d), *decode(test_d)


def make_floor_background(size: int) -> np.ndarray:
    """生成仿泡沫地垫背景（绿黄格子）。"""
    bg = np.zeros((size, size, 3), dtype=np.uint8)
    tile = max(size // 8, 20)
    colors = [
        (34, 139, 34),   # 绿
        (154, 205, 50),  # 黄绿
        (50, 180, 50),
        (180, 200, 30),
    ]
    for row in range(0, size, tile):
        for col in range(0, size, tile):
            c = colors[((row // tile) + (col // tile)) % len(colors)]
            # 随机偏色
            jitter = [random.randint(-20, 20) for _ in range(3)]
            c_j = tuple(max(0, min(255, c[i] + jitter[i])) for i in range(3))
            bg[row:row+tile, col:col+tile] = c_j

    # 加轻微噪声
    noise = np.random.randint(-15, 15, bg.shape, dtype=np.int16)
    bg = np.clip(bg.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return bg


def make_board(cifar_rgb: np.ndarray, train: bool) -> np.ndarray:
    """
    把一张 32×32 CIFAR-100 图合成到仿真靶板上，返回 BOARD_SIZE×BOARD_SIZE BGR 图。
    """
    S = BOARD_SIZE
    cx, cy = S // 2, S // 2

    # ---------- 靶板尺寸随机浮动（模拟不同飞行高度） ----------
    board_scale = random.uniform(0.55, 0.90) if train else 0.72
    board_half = int(S * board_scale / 2)

    # ---------- 生成地板背景 ----------
    canvas = make_floor_background(S)

    # ---------- 绘制靶板 ----------
    # 黑色方形
    x0, y0 = cx - board_half, cy - board_half
    x1, y1 = cx + board_half, cy + board_half
    cv2.rectangle(canvas, (x0, y0), (x1, y1), (15, 15, 15), -1)

    # 深灰色圆（外环）
    outer_r = int(board_half * random.uniform(0.88, 0.96) if train else board_half * 0.92)
    gray_outer = random.randint(80, 115) if train else 95
    cv2.circle(canvas, (cx, cy), outer_r, (gray_outer,) * 3, -1)

    # 黑色遮罩（让外环看起来是环而不是实心圆）
    inner_border_r = int(outer_r * random.uniform(0.72, 0.82) if train else outer_r * 0.77)
    cv2.circle(canvas, (cx, cy), inner_border_r, (15, 15, 15), -1)

    # 白/浅灰色内圆
    white_r = int(inner_border_r * random.uniform(0.82, 0.95) if train else inner_border_r * 0.88)
    white_val = random.randint(215, 245) if train else 230
    cv2.circle(canvas, (cx, cy), white_r, (white_val,) * 3, -1)

    # ---------- 贴 CIFAR 小图 ----------
    # 图像尺寸：白圆直径的 35~50%
    img_size = int(white_r * 2 * random.uniform(0.35, 0.50) if train else white_r * 2 * 0.42)
    img_size = max(img_size, 20)

    # 从 32×32 上采样（模拟打印模糊）
    cifar_bgr = cv2.cvtColor(cifar_rgb, cv2.COLOR_RGB2BGR)
    upscale = cv2.resize(cifar_bgr, (img_size, img_size), interpolation=cv2.INTER_CUBIC)

    # 模拟打印模糊
    blur_k = random.choice([3, 5, 7]) if train else 5
    upscale = cv2.GaussianBlur(upscale, (blur_k, blur_k), sigmaX=random.uniform(0.5, 1.8))

    # 粘贴到画布中心（轻微偏移）
    offset_x = random.randint(-4, 4) if train else 0
    offset_y = random.randint(-4, 4) if train else 0
    px = cx - img_size // 2 + offset_x
    py = cy - img_size // 2 + offset_y
    px = max(0, min(S - img_size, px))
    py = max(0, min(S - img_size, py))
    canvas[py:py+img_size, px:px+img_size] = upscale

    return canvas


def apply_drone_aug(img: np.ndarray, size: int, train: bool) -> np.ndarray:
    """透视、旋转、亮度、噪声等俯拍增强。"""
    S = img.shape[0]

    if train:
        # 透视变形（模拟摄像头不完全垂直）
        jitter = random.uniform(0.0, 0.07) * S
        src = np.float32([[0,0],[S-1,0],[S-1,S-1],[0,S-1]])
        dst = src + np.float32([[random.uniform(-jitter,jitter),
                                  random.uniform(-jitter,jitter)] for _ in range(4)])
        M = cv2.getPerspectiveTransform(src, dst)
        img = cv2.warpPerspective(img, M, (S, S), borderMode=cv2.BORDER_REFLECT_101)

        # 旋转（无人机 yaw 偏差）
        angle = random.uniform(-30, 30)
        scale = random.uniform(0.90, 1.05)
        M2 = cv2.getRotationMatrix2D((S/2, S/2), angle, scale)
        img = cv2.warpAffine(img, M2, (S, S), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_REFLECT_101)

        # 亮度 / 对比度
        alpha = random.uniform(0.7, 1.35)
        beta  = random.uniform(-30, 30)
        img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

        # 运动模糊（无人机移动）
        if random.random() < 0.3:
            k = random.choice([3, 5])
            kernel = np.zeros((k, k))
            kernel[k//2, :] = 1.0 / k
            if random.random() < 0.5:
                kernel = kernel.T
            img = cv2.filter2D(img, -1, kernel)

        # 噪声
        if random.random() < 0.5:
            noise = np.random.normal(0, random.uniform(3, 12), img.shape).astype(np.int16)
            img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # 随机遮挡（模拟部分阴影）
        if random.random() < 0.2:
            x1 = random.randint(0, S-40)
            y1 = random.randint(0, S-40)
            w  = random.randint(20, S//3)
            h  = random.randint(20, S//3)
            overlay = img.copy()
            cv2.rectangle(overlay, (x1,y1), (x1+w,y1+h), (0,0,0), -1)
            img = cv2.addWeighted(img, 0.7, overlay, 0.3, 0)

    # 最终缩放到目标尺寸
    img = cv2.resize(img, (size, size), interpolation=cv2.INTER_AREA)

    # JPEG 压缩（模拟视频流画质）
    q = random.randint(65, 90) if train else 88
    _, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return cv2.imdecode(enc, cv2.IMREAD_COLOR)


def write_split(
    images: np.ndarray,
    labels: list[int],
    names: list[str],
    out_dir: Path,
    split_name: str,
    max_per_class: int,
    augs_per_image: int,
    image_size: int,
    seed: int,
    train: bool,
) -> int:
    rng = random.Random(seed)

    # 按类别分组
    groups: dict[int, list[int]] = {i: [] for i in range(len(names))}
    for idx, label in enumerate(labels):
        groups[label].append(idx)

    count = 0
    for cls_id, cls_name in enumerate(names):
        indices = groups[cls_id][:]
        rng.shuffle(indices)
        if max_per_class > 0:
            indices = indices[:max_per_class]

        cls_dir = out_dir / split_name / cls_name.replace(" ", "_")
        cls_dir.mkdir(parents=True, exist_ok=True)

        for item_no, img_idx in enumerate(indices):
            for aug_no in range(augs_per_image):
                random.seed(seed + cls_id * 100000 + item_no * 100 + aug_no)
                np.random.seed((seed + cls_id * 100000 + item_no * 100 + aug_no) % (2**32 - 1))

                board = make_board(images[img_idx], train=train)
                final = apply_drone_aug(board, image_size, train=train)

                out_path = cls_dir / f"{cls_name}_{img_idx:05d}_{aug_no:02d}.jpg"
                cv2.imwrite(str(out_path), final)
                count += 1

    return count


def main() -> int:
    parser = argparse.ArgumentParser(description="生成靶板合成增强训练数据")
    parser.add_argument("--raw-dir",  type=Path, default=DEFAULT_RAW_DIR)
    parser.add_argument("--out-dir",  type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--image-size", type=int, default=224)
    parser.add_argument("--max-train-per-class", type=int, default=150,
                        help="每类最多取多少张原图（原图×augs=训练数据量）")
    parser.add_argument("--train-augs", type=int, default=3,
                        help="每张原图生成几个增强样本")
    parser.add_argument("--max-val-per-class", type=int, default=30)
    parser.add_argument("--val-augs", type=int, default=1)
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()

    raw = args.raw_dir
    if not (raw / "train").exists():
        raise SystemExit(
            f"找不到 CIFAR-100 原始数据：{raw}\n"
            "请先运行：scripts/classification/download_cifar100.sh"
        )

    print("加载 CIFAR-100...")
    names, tr_imgs, tr_labels, te_imgs, te_labels = load_cifar100(raw)
    print(f"  train: {len(tr_labels)} 张  val: {len(te_labels)} 张  类别: {len(names)}")

    print("生成训练集...")
    n_train = write_split(tr_imgs, tr_labels, names, args.out_dir, "train",
                          args.max_train_per_class, args.train_augs,
                          args.image_size, args.seed, train=True)

    print("生成验证集...")
    n_val = write_split(te_imgs, te_labels, names, args.out_dir, "val",
                        args.max_val_per_class, args.val_augs,
                        args.image_size, args.seed + 1, train=False)

    print(f"\n完成！train={n_train} 张  val={n_val} 张")
    print(f"输出目录：{args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
