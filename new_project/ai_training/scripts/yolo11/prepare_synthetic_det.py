#!/usr/bin/env python3
from __future__ import annotations

import argparse
import random
import shutil
from pathlib import Path

from PIL import Image, ImageDraw, ImageEnhance, ImageFilter


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CIFAR_DIR = ROOT / "datasets" / "cifar100_target_cls"
DEFAULT_OUT_DIR = ROOT / "datasets" / "micro_drone_det"

CLASSES = {
    0: "picture_target",
    1: "special_target",
    2: "ring",
    3: "obstacle",
    4: "landing_h",
    5: "red_light",
    6: "blue_light",
}

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def collect_cifar_images(cifar_dir: Path, split: str) -> list[Path]:
    base = cifar_dir / split
    if not base.exists():
        raise SystemExit(f"CIFAR classification split not found: {base}")
    images = sorted(p for p in base.glob("*/*") if p.suffix.lower() in IMAGE_EXTS)
    if not images:
        raise SystemExit(f"No CIFAR images found under {base}")
    return images


def reset_split(out_dir: Path, split: str) -> None:
    for kind in ("images", "labels"):
        target = out_dir / kind / split
        shutil.rmtree(target, ignore_errors=True)
        target.mkdir(parents=True, exist_ok=True)
        (target / ".gitkeep").write_text("\n", encoding="utf-8")


def make_background(rng: random.Random, size: int) -> Image.Image:
    base = Image.new(
        "RGB",
        (size, size),
        (
            rng.randint(168, 224),
            rng.randint(168, 224),
            rng.randint(168, 224),
        ),
    )
    draw = ImageDraw.Draw(base, "RGBA")
    for _ in range(rng.randint(14, 28)):
        x1 = rng.randint(-80, size)
        y1 = rng.randint(-80, size)
        x2 = x1 + rng.randint(120, 360)
        y2 = y1 + rng.randint(80, 260)
        color = (
            rng.randint(120, 245),
            rng.randint(120, 245),
            rng.randint(120, 245),
            rng.randint(12, 44),
        )
        draw.rectangle([x1, y1, x2, y2], fill=color)
    if rng.random() < 0.5:
        step = rng.choice((32, 40, 48, 64))
        for pos in range(0, size, step):
            draw.line([(pos, 0), (pos, size)], fill=(255, 255, 255, 28), width=1)
            draw.line([(0, pos), (size, pos)], fill=(255, 255, 255, 28), width=1)
    return base.filter(ImageFilter.GaussianBlur(radius=rng.uniform(0.0, 0.4)))


def paste_patch(canvas: Image.Image, patch: Image.Image, box: tuple[int, int, int, int]) -> None:
    x1, y1, x2, y2 = box
    canvas.paste(patch.resize((x2 - x1, y2 - y1), Image.Resampling.BICUBIC), (x1, y1))


def random_box(rng: random.Random, size: int, min_size: int, max_size: int) -> tuple[int, int, int, int]:
    w = rng.randint(min_size, max_size)
    h = rng.randint(min_size, max_size)
    if rng.random() < 0.55:
        h = int(w * rng.uniform(0.72, 1.25))
    w = min(w, size - 24)
    h = min(h, size - 24)
    x1 = rng.randint(12, size - w - 12)
    y1 = rng.randint(12, size - h - 12)
    return x1, y1, x1 + w, y1 + h


def label_line(class_id: int, box: tuple[int, int, int, int], size: int) -> str:
    x1, y1, x2, y2 = box
    xc = ((x1 + x2) / 2) / size
    yc = ((y1 + y2) / 2) / size
    w = (x2 - x1) / size
    h = (y2 - y1) / size
    return f"{class_id} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}"


def draw_picture_target(
    rng: random.Random,
    canvas: Image.Image,
    box: tuple[int, int, int, int],
    cifar_images: list[Path],
) -> None:
    x1, y1, x2, y2 = box
    draw = ImageDraw.Draw(canvas, "RGBA")
    margin = max(5, int((x2 - x1) * 0.08))
    draw.rectangle([x1, y1, x2, y2], fill=(245, 245, 238, 255), outline=(25, 25, 25, 255), width=max(2, margin // 2))
    patch = Image.open(rng.choice(cifar_images)).convert("RGB")
    patch = ImageEnhance.Color(patch).enhance(rng.uniform(0.85, 1.3))
    patch = ImageEnhance.Contrast(patch).enhance(rng.uniform(0.85, 1.25))
    paste_patch(canvas, patch, (x1 + margin, y1 + margin, x2 - margin, y2 - margin))


def draw_special_target(rng: random.Random, canvas: Image.Image, box: tuple[int, int, int, int]) -> None:
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    draw = ImageDraw.Draw(canvas, "RGBA")
    points = [(cx, y1), (x2, cy), (cx, y2), (x1, cy)]
    draw.polygon(points, fill=(240, 230, 52, 235), outline=(20, 20, 20, 255))
    inset = max(8, min(x2 - x1, y2 - y1) // 5)
    draw.rectangle([x1 + inset, y1 + inset, x2 - inset, y2 - inset], outline=(180, 0, 190, 255), width=max(3, inset // 4))
    if rng.random() < 0.5:
        draw.line([(x1 + inset, cy), (x2 - inset, cy)], fill=(0, 140, 220, 255), width=max(3, inset // 5))


def draw_ring(rng: random.Random, canvas: Image.Image, box: tuple[int, int, int, int]) -> None:
    draw = ImageDraw.Draw(canvas, "RGBA")
    width = max(6, (box[2] - box[0]) // rng.randint(8, 12))
    draw.ellipse(box, outline=(245, 245, 245, 255), width=width)
    inner = (box[0] + width, box[1] + width, box[2] - width, box[3] - width)
    draw.ellipse(inner, outline=(30, 30, 30, 190), width=max(2, width // 3))


def draw_obstacle(rng: random.Random, canvas: Image.Image, box: tuple[int, int, int, int]) -> None:
    x1, y1, x2, y2 = box
    draw = ImageDraw.Draw(canvas, "RGBA")
    draw.ellipse([x1, y1 - 8, x2, y1 + 14], fill=(75, 75, 82, 255), outline=(25, 25, 30, 255))
    draw.rectangle([x1, y1, x2, y2], fill=(88, 88, 96, 245), outline=(25, 25, 30, 255), width=3)
    draw.ellipse([x1, y2 - 14, x2, y2 + 8], fill=(55, 55, 62, 220), outline=(25, 25, 30, 255))
    if rng.random() < 0.6:
        draw.line([(x1 + 8, y1 + 8), (x1 + 8, y2 - 8)], fill=(150, 150, 160, 180), width=3)


def draw_landing_h(rng: random.Random, canvas: Image.Image, box: tuple[int, int, int, int]) -> None:
    x1, y1, x2, y2 = box
    draw = ImageDraw.Draw(canvas, "RGBA")
    draw.rectangle([x1, y1, x2, y2], fill=(40, 120, 80, 240), outline=(255, 255, 255, 255), width=4)
    pad = max(8, (x2 - x1) // 5)
    line_w = max(6, (x2 - x1) // 9)
    draw.line([(x1 + pad, y1 + pad), (x1 + pad, y2 - pad)], fill=(255, 255, 255, 255), width=line_w)
    draw.line([(x2 - pad, y1 + pad), (x2 - pad, y2 - pad)], fill=(255, 255, 255, 255), width=line_w)
    draw.line([(x1 + pad, (y1 + y2) // 2), (x2 - pad, (y1 + y2) // 2)], fill=(255, 255, 255, 255), width=line_w)


def draw_light(canvas: Image.Image, box: tuple[int, int, int, int], color: tuple[int, int, int]) -> None:
    draw = ImageDraw.Draw(canvas, "RGBA")
    glow = (color[0], color[1], color[2], 55)
    x1, y1, x2, y2 = box
    draw.ellipse([x1 - 8, y1 - 8, x2 + 8, y2 + 8], fill=glow)
    draw.ellipse(box, fill=(*color, 245), outline=(30, 30, 30, 255), width=3)
    highlight = (x1 + 8, y1 + 8, x1 + max(14, (x2 - x1) // 3), y1 + max(14, (y2 - y1) // 3))
    draw.ellipse(highlight, fill=(255, 255, 255, 120))


def draw_class(
    rng: random.Random,
    canvas: Image.Image,
    class_id: int,
    box: tuple[int, int, int, int],
    cifar_images: list[Path],
) -> None:
    if class_id == 0:
        draw_picture_target(rng, canvas, box, cifar_images)
    elif class_id == 1:
        draw_special_target(rng, canvas, box)
    elif class_id == 2:
        draw_ring(rng, canvas, box)
    elif class_id == 3:
        draw_obstacle(rng, canvas, box)
    elif class_id == 4:
        draw_landing_h(rng, canvas, box)
    elif class_id == 5:
        draw_light(canvas, box, (235, 35, 30))
    elif class_id == 6:
        draw_light(canvas, box, (35, 85, 240))
    else:
        raise ValueError(class_id)


def write_split(
    split: str,
    out_dir: Path,
    cifar_images: list[Path],
    count: int,
    image_size: int,
    seed: int,
) -> None:
    rng = random.Random(seed)
    image_dir = out_dir / "images" / split
    label_dir = out_dir / "labels" / split
    for idx in range(count):
        canvas = make_background(rng, image_size)
        labels: list[str] = []

        primary = idx % len(CLASSES)
        class_ids = [primary]
        if rng.random() < 0.45:
            class_ids.append(rng.randrange(len(CLASSES)))
        if rng.random() < 0.18:
            class_ids.append(rng.randrange(len(CLASSES)))

        for class_id in class_ids:
            if class_id in (5, 6):
                box = random_box(rng, image_size, 38, 100)
            elif class_id == 3:
                box = random_box(rng, image_size, 70, 170)
            else:
                box = random_box(rng, image_size, 70, 190)
            draw_class(rng, canvas, class_id, box, cifar_images)
            labels.append(label_line(class_id, box, image_size))

        if rng.random() < 0.4:
            canvas = ImageEnhance.Brightness(canvas).enhance(rng.uniform(0.78, 1.18))
        if rng.random() < 0.35:
            canvas = canvas.filter(ImageFilter.GaussianBlur(radius=rng.uniform(0.2, 0.8)))

        stem = f"synthetic_{split}_{idx:05d}"
        canvas.save(image_dir / f"{stem}.jpg", quality=rng.randint(82, 95))
        (label_dir / f"{stem}.txt").write_text("\n".join(labels) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a YOLO detection smoke-test dataset from CIFAR-100 target images.")
    parser.add_argument("--cifar-dir", type=Path, default=DEFAULT_CIFAR_DIR)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--train-count", type=int, default=1400)
    parser.add_argument("--val-count", type=int, default=350)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--seed", type=int, default=20260512)
    parser.add_argument("--clean", action="store_true")
    args = parser.parse_args()

    if args.clean:
        reset_split(args.out_dir, "train")
        reset_split(args.out_dir, "val")
    else:
        for split in ("train", "val"):
            (args.out_dir / "images" / split).mkdir(parents=True, exist_ok=True)
            (args.out_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

    train_cifar = collect_cifar_images(args.cifar_dir, "train")
    val_cifar = collect_cifar_images(args.cifar_dir, "val")
    write_split("train", args.out_dir, train_cifar, args.train_count, args.image_size, args.seed)
    write_split("val", args.out_dir, val_cifar, args.val_count, args.image_size, args.seed + 1)

    print(f"Wrote synthetic detection data to {args.out_dir}")
    print(f"  train images: {args.train_count}")
    print(f"  val images: {args.val_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
