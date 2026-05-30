#!/usr/bin/env python3
from __future__ import annotations

import argparse
import pickle
import random
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RAW_DIR = ROOT / "datasets" / "raw" / "cifar-100-python"
DEFAULT_OUT_DIR = ROOT / "datasets" / "cifar100_target_cls"


@dataclass(frozen=True)
class CifarSplit:
    images: np.ndarray
    labels: list[int]


def load_pickle(path: Path) -> dict:
    with path.open("rb") as f:
        return pickle.load(f, encoding="latin1")


def load_cifar100(raw_dir: Path) -> tuple[list[str], CifarSplit, CifarSplit]:
    meta = load_pickle(raw_dir / "meta")
    train = load_pickle(raw_dir / "train")
    test = load_pickle(raw_dir / "test")

    names = [name.decode("utf-8") if isinstance(name, bytes) else name for name in meta["fine_label_names"]]

    def decode(split: dict) -> CifarSplit:
        data = split["data"].reshape(-1, 3, 32, 32).transpose(0, 2, 3, 1)
        labels = list(split["fine_labels"])
        return CifarSplit(images=data, labels=labels)

    return names, decode(train), decode(test)


def safe_name(name: str) -> str:
    return name.replace(" ", "_").replace("/", "_")


def apply_camera_like_aug(rgb: np.ndarray, size: int, train: bool) -> np.ndarray:
    img = cv2.resize(rgb, (size, size), interpolation=cv2.INTER_CUBIC)

    if train:
        # Simulate imperfect target-center crops from OpenCV perspective correction.
        pad = random.randint(0, 18)
        if pad:
            canvas = np.full((size + 2 * pad, size + 2 * pad, 3), random.randint(210, 245), dtype=np.uint8)
            canvas[pad : pad + size, pad : pad + size] = img
            img = cv2.resize(canvas, (size, size), interpolation=cv2.INTER_AREA)

        jitter = random.uniform(0.0, 0.08) * size
        src = np.float32([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]])
        dst = src + np.float32([[random.uniform(-jitter, jitter), random.uniform(-jitter, jitter)] for _ in range(4)])
        matrix = cv2.getPerspectiveTransform(src, dst)
        img = cv2.warpPerspective(img, matrix, (size, size), borderMode=cv2.BORDER_REFLECT_101)

        angle = random.uniform(-18, 18)
        scale = random.uniform(0.88, 1.08)
        matrix = cv2.getRotationMatrix2D((size / 2, size / 2), angle, scale)
        img = cv2.warpAffine(img, matrix, (size, size), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)

        alpha = random.uniform(0.75, 1.25)
        beta = random.uniform(-24, 24)
        img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

        if random.random() < 0.45:
            kernel = random.choice((3, 5))
            img = cv2.GaussianBlur(img, (kernel, kernel), sigmaX=random.uniform(0.2, 1.2))

        if random.random() < 0.5:
            noise = np.random.normal(0, random.uniform(2, 9), img.shape).astype(np.int16)
            img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        if random.random() < 0.35:
            x1 = random.randint(0, size - 28)
            y1 = random.randint(0, size - 28)
            x2 = min(size, x1 + random.randint(12, 42))
            y2 = min(size, y1 + random.randint(12, 42))
            img[y1:y2, x1:x2] = np.clip(img[y1:y2, x1:x2].astype(np.int16) + random.randint(-35, 35), 0, 255)

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), random.randint(70, 95) if train else 92]
    ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR), encode_param)
    if not ok:
        raise RuntimeError("failed to encode augmented image")
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def grouped_indices(labels: list[int], class_count: int) -> dict[int, list[int]]:
    groups = {i: [] for i in range(class_count)}
    for idx, label in enumerate(labels):
        groups[label].append(idx)
    return groups


def write_split(
    split: CifarSplit,
    names: list[str],
    out_dir: Path,
    split_name: str,
    max_per_class: int,
    augmentations: int,
    image_size: int,
    seed: int,
) -> None:
    rng = random.Random(seed)
    groups = grouped_indices(split.labels, len(names))

    for class_id, class_name in enumerate(names):
        indices = groups[class_id][:]
        rng.shuffle(indices)
        if max_per_class > 0:
            indices = indices[:max_per_class]

        class_dir = out_dir / split_name / safe_name(class_name)
        class_dir.mkdir(parents=True, exist_ok=True)

        for item_no, image_idx in enumerate(indices):
            rgb = split.images[image_idx]
            for aug_no in range(augmentations):
                random.seed(seed + class_id * 100000 + item_no * 100 + aug_no)
                np.random.seed((seed + class_id * 100000 + item_no * 100 + aug_no) % (2**32 - 1))
                img = apply_camera_like_aug(rgb, image_size, train=(split_name == "train"))
                out_path = class_dir / f"{safe_name(class_name)}_{image_idx:05d}_{aug_no:02d}.jpg"
                cv2.imwrite(str(out_path), img)


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare augmented CIFAR-100 data for target-center classification.")
    parser.add_argument("--raw-dir", type=Path, default=DEFAULT_RAW_DIR)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--image-size", type=int, default=224)
    parser.add_argument("--max-train-per-class", type=int, default=120)
    parser.add_argument("--max-val-per-class", type=int, default=30)
    parser.add_argument("--train-augmentations", type=int, default=2)
    parser.add_argument("--val-augmentations", type=int, default=1)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--clean", action="store_true", help="Remove existing train/val folders before writing.")
    args = parser.parse_args()

    if not (args.raw_dir / "train").exists():
        raise SystemExit(f"CIFAR-100 not found at {args.raw_dir}. Run scripts/classification/download_cifar100.sh first.")

    if args.clean:
        shutil.rmtree(args.out_dir / "train", ignore_errors=True)
        shutil.rmtree(args.out_dir / "val", ignore_errors=True)

    names, train, test = load_cifar100(args.raw_dir)
    write_split(
        train,
        names,
        args.out_dir,
        "train",
        args.max_train_per_class,
        args.train_augmentations,
        args.image_size,
        args.seed,
    )
    write_split(
        test,
        names,
        args.out_dir,
        "val",
        args.max_val_per_class,
        args.val_augmentations,
        args.image_size,
        args.seed + 1,
    )

    train_count = sum(1 for _ in (args.out_dir / "train").glob("*/*.jpg"))
    val_count = sum(1 for _ in (args.out_dir / "val").glob("*/*.jpg"))
    print(f"Wrote {train_count} train images and {val_count} val images to {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
