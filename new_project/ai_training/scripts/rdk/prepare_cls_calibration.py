#!/usr/bin/env python3
from __future__ import annotations

import argparse
import random
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SRC = ROOT / "datasets" / "cifar100_target_cls" / "val"
DEFAULT_DST = ROOT / "rdk_convert" / "cifar100_cls" / "calibration_images"
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare calibration images for RDK X5 CIFAR-100 classifier conversion.")
    parser.add_argument("--src", type=Path, default=DEFAULT_SRC)
    parser.add_argument("--dst", type=Path, default=DEFAULT_DST)
    parser.add_argument("--count", type=int, default=300)
    parser.add_argument("--seed", type=int, default=20260512)
    parser.add_argument("--clean", action="store_true")
    args = parser.parse_args()

    images = sorted(p for p in args.src.glob("*/*") if p.suffix.lower() in IMAGE_EXTS)
    if not images:
        raise SystemExit(f"No calibration images found under {args.src}")

    rng = random.Random(args.seed)
    rng.shuffle(images)
    selected = images[: min(args.count, len(images))]

    if args.clean:
        shutil.rmtree(args.dst, ignore_errors=True)
    args.dst.mkdir(parents=True, exist_ok=True)

    for idx, src in enumerate(selected):
        shutil.copy2(src, args.dst / f"calib_{idx:04d}{src.suffix.lower()}")

    print(f"Wrote {len(selected)} calibration images to {args.dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
