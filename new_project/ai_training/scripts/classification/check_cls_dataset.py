#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DATASET = ROOT / "datasets" / "cifar100_target_cls"
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def count_images(split: str) -> tuple[int, list[tuple[str, int]]]:
    split_dir = DATASET / split
    rows: list[tuple[str, int]] = []
    total = 0
    for class_dir in sorted(p for p in split_dir.iterdir() if p.is_dir()):
        count = sum(1 for p in class_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)
        rows.append((class_dir.name, count))
        total += count
    return total, rows


def main() -> int:
    failed = False
    for split in ("train", "val"):
        total, rows = count_images(split)
        print(f"\n[{split}] classes: {len(rows)}, images: {total}")
        empty = [name for name, count in rows if count == 0]
        for name, count in rows[:10]:
            print(f"  {name}: {count}")
        if len(rows) > 10:
            print("  ...")
        if len(rows) != 100:
            print(f"ERROR: expected 100 classes in {split}, got {len(rows)}")
            failed = True
        if empty:
            print(f"ERROR: empty classes in {split}: {', '.join(empty[:20])}")
            failed = True

    if failed:
        return 1
    print("\nClassification dataset check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
