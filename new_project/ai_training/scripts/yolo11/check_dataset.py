#!/usr/bin/env python3
from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DATASET = ROOT / "datasets" / "micro_drone_det"
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


def iter_images(split: str) -> list[Path]:
    image_dir = DATASET / "images" / split
    if not image_dir.exists():
        return []
    return sorted(p for p in image_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)


def check_label(label_path: Path) -> tuple[list[str], Counter[int]]:
    errors: list[str] = []
    counts: Counter[int] = Counter()

    if not label_path.exists():
        return [f"missing label: {label_path.relative_to(ROOT)}"], counts

    for line_no, raw in enumerate(label_path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 5:
            errors.append(f"{label_path.relative_to(ROOT)}:{line_no}: expected 5 fields")
            continue
        try:
            class_id = int(parts[0])
            coords = [float(v) for v in parts[1:]]
        except ValueError:
            errors.append(f"{label_path.relative_to(ROOT)}:{line_no}: non-numeric value")
            continue
        if class_id not in CLASSES:
            errors.append(f"{label_path.relative_to(ROOT)}:{line_no}: unknown class {class_id}")
            continue
        if any(v < 0.0 or v > 1.0 for v in coords):
            errors.append(f"{label_path.relative_to(ROOT)}:{line_no}: coord outside [0, 1]")
            continue
        if coords[2] <= 0.0 or coords[3] <= 0.0:
            errors.append(f"{label_path.relative_to(ROOT)}:{line_no}: width/height must be > 0")
            continue
        counts[class_id] += 1

    return errors, counts


def main() -> int:
    all_errors: list[str] = []

    for split in ("train", "val"):
        images = iter_images(split)
        split_counts: Counter[int] = Counter()

        if not images:
            all_errors.append(f"no images found in {DATASET / 'images' / split}")

        for image_path in images:
            label_path = DATASET / "labels" / split / f"{image_path.stem}.txt"
            errors, counts = check_label(label_path)
            all_errors.extend(errors)
            split_counts.update(counts)

        print(f"\n[{split}] images: {len(images)}")
        for class_id, name in CLASSES.items():
            print(f"  {class_id} {name}: {split_counts[class_id]} boxes")

        label_dir = DATASET / "labels" / split
        if not label_dir.exists():
            all_errors.append(f"missing label directory: {label_dir}")
            continue
        label_stems = {p.stem for p in label_dir.glob("*.txt")}
        image_stems = {p.stem for p in images}
        for extra in sorted(label_stems - image_stems):
            all_errors.append(f"orphan label without image: {label_dir / (extra + '.txt')}")

    if all_errors:
        print("\nErrors:", file=sys.stderr)
        for error in all_errors[:80]:
            print(f"  - {error}", file=sys.stderr)
        if len(all_errors) > 80:
            print(f"  ... {len(all_errors) - 80} more", file=sys.stderr)
        return 1

    print("\nDataset check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
