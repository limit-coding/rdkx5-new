#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

OUT="${OUT:-datasets/calibration_images/calibration_images.txt}"

find datasets/micro_drone_det/images/train datasets/micro_drone_det/images/val \
  -type f \( -iname '*.jpg' -o -iname '*.jpeg' -o -iname '*.png' -o -iname '*.bmp' -o -iname '*.webp' \) \
  | sort \
  | head -300 > "$OUT"

echo "Wrote calibration image list: $OUT"
