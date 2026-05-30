#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

MODEL="${MODEL:-yolo11n.pt}"
DATA="${DATA:-datasets/micro_drone_det/micro_drone_det.yaml}"
IMGSZ="${IMGSZ:-640}"
EPOCHS="${EPOCHS:-120}"
BATCH="${BATCH:-16}"
DEVICE="${DEVICE:-0}"
PROJECT="${PROJECT:-runs/micro_drone}"
NAME="${NAME:-yolo11n_det}"

python scripts/yolo11/check_dataset.py

yolo detect train \
  model="$MODEL" \
  data="$DATA" \
  imgsz="$IMGSZ" \
  epochs="$EPOCHS" \
  batch="$BATCH" \
  device="$DEVICE" \
  project="$PROJECT" \
  name="$NAME"
