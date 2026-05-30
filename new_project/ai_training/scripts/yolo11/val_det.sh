#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

MODEL="${MODEL:-runs/micro_drone/yolo11n_det/weights/best.pt}"
DATA="${DATA:-datasets/micro_drone_det/micro_drone_det.yaml}"
IMGSZ="${IMGSZ:-640}"
DEVICE="${DEVICE:-0}"

yolo detect val \
  model="$MODEL" \
  data="$DATA" \
  imgsz="$IMGSZ" \
  device="$DEVICE"
