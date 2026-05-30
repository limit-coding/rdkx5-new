#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

MODEL="${MODEL:-runs/micro_drone/yolo11n_det/weights/best.pt}"
SOURCE="${SOURCE:-0}"
IMGSZ="${IMGSZ:-640}"
CONF="${CONF:-0.3}"

yolo detect predict \
  model="$MODEL" \
  source="$SOURCE" \
  imgsz="$IMGSZ" \
  conf="$CONF" \
  save=True
