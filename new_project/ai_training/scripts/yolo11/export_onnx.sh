#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

MODEL="${MODEL:-runs/micro_drone/yolo11n_det/weights/best.pt}"
IMGSZ="${IMGSZ:-640}"
OPSET="${OPSET:-12}"

yolo export \
  model="$MODEL" \
  format=onnx \
  imgsz="$IMGSZ" \
  opset="$OPSET" \
  simplify=True
