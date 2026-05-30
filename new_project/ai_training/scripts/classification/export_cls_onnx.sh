#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"
PYTHON_BIN="${PYTHON:-python3}"

MODEL="${MODEL:-runs/micro_drone/yolo11n_cifar100_cls/weights/best.pt}"
IMGSZ="${IMGSZ:-224}"
OPSET="${OPSET:-12}"

"$PYTHON_BIN" scripts/classification/run_cls.py export \
  --model "$MODEL" \
  --imgsz "$IMGSZ" \
  --opset "$OPSET"
