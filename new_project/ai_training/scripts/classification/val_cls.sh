#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"
PYTHON_BIN="${PYTHON:-python3}"

MODEL="${MODEL:-runs/micro_drone/yolo11n_cifar100_cls/weights/best.pt}"
DATA="${DATA:-datasets/cifar100_target_cls}"
IMGSZ="${IMGSZ:-224}"
DEVICE="${DEVICE:-mps}"

"$PYTHON_BIN" scripts/classification/run_cls.py val \
  --model "$MODEL" \
  --data "$DATA" \
  --imgsz "$IMGSZ" \
  --device "$DEVICE"
