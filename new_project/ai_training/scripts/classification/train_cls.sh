#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"
PYTHON_BIN="${PYTHON:-python3}"

MODEL="${MODEL:-yolo11n-cls.pt}"
DATA="${DATA:-datasets/cifar100_target_cls}"
IMGSZ="${IMGSZ:-224}"
EPOCHS="${EPOCHS:-80}"
BATCH="${BATCH:-64}"
DEVICE="${DEVICE:-mps}"
PROJECT="${PROJECT:-runs/micro_drone}"
NAME="${NAME:-yolo11n_cifar100_cls}"

"$PYTHON_BIN" scripts/classification/check_cls_dataset.py

"$PYTHON_BIN" scripts/classification/run_cls.py train \
  --model "$MODEL" \
  --data "$DATA" \
  --imgsz "$IMGSZ" \
  --epochs "$EPOCHS" \
  --batch "$BATCH" \
  --device "$DEVICE" \
  --project "$PROJECT" \
  --name "$NAME"
