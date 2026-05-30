#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"
PYTHON_BIN="${PYTHON:-python3}"

if [ ! -d datasets/raw/cifar-100-python ]; then
  bash scripts/classification/download_cifar100.sh
fi

"$PYTHON_BIN" scripts/classification/prepare_cifar100_cls.py \
  --clean \
  --max-train-per-class "${MAX_TRAIN_PER_CLASS:-120}" \
  --max-val-per-class "${MAX_VAL_PER_CLASS:-30}" \
  --train-augmentations "${TRAIN_AUGMENTATIONS:-2}" \
  --val-augmentations "${VAL_AUGMENTATIONS:-1}" \
  --image-size "${IMAGE_SIZE:-224}"

"$PYTHON_BIN" scripts/classification/check_cls_dataset.py
