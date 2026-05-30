#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

OUT_DIR="${OUT_DIR:-datasets/raw}"
URL="${URL:-https://www.cs.toronto.edu/~kriz/cifar-100-python.tar.gz}"

mkdir -p "$OUT_DIR"
curl -L "$URL" -o "$OUT_DIR/cifar-100-python.tar.gz"
tar -xzf "$OUT_DIR/cifar-100-python.tar.gz" -C "$OUT_DIR"

echo "CIFAR-100 downloaded to $OUT_DIR/cifar-100-python"
