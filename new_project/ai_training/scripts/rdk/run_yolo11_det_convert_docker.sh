#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

IMAGE="${RDK_TOOLCHAIN_IMAGE:-crpi-0uog49363mcubexr.cn-hangzhou.personal.cr.aliyuncs.com/skyxz/rdk_toolchain:v2.0}"
CONVERT_ROOT="$(pwd)/rdk_convert"
CONVERT_DIR="rdk_convert/yolo11_det"
DEST_BIN="${DEST_BIN:-../ros2_ws/src/camera/resource/yolo11_det.bin}"

if [[ ! -f "$CONVERT_DIR/best.onnx" ]]; then
  echo "Missing $CONVERT_DIR/best.onnx" >&2
  echo "Run scripts/rdk/prepare_yolo11_det_convert.sh first." >&2
  exit 1
fi

if [[ ! -d "$CONVERT_DIR/calibration_images" ]]; then
  echo "Missing $CONVERT_DIR/calibration_images" >&2
  echo "Run scripts/rdk/prepare_yolo11_det_convert.sh first." >&2
  exit 1
fi

docker pull "$IMAGE"
docker run --rm -i \
  --shm-size=8g \
  -v "${CONVERT_ROOT}:/data" \
  "$IMAGE" \
  bash -lc "cd /data/yolo11_det && hb_mapper checker --model-type onnx --march bayes-e --model best.onnx && hb_mapper makertbin --model-type onnx --config yolo11_det_bayese_640x640_nv12.yaml"

bin_path="$(find "$CONVERT_DIR/work" -type f -name '*.bin' -exec ls -t {} + | head -n 1)"
if [[ -z "$bin_path" ]]; then
  echo "No .bin file was produced under $CONVERT_DIR/work" >&2
  exit 1
fi

mkdir -p "$(dirname "$DEST_BIN")"
cp "$bin_path" "$DEST_BIN"
echo "Copied BPU model to $DEST_BIN"
