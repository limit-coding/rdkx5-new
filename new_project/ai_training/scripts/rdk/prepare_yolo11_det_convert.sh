#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

DEFAULT_MODEL="model_exports/yolo11n_det_synthetic_rtx5060/best.onnx"
if [[ ! -f "$DEFAULT_MODEL" ]]; then
  DEFAULT_MODEL="runs/micro_drone/yolo11n_det/weights/best.onnx"
fi

MODEL="${MODEL:-$DEFAULT_MODEL}"
CALIB_COUNT="${CALIB_COUNT:-300}"
CONVERT_DIR="${CONVERT_DIR:-rdk_convert/yolo11_det}"
CALIB_DIR="$CONVERT_DIR/calibration_images"

if [[ ! -f "$MODEL" ]]; then
  echo "ONNX model not found: $MODEL" >&2
  echo "Set MODEL=/path/to/best.onnx or run scripts/yolo11/export_onnx.sh first." >&2
  exit 1
fi

mkdir -p "$CONVERT_DIR" "$CALIB_DIR"
cp "$MODEL" "$CONVERT_DIR/best.onnx"

rm -rf "$CALIB_DIR"
mkdir -p "$CALIB_DIR"

tmp_list="$(mktemp)"
trap 'rm -f "$tmp_list"' EXIT

if [[ -n "${CALIB_LIST:-}" ]]; then
  if [[ ! -f "$CALIB_LIST" ]]; then
    echo "Calibration list not found: $CALIB_LIST" >&2
    exit 1
  fi
  grep -Ev '^[[:space:]]*($|#)' "$CALIB_LIST" | head -n "$CALIB_COUNT" > "$tmp_list"
else
  CALIB_SOURCE="${CALIB_SOURCE:-datasets/micro_drone_det/images/train datasets/micro_drone_det/images/val ../rdk_deploy/camera_check}"
  read -r -a source_dirs <<< "$CALIB_SOURCE"
  existing_dirs=()
  for dir in "${source_dirs[@]}"; do
    if [[ -d "$dir" ]]; then
      existing_dirs+=("$dir")
    fi
  done

  if [[ "${#existing_dirs[@]}" -gt 0 ]]; then
    find "${existing_dirs[@]}" -type f \
      \( -iname '*.jpg' -o -iname '*.jpeg' -o -iname '*.png' -o -iname '*.bmp' -o -iname '*.webp' \) \
      | sort \
      | head -n "$CALIB_COUNT" > "$tmp_list"
  fi
fi

if [[ ! -s "$tmp_list" ]]; then
  echo "No calibration images found." >&2
  echo "Set CALIB_SOURCE='dir1 dir2' or CALIB_LIST=/path/to/images.txt." >&2
  exit 1
fi

i=0
while IFS= read -r img; do
  ext="${img##*.}"
  printf -v name "calib_%04d.%s" "$i" "$ext"
  cp "$img" "$CALIB_DIR/$name"
  i=$((i + 1))
done < "$tmp_list"

echo "Prepared YOLO11 RDK conversion package:"
echo "  model: $CONVERT_DIR/best.onnx"
echo "  calibration images: $CALIB_DIR ($i files)"
if [[ "$i" -lt 50 ]]; then
  echo "Warning: calibration image count is low. Use 100-300 real board-camera images for better quantization."
fi
echo "Next:"
echo "  ./scripts/rdk/run_yolo11_det_convert_docker.sh"
