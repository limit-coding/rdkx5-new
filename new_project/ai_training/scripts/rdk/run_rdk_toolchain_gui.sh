#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

IMAGE="${RDK_TOOLCHAIN_IMAGE:-crpi-0uog49363mcubexr.cn-hangzhou.personal.cr.aliyuncs.com/skyxz/rdk_toolchain:v2.0}"
DATASET_PATH="${RDK_TOOLCHAIN_DATASET_PATH:-$(pwd)/rdk_convert}"
CONTAINER_NAME="${RDK_TOOLCHAIN_CONTAINER_NAME:-rdk-toolchain}"
SHM_SIZE="${RDK_TOOLCHAIN_SHM_SIZE:-16g}"
WEB_PORT="${RDK_TOOLCHAIN_WEB_PORT:-5000}"
STREAM_PORT="${RDK_TOOLCHAIN_STREAM_PORT:-8080}"

mkdir -p "$DATASET_PATH"

docker pull "$IMAGE"

gpu_args=()
if [[ "${RDK_TOOLCHAIN_GPUS:-}" != "" ]]; then
  gpu_args=(--gpus "${RDK_TOOLCHAIN_GPUS}")
fi

docker run --rm -it \
  --name "$CONTAINER_NAME" \
  "${gpu_args[@]}" \
  --shm-size="$SHM_SIZE" \
  --ipc=host \
  -e PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128 \
  -e CUDA_LAUNCH_BLOCKING=1 \
  -p "${WEB_PORT}:5000" \
  -p "${STREAM_PORT}:8080" \
  -v "${DATASET_PATH}:/data" \
  "$IMAGE"
