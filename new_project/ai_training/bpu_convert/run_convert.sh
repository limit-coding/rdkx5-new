#!/bin/bash
# 在RDK X5板子上运行BPU转换
# 用法：把整个bpu_convert目录scp到板子，然后执行此脚本
set -e
cd "$(dirname "$0")"

echo "=== 生成校准数据 ==="
python3 prep_calib.py

echo "=== 开始BPU转换 ==="
hb_mapper makertbin --config yolo11_cifar100_cls.yaml --model-type onnx

echo "=== 转换完成 ==="
ls -lh *.bin 2>/dev/null || ls -lh yolo11n_cifar100_cls*.bin 2>/dev/null || find . -name "*.bin"
