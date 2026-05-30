#!/bin/bash
# 方案A：原始 CIFAR-100 数据训练（无靶板合成）
# 用法：bash scripts/classification/train_normal.sh

set -e
cd "$(dirname "$0")/../.."

echo "=== 方案A：原始数据训练 ==="

# 1. 如果数据集不存在，先生成
if [ ! -d "datasets/cifar100_target_cls/train" ]; then
    echo "生成原始增强数据集..."
    python3 scripts/classification/prepare_cifar100_cls.py \
        --max-train-per-class 150 \
        --train-augmentations 3 \
        --max-val-per-class 30 \
        --image-size 224
fi

# 2. 训练
python3 - <<'EOF'
from ultralytics import YOLO
model = YOLO("yolo11n-cls.pt")
model.train(
    data="datasets/cifar100_target_cls",
    epochs=80,
    imgsz=224,
    batch=256,
    device=0,
    workers=8,
    project="runs/compare",
    name="A_normal",
    pretrained=True,
    optimizer="AdamW",
    lr0=5e-4,
    lrf=0.01,
    warmup_epochs=3,
    dropout=0.2,
    hsv_h=0.015,
    hsv_s=0.5,
    hsv_v=0.4,
    fliplr=0.5,
    flipud=0.1,
    degrees=15,
    translate=0.1,
    scale=0.4,
    erasing=0.3,
)
EOF

echo "=== 方案A 训练完成，结果在 runs/compare/A_normal ==="
