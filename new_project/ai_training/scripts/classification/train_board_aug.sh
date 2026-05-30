#!/bin/bash
# 方案B：靶板合成增强数据训练
# 用法：bash scripts/classification/train_board_aug.sh

set -e
cd "$(dirname "$0")/../.."

echo "=== 方案B：靶板合成增强训练 ==="

# 1. 生成合成靶板数据集
if [ ! -d "datasets/cifar100_board_aug/train" ]; then
    echo "生成靶板合成数据集（约需 5~10 分钟）..."
    python3 scripts/classification/prepare_board_aug.py \
        --max-train-per-class 150 \
        --train-augs 3 \
        --max-val-per-class 30 \
        --val-augs 1 \
        --image-size 224
fi

# 2. 训练
python3 - <<'EOF'
from ultralytics import YOLO
model = YOLO("yolo11n-cls.pt")
model.train(
    data="datasets/cifar100_board_aug",
    epochs=80,
    imgsz=224,
    batch=256,
    device=0,
    workers=8,
    project="runs/compare",
    name="B_board_aug",
    pretrained=True,
    optimizer="AdamW",
    lr0=5e-4,
    lrf=0.01,
    warmup_epochs=3,
    dropout=0.2,
    # 靶板数据本身已经有大量几何增强，这里只加颜色扰动
    hsv_h=0.02,
    hsv_s=0.4,
    hsv_v=0.5,
    fliplr=0.5,
    flipud=0.1,
    degrees=0.0,   # 靶板脚本已做旋转，这里不重复
    translate=0.05,
    scale=0.2,
    erasing=0.2,
)
EOF

echo "=== 方案B 训练完成，结果在 runs/compare/B_board_aug ==="
