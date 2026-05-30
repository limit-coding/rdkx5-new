import subprocess
import sys

print("=== 生成靶板合成数据集 ===")
subprocess.run([
    sys.executable, "scripts/classification/prepare_board_aug.py",
    "--max-train-per-class", "150",
    "--train-augs", "3",
    "--max-val-per-class", "30",
    "--image-size", "224",
], check=True)

print("=== 开始训练方案B ===")
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
    hsv_h=0.02,
    hsv_s=0.4,
    hsv_v=0.5,
    fliplr=0.5,
    flipud=0.1,
    erasing=0.2,
    amp=False,
)
