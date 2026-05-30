"""
准备hb_mapper校准数据：从cifar100_target_cls取每类1张，共100张。
用法：python3 prep_calib.py
"""
import shutil, os
from pathlib import Path
from PIL import Image

src = Path("../datasets/cifar100_target_cls/train")
dst = Path("calibration_data")
dst.mkdir(exist_ok=True)

count = 0
for cls_dir in sorted(src.iterdir()):
    if not cls_dir.is_dir():
        continue
    imgs = list(cls_dir.glob("*.jpg")) + list(cls_dir.glob("*.png")) + list(cls_dir.glob("*.JPEG"))
    if not imgs:
        continue
    img = Image.open(imgs[0]).resize((224, 224)).convert("RGB")
    img.save(dst / f"{cls_dir.name}_0.jpg")
    count += 1

print(f"校准数据已生成: {count} 张 → {dst}")
