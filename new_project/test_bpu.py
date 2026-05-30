"""
RDK X5 BPU模型验证脚本
用法：python3 test_bpu.py --model yolo11n_cifar100_cls.bin --imgs img1.jpg img2.jpg
"""
import argparse
import numpy as np
import cv2
import json
from pathlib import Path

CIFAR100_CLASSES = [
    "apple","aquarium_fish","baby","bear","beaver","bed","bee","beetle","bicycle","bottle",
    "bowl","boy","bridge","bus","butterfly","camel","can","castle","caterpillar","cattle",
    "chair","chimpanzee","clock","cloud","cockroach","couch","crab","crocodile","cup","dinosaur",
    "dolphin","elephant","flatfish","forest","fox","girl","hamster","house","kangaroo","keyboard",
    "lamp","lawn_mower","leopard","lion","lizard","lobster","man","maple_tree","motorcycle","mountain",
    "mouse","mushroom","oak_tree","orange","orchid","otter","palm_tree","pear","pickup_truck","pine_tree",
    "plain","plate","poppy","porcupine","possum","rabbit","raccoon","ray","road","rocket",
    "rose","sea","seal","shark","shrew","skunk","skyscraper","snail","snake","spider",
    "squirrel","streetcar","sunflower","sweet_pepper","table","tank","telephone","television","tiger","tractor",
    "train","trout","tulip","turtle","wardrobe","whale","willow_tree","wolf","woman","worm"
]


def preprocess(img_path: str) -> np.ndarray:
    """读图 → 中心裁剪1000px → resize 224 → RGB uint8 NCHW"""
    img = cv2.imread(img_path)
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2
    # 大图（原始靶板照片）裁中心
    if min(h, w) > 500:
        s = min(1000, min(h, w))
        img = img[cy-s//2:cy+s//2, cx-s//2:cx+s//2]
    img = cv2.resize(img, (224, 224))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # NCHW uint8
    arr = np.asarray(img, dtype=np.uint8).transpose(2, 0, 1)[np.newaxis]
    return arr


def run_bpu(model_path: str, imgs: list):
    try:
        from hobot_dnn import pyeasy_dnn as dnn
    except ImportError:
        print("错误: 找不到 hobot_dnn，请确认在RDK X5板子上运行")
        return

    print(f"加载BPU模型: {model_path}")
    models = dnn.load(model_path)
    model = models[0]
    print(f"模型加载成功，输入shape: {model.inputs[0].properties.shape}")

    for img_path in imgs:
        arr = preprocess(img_path)
        outputs = model.forward(arr)
        probs = np.array(outputs[0].buffer).flatten()
        top5_idx = np.argsort(probs)[::-1][:5]

        print(f"\n{'='*45}")
        print(f"图片: {Path(img_path).name}")
        for rank, idx in enumerate(top5_idx, 1):
            label = CIFAR100_CLASSES[idx] if idx < len(CIFAR100_CLASSES) else f"class_{idx}"
            print(f"  Top{rank}: {label:<25s} {probs[idx]:.1%}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="/home/sunrise/model/yolo11n_cifar100_cls.bin")
    parser.add_argument("--imgs", nargs="+", required=True)
    args = parser.parse_args()
    run_bpu(args.model, args.imgs)
