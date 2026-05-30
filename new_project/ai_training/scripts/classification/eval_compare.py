#!/usr/bin/env python3
"""
对比方案A和方案B的模型，在真实靶标图上跑预测。
用法：python3 scripts/classification/eval_compare.py --imgs path/to/img1.jpg path/to/img2.jpg
"""
from __future__ import annotations
import argparse
from pathlib import Path
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[2]


def predict_one(model: YOLO, img_path: str) -> list[tuple[str, float]]:
    r = model(img_path, verbose=False)[0]
    return [(r.names[i], float(r.probs.data[i])) for i in r.probs.top5]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--imgs", nargs="+", required=True, help="测试图片路径")
    parser.add_argument("--model-a", type=str,
                        default=str(ROOT / "runs/compare/A_normal/weights/best.pt"))
    parser.add_argument("--model-b", type=str,
                        default=str(ROOT / "runs/compare/B_board_aug/weights/best.pt"))
    args = parser.parse_args()

    print("加载模型...")
    model_a = YOLO(args.model_a)
    model_b = YOLO(args.model_b)

    for img in args.imgs:
        print(f"\n{'='*50}")
        print(f"图片: {img}")
        print(f"{'─'*50}")

        top5_a = predict_one(model_a, img)
        top5_b = predict_one(model_b, img)

        print(f"{'方案A(原始)':30s}  {'方案B(靶板增强)':30s}")
        print(f"{'─'*30}  {'─'*30}")
        for (cls_a, conf_a), (cls_b, conf_b) in zip(top5_a, top5_b):
            print(f"{cls_a:25s} {conf_a:5.1%}  {cls_b:25s} {conf_b:5.1%}")


if __name__ == "__main__":
    main()
