#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[2]


def resolve(path: str) -> str:
    candidate = Path(path)
    if candidate.is_absolute():
        return str(candidate)
    return str(ROOT / candidate)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run YOLO classification train/val/predict/export.")
    parser.add_argument("mode", choices=("train", "val", "predict", "export"))
    parser.add_argument("--model", default="yolo11n-cls.pt")
    parser.add_argument("--data", default="datasets/cifar100_target_cls")
    parser.add_argument("--source", default="datasets/cifar100_target_cls/val")
    parser.add_argument("--imgsz", type=int, default=224)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch", type=int, default=64)
    parser.add_argument("--device", default="mps")
    parser.add_argument("--project", default="runs/micro_drone")
    parser.add_argument("--name", default="yolo11n_cifar100_cls")
    parser.add_argument("--opset", type=int, default=12)
    args = parser.parse_args()

    model = YOLO(args.model)

    if args.mode == "train":
        model.train(
            data=resolve(args.data),
            imgsz=args.imgsz,
            epochs=args.epochs,
            batch=args.batch,
            device=args.device,
            project=resolve(args.project),
            name=args.name,
        )
    elif args.mode == "val":
        model.val(data=resolve(args.data), imgsz=args.imgsz, device=args.device)
    elif args.mode == "predict":
        model.predict(source=resolve(args.source), imgsz=args.imgsz, device=args.device, save=True)
    elif args.mode == "export":
        model.export(format="onnx", imgsz=args.imgsz, opset=args.opset, simplify=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
