# Model Exports

This directory contains compact trained model artifacts that are safe to pull from GitHub. Full training runs, datasets, caches, and intermediate conversion work directories are intentionally not stored here.

## YOLO11 Detection

```text
yolo11n_det_synthetic_rtx5060/
  best.pt      Ultralytics checkpoint
  best.onnx    ONNX export for RDK X5 conversion
  args.yaml    training arguments
  results.csv  training metrics
```

Source run on the Windows training machine:

```text
runs/detect/runs/micro_drone/yolo11n_det_synthetic_rtx5060/
```

Use this ONNX for RDK conversion:

```bash
cd ai_training
MODEL=model_exports/yolo11n_det_synthetic_rtx5060/best.onnx ./scripts/rdk/prepare_yolo11_det_convert.sh
```

## CIFAR-100 Classification

```text
yolo11n_cifar100_cls_full_rtx5060/
  best.pt      Ultralytics checkpoint
  best.onnx    ONNX export for RDK X5 conversion
  args.yaml    training arguments
  results.csv  training metrics
```

Source run on the Windows training machine:

```text
ai_training/runs/micro_drone/yolo11n_cifar100_cls_full_rtx5060/
```

The classification ONNX is also prepared for the existing classifier conversion path:

```bash
cd ai_training
cp model_exports/yolo11n_cifar100_cls_full_rtx5060/best.onnx rdk_convert/cifar100_cls/best.onnx
python scripts/rdk/prepare_cls_calibration.py --clean --count 300
```
