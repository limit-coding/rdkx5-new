# CIFAR-100 Target-Center Classification

This is phase 1 of the vision plan:

```text
OpenCV locates the picture target and crops the center
-> classifier recognizes the CIFAR-100 class
-> mission logic compares it with QR code targets
```

This does not require a full YOLO detection dataset.

## Prepare Data

```bash
cd new_project/ai_training
./scripts/classification/prepare_phase1.sh
```

Default output:

```text
datasets/cifar100_target_cls/
  train/<100 CIFAR classes>/*.jpg
  val/<100 CIFAR classes>/*.jpg
```

The preparation script enlarges CIFAR-100 images to `224x224` and adds camera-like augmentation: perspective jitter, rotation, brightness changes, blur, noise, JPEG artifacts, and slightly imperfect crops.

Smaller quick test:

```bash
MAX_TRAIN_PER_CLASS=20 MAX_VAL_PER_CLASS=5 TRAIN_AUGMENTATIONS=1 ./scripts/classification/prepare_phase1.sh
```

## Train

```bash
./scripts/classification/train_cls.sh
```

Useful overrides:

```bash
DEVICE=cpu BATCH=16 EPOCHS=5 ./scripts/classification/train_cls.sh
MODEL=yolo11s-cls.pt NAME=yolo11s_cifar100_cls ./scripts/classification/train_cls.sh
```

On Apple Silicon Macs, the default device is `mps`. If MPS is unstable on a machine, use `DEVICE=cpu`.

If the conda environment is not activated, point the scripts at the exact Python executable:

```bash
PYTHON=/opt/anaconda3/envs/ml/bin/python ./scripts/classification/train_cls.sh
```

If a wrapper script unexpectedly falls back to CPU, use the exact Ultralytics CLI command that was verified on this Mac:

```bash
/opt/anaconda3/envs/ml/bin/yolo classify train \
  model=yolo11n-cls.pt \
  data=datasets/cifar100_target_cls \
  imgsz=224 \
  epochs=20 \
  batch=32 \
  device=mps \
  project=runs/micro_drone \
  name=yolo11n_cifar100_cls_mps
```

## Validate / Predict / Export

```bash
./scripts/classification/val_cls.sh
./scripts/classification/predict_cls.sh
./scripts/classification/export_cls_onnx.sh
```

The first usable classifier should appear at:

```text
runs/micro_drone/yolo11n_cifar100_cls/weights/best.pt
```
