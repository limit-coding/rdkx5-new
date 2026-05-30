# YOLO11 Training Pipeline

This folder is the local training entrypoint for the micro drone vision model.

## Dataset URL

The competition PDF says picture targets are sampled from CIFAR-100:

- Official page: https://www.cs.toronto.edu/~kriz/cifar.html
- Direct archive: https://www.cs.toronto.edu/~kriz/cifar-100-python.tar.gz

CIFAR-100 is only the image content used inside printed picture targets. The YOLO detection dataset for `picture_target`, `special_target`, `ring`, `obstacle`, `landing_h`, `red_light`, and `blue_light` still needs local capture or synthetic generation, then annotation.

## Dataset Layout

Put images and YOLO txt labels here:

```text
datasets/micro_drone_det/
  images/train/
  images/val/
  labels/train/
  labels/val/
  micro_drone_det.yaml
```

Classes:

```text
0 picture_target
1 special_target
2 ring
3 obstacle
4 landing_h
5 red_light
6 blue_light
```

Each label line:

```text
class_id x_center y_center width height
```

All coordinates are normalized to `[0, 1]`.

## Setup

```bash
conda create -n drone-yolo11 python=3.11 -y
conda activate drone-yolo11
pip install -r requirements-yolo11.txt
yolo checks
```

## Train

```bash
bash scripts/yolo11/train_det.sh
```

Useful overrides:

```bash
BATCH=8 DEVICE=cpu bash scripts/yolo11/train_det.sh
MODEL=yolo11s.pt NAME=yolo11s_det BATCH=8 bash scripts/yolo11/train_det.sh
```

## Validate And Predict

```bash
bash scripts/yolo11/val_det.sh
SOURCE=videos/test_flight.mp4 bash scripts/yolo11/predict_det.sh
```

## Export ONNX

```bash
bash scripts/yolo11/export_onnx.sh
```

The expected model path after training is:

```text
runs/micro_drone/yolo11n_det/weights/best.onnx
```

Then convert the ONNX model to an RDK X5 BPU `.bin` model with RDK_ToolChain:

```text
https://github.com/xiongqi123123/RDK_ToolChain
```

Use 100 to 300 real competition-style images as calibration data. A helper list can be created with:

```bash
bash scripts/yolo11/make_calibration_list.sh
```

After conversion, copy the `.bin` into `camera/resource/` and update `camera/camera/animal_detect_enable.py`:

```python
coco_names = ["picture_target", "special_target", "ring", "obstacle", "landing_h", "red_light", "blue_light"]
model = YOLO11_Detect(model_path, conf_thres, iou_thres, 7)
```
