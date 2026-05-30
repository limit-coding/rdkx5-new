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
    amp=False,
)
