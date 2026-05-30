# YOLO11 微型无人机视觉 AI 测试指南

> 分工说明：本指南给负责 YOLO11 的同学使用。目标是把 YOLO11n 在比赛视觉任务上训练、验证、导出，并产出可和 YOLO26 对比的测试结果。

## 1. 你的任务

你负责测试 YOLO11 路线，重点回答 4 个问题：

1. YOLO11n 能不能稳定识别比赛道具。
2. 本地训练效果怎么样。
3. 模型导出后是否适合放到 RDK X5 上部署。
4. 和 YOLO26 对比时，YOLO11 的速度、精度、稳定性如何。

第一版先做目标检测，不要一开始就把分类、追踪、分割都加进去。

## 2. 推荐模型

优先使用：

```text
yolo11n.pt
```

原因：

- `n` 是 nano 版本，速度最快，适合无人机机载计算。
- 当前项目已有 YOLO11 风格的 RDK X5 BPU 推理代码，后续迁移成本最低。
- 比赛任务更看重实时稳定，不是单纯追求最高精度。

如果 `yolo11n.pt` 精度明显不够，再补测：

```text
yolo11s.pt
```

但第一轮必须先测 `yolo11n.pt`。

## 3. 要识别的类别

第一版检测类别建议如下：

```text
0: picture_target
1: special_target
2: ring
3: obstacle
4: landing_h
5: red_light
6: blue_light
```

说明：

- `picture_target`：普通图片靶，先只检测靶的位置。
- `special_target`：带信号灯的特殊靶。
- `ring`：圆环。
- `obstacle`：绕飞障碍物。
- `landing_h`：起飞/降落点 H 标志。
- `red_light` / `blue_light`：如果灯太小、检测不稳定，可以后面改成颜色阈值判断。

CIFAR-100 图片类别识别先不强行放进这个检测模型。后续可以走“检测图片靶 -> 裁剪靶心 -> 分类模型”的路线。

## 4. 数据集目录

请按下面结构放数据：

```text
datasets/micro_drone_yolo11/
  images/
    train/
    val/
  labels/
    train/
    val/
  micro_drone_yolo11.yaml
```

`micro_drone_yolo11.yaml` 内容：

```yaml
path: datasets/micro_drone_yolo11
train: images/train
val: images/val
names:
  0: picture_target
  1: special_target
  2: ring
  3: obstacle
  4: landing_h
  5: red_light
  6: blue_light
```

YOLO 标签格式：

```text
class_id x_center y_center width height
```

注意：后面 4 个数字都是 0 到 1 的归一化坐标。

## 5. 数据采集要求

每类至少先准备：

| 类别 | 第一轮建议数量 |
|------|----------------|
| picture_target | 300 张以上 |
| special_target | 300 张以上 |
| ring | 300 张以上 |
| obstacle | 200 张以上 |
| landing_h | 200 张以上 |
| red_light / blue_light | 各 200 张以上 |

拍摄时尽量模拟真实比赛：

- 无人机视角，不要只用人手平视拍。
- 包含不同距离、角度、光照、模糊。
- 包含目标被部分遮挡的情况。
- 训练集和验证集不要用同一段视频连续帧硬切，否则验证结果会虚高。

## 6. 安装环境

推荐用有 NVIDIA GPU 的电脑训练。

```bash
conda create -n drone-yolo11 python=3.11 -y
conda activate drone-yolo11
pip install -U ultralytics opencv-python numpy matplotlib
```

检查环境：

```bash
yolo checks
python -c "import torch; print(torch.cuda.is_available())"
```

如果最后输出 `True`，说明 GPU 可用。

## 7. 训练 YOLO11n

在项目根目录或数据集所在目录执行：

```bash
yolo detect train \
  model=yolo11n.pt \
  data=datasets/micro_drone_yolo11/micro_drone_yolo11.yaml \
  imgsz=640 \
  epochs=120 \
  batch=16 \
  device=0 \
  project=runs/micro_drone_compare \
  name=yolo11n_det
```

如果显存不够，把 `batch=16` 改成：

```text
batch=8
```

训练结果在：

```text
runs/micro_drone_compare/yolo11n_det/
```

最重要的模型文件：

```text
runs/micro_drone_compare/yolo11n_det/weights/best.pt
```

## 8. 验证模型

训练完成后跑验证：

```bash
yolo detect val \
  model=runs/micro_drone_compare/yolo11n_det/weights/best.pt \
  data=datasets/micro_drone_yolo11/micro_drone_yolo11.yaml \
  imgsz=640 \
  device=0
```

需要记录：

```text
mAP50
mAP50-95
precision
recall
每个类别的 AP
```

不要只看总 mAP。比赛里 `ring`、`landing_h`、`picture_target` 这三个类别更关键。

## 9. 本地视频测试

用摄像头实时测试：

```bash
yolo detect predict \
  model=runs/micro_drone_compare/yolo11n_det/weights/best.pt \
  source=0 \
  imgsz=640 \
  conf=0.3
```

用录制视频测试：

```bash
yolo detect predict \
  model=runs/micro_drone_compare/yolo11n_det/weights/best.pt \
  source=videos/test_flight.mp4 \
  imgsz=640 \
  conf=0.3 \
  save=True
```

测试时重点观察：

- 圆环远距离能不能提前识别。
- 降落点 H 是否会误检。
- 图片靶倾斜时是否还能识别。
- 红蓝灯是否太小导致漏检。
- 无人机运动模糊时框是否乱跳。

## 10. 导出 ONNX

训练完成后导出：

```bash
yolo export \
  model=runs/micro_drone_compare/yolo11n_det/weights/best.pt \
  format=onnx \
  imgsz=640 \
  opset=12 \
  simplify=True
```

导出后应得到：

```text
runs/micro_drone_compare/yolo11n_det/weights/best.onnx
```

这个文件后续交给负责 RDK X5 部署的同学转换成 `.bin`。

## 11. 可选：测试 YOLO11s

如果 YOLO11n 漏检严重，再测 YOLO11s：

```bash
yolo detect train \
  model=yolo11s.pt \
  data=datasets/micro_drone_yolo11/micro_drone_yolo11.yaml \
  imgsz=640 \
  epochs=120 \
  batch=8 \
  device=0 \
  project=runs/micro_drone_compare \
  name=yolo11s_det
```

注意：YOLO11s 可能精度更好，但速度会慢。无人机上优先考虑稳定实时。

## 12. 最终交付物

请整理以下内容：

```text
1. 数据集说明
2. 训练命令
3. best.pt
4. best.onnx
5. val 指标截图或 results.csv
6. 预测效果视频或图片
7. 误检/漏检案例
8. 结论：YOLO11n 是否够用，是否需要 YOLO11s
```

结论模板：

```text
YOLO11n 测试结论：
- 训练数据量：
- mAP50：
- mAP50-95：
- 本地摄像头是否实时：
- 最容易漏检的类别：
- 最容易误检的类别：
- 是否建议上板：
- 后续建议：
```

## 13. 和 YOLO26 对比时统一这些参数

为了公平比较，请和 YOLO26 测试保持一致：

```text
imgsz=640
epochs=120
conf=0.3
同一份 train/val 数据集
同一批测试视频
同样记录 mAP、FPS、误检、漏检
```

最后我们比较：

| 模型 | mAP50 | mAP50-95 | 本地 FPS | 上板 FPS | 漏检情况 | 误检情况 | 是否推荐 |
|------|-------|----------|----------|----------|----------|----------|----------|
| YOLO11n | | | | | | | |
| YOLO11s | | | | | | | |
| YOLO26n | | | | | | | |

## 14. 参考资料

- Ultralytics 官方文档：https://docs.ultralytics.com/
- YOLO 训练文档：https://docs.ultralytics.com/modes/train/
- YOLO 检测任务：https://docs.ultralytics.com/tasks/detect/

