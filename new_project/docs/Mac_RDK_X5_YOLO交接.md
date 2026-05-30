# Mac 上继续做 RDK X5 YOLO 转换

这份说明用于把同学 Windows 电脑上的项目交接到自己的 Mac 上继续做。GitHub 只同步代码、文档和小配置；训练数据、`.pt`、`.onnx`、转换中间目录默认不放进 Git。

## 1. 拉代码

```bash
git clone https://github.com/limit-coding/BUPT--rdkx5.git
cd BUPT--rdkx5
```

如果你本机已经有仓库：

```bash
git pull origin main
```

## 2. 准备训练环境

```bash
cd ai_training
conda create -n drone-yolo11 python=3.11 -y
conda activate drone-yolo11
pip install -r requirements-yolo11.txt
yolo checks
```

已有环境的话，直接 `conda activate drone-yolo11` 即可。

## 3. 导出 YOLO11 ONNX

训练完成后默认模型路径是：

```text
ai_training/runs/micro_drone/yolo11n_det/weights/best.pt
```

仓库里已经上传了一份同学电脑上训练好的检测模型：

```text
ai_training/model_exports/yolo11n_det_synthetic_rtx5060/best.pt
ai_training/model_exports/yolo11n_det_synthetic_rtx5060/best.onnx
```

导出：

```bash
cd ai_training
MODEL=runs/micro_drone/yolo11n_det/weights/best.pt ./scripts/yolo11/export_onnx.sh
```

得到：

```text
ai_training/runs/micro_drone/yolo11n_det/weights/best.onnx
```

如果模型在别的位置，把 `MODEL=` 改成你的 `.pt` 路径。
如果直接用仓库里已有的 ONNX，可以跳过导出步骤。

## 4. 准备 RDK 转换包

校准图建议用 100 到 300 张真实比赛视角图片，越接近板端摄像头画面越好。

```bash
cd ai_training
MODEL=runs/micro_drone/yolo11n_det/weights/best.onnx \
CALIB_SOURCE="datasets/micro_drone_det/images/train datasets/micro_drone_det/images/val" \
CALIB_COUNT=300 \
./scripts/rdk/prepare_yolo11_det_convert.sh
```

如果使用仓库里已经上传的 ONNX：

```bash
cd ai_training
MODEL=model_exports/yolo11n_det_synthetic_rtx5060/best.onnx \
CALIB_SOURCE="datasets/micro_drone_det/images/train datasets/micro_drone_det/images/val" \
CALIB_COUNT=300 \
./scripts/rdk/prepare_yolo11_det_convert.sh
```

这会生成：

```text
ai_training/rdk_convert/yolo11_det/best.onnx
ai_training/rdk_convert/yolo11_det/calibration_images/
```

如果你已经有校准图片列表：

```bash
CALIB_LIST=datasets/calibration_images/calibration_images.txt \
MODEL=runs/micro_drone/yolo11n_det/weights/best.onnx \
./scripts/rdk/prepare_yolo11_det_convert.sh
```

## 5. Docker 转 `.bin`

如果你想用命令行工具链：

```bash
cd ai_training
./scripts/rdk/run_yolo11_det_convert_docker.sh
```

默认使用镜像：

```text
crpi-0uog49363mcubexr.cn-hangzhou.personal.cr.aliyuncs.com/skyxz/rdk_toolchain:v2.0
```

成功后会复制到：

```text
ros2_ws/src/camera/resource/yolo11_det.bin
```

如果你想用可视化工具，就打开 RDK_ToolChain，把 `rdk_convert/yolo11_det/` 里的 `best.onnx`、`calibration_images/` 和 `yolo11_det_bayese_640x640_nv12.yaml` 作为转换输入。

RDK_ToolChain 项目地址：

```text
https://github.com/xiongqi123123/RDK_ToolChain
```

## 6. 上板替换模型

板端检测代码：

```text
ros2_ws/src/camera/camera/animal_detect_enable.py
```

当前节点已经参数化，默认使用这次 7 类 YOLO11 检测模型：

```text
/home/sunrise/ros2/diansai/ws/src/camera/resource/yolo11_det.bin
```

构建运行：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run camera animal_enable
```

如果要覆盖参数：

```bash
ros2 run camera animal_enable --ros-args \
  -p model_path:=/home/sunrise/ros2/diansai/ws/src/camera/resource/yolo11_det.bin \
  -p class_names:=picture_target,special_target,ring,obstacle,landing_h,red_light,blue_light \
  -p conf:=0.3 \
  -p iou:=0.5
```

注意：README 里有些地方写的是 `animal_detect`，当前实际入口是 `animal_enable`。
