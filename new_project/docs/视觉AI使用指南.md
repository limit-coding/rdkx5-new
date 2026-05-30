# RDK X5 视觉 AI 部署

RDK X5 的 BPU 是整数推理单元，不能直接加速 YOLO 原生 `.pt` 浮点模型。`best.pt` 或普通 `best.onnx` 如果直接在 CPU 上跑，帧率通常会非常低；正确链路是先导出 ONNX，再用 RDK/OpenExplorer 工具链量化编译成 `.bin`，最后在 `camera` ROS2 包里用 `hobot_dnn` 调用 BPU。

本仓库已经接好这条链路，默认使用同学 RTX 5060 训练出的检测模型：

```text
ai_training/model_exports/yolo11n_det_synthetic_rtx5060/best.onnx
```

检测类别是 7 类：

```text
0 picture_target
1 special_target
2 ring
3 obstacle
4 landing_h
5 red_light
6 blue_light
```

## 1. 准备转换包

在电脑上执行：

```bash
cd ai_training
./scripts/rdk/prepare_yolo11_det_convert.sh
```

脚本会把 ONNX 和校准图片整理到：

```text
ai_training/rdk_convert/yolo11_det/
  best.onnx
  calibration_images/
  yolo11_det_bayese_640x640_nv12.yaml
```

量化校准最好放 100 到 300 张真实板端摄像头图片。没有数据时脚本会临时使用已有 `rdk_deploy/camera_check` 图片跑通流程，但最终比赛模型建议换成真实场地图片：

```bash
MODEL=model_exports/yolo11n_det_synthetic_rtx5060/best.onnx \
CALIB_SOURCE="/path/to/real_camera_images" \
CALIB_COUNT=300 \
./scripts/rdk/prepare_yolo11_det_convert.sh
```

## 2. 转换成 `.bin`

命令行转换：

```bash
cd ai_training
./scripts/rdk/run_yolo11_det_convert_docker.sh
```

成功后会复制到：

```text
ros2_ws/src/camera/resource/yolo11_det.bin
```

如果想用可视化工具，启动 RDK_ToolChain：

```bash
cd ai_training
./scripts/rdk/run_rdk_toolchain_gui.sh
```

浏览器打开：

```text
http://127.0.0.1:5000
```

RDK_ToolChain 项目地址：

```text
https://github.com/xiongqi123123/RDK_ToolChain
```

工具默认镜像：

```text
crpi-0uog49363mcubexr.cn-hangzhou.personal.cr.aliyuncs.com/skyxz/rdk_toolchain:v2.0
```

## 3. 上板运行

把生成的 `yolo11_det.bin` 放到板端工程：

```text
/home/sunrise/ros2/diansai/ws/src/camera/resource/yolo11_det.bin
```

构建并运行：

```bash
cd /home/sunrise/ros2/diansai/ws
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run camera animal_enable
```

节点默认订阅 `/pic_enable`，收到 `1` 开始推理，收到 `0` 暂停；检测计数发布到 `/pic_cnt`。

常用参数：

```bash
ros2 run camera animal_enable --ros-args \
  -p model_path:=/home/sunrise/ros2/diansai/ws/src/camera/resource/yolo11_det.bin \
  -p conf:=0.3 \
  -p iou:=0.5 \
  -p camera_index:=1 \
  -p show:=true
```

如果继续使用旧的 5 类动物模型，类别名必须一起改：

```bash
ros2 run camera animal_enable --ros-args \
  -p model_path:=/home/sunrise/ros2/diansai/ws/src/camera/resource/diansai0801.bin \
  -p class_names:=peacock,wolf,monkey,elephant,tiger
```
