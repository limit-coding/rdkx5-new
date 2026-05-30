# Micro Drone Competition Project

This is the cleaned project folder for the micro drone competition.

## Layout

```text
new_project/
  ros2_ws/src/        ROS2 packages used on the RDK X5
  ai_training/        YOLO11/CIFAR training workspace
  rdk_deploy/         Board deployment helpers and camera debug notes
  docs/               Competition and project documents
  tools/              Small local/board check scripts
```

## ROS2 Runtime Packages

```text
ros2_ws/src/interfaces      Custom ROS2 msg/srv definitions
ros2_ws/src/lidar_fc_cpp    MID-360, relative pose, FC bridge, QR C++ node
ros2_ws/src/camera          QR recognition and YOLO11 BPU inference
ros2_ws/src/main            Mission state machine, controller, planner, map
ros2_ws/src/communication   UART and Bluetooth communication
ros2_ws/src/tf              TF publisher
ros2_ws/src/launch          Launch files
```

Build on the RDK X5:

```bash
cd new_project/ros2_ws
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Common launch entry:

```bash
ros2 launch launch real_pid_launch.py
```

Radar/flight-control C++ entry:

```bash
ros2 launch lidar_fc_cpp auto_flight_cpp.launch.py
```

QR test:

```bash
ros2 run camera qr_show
```

YOLO BPU inference node:

```bash
ros2 run camera animal_detect
```

## AI Training

The CIFAR-100 source used by the competition picture targets:

```text
https://www.cs.toronto.edu/~kriz/cifar.html
```

Phase 1 should start with CIFAR-100 target-center classification:

```bash
cd new_project/ai_training
conda create -n drone-yolo11 python=3.11 -y
conda activate drone-yolo11
pip install -r requirements-yolo11.txt
./scripts/classification/prepare_phase1.sh
./scripts/classification/train_cls.sh
```

YOLO detection data still needs competition-style capture/annotation. Put YOLO images and labels under:

```text
ai_training/datasets/micro_drone_det/
```

Trained checkpoints and exported ONNX files that are small enough for GitHub are stored in:

```text
ai_training/model_exports/
```

Train:

```bash
cd new_project/ai_training
python scripts/yolo11/check_dataset.py
./scripts/yolo11/train_det.sh
./scripts/yolo11/export_onnx.sh
```

After ONNX export, convert to RDK X5 `.bin`, copy it to:

```text
ros2_ws/src/camera/resource/
```

Mac handoff notes for continuing YOLO-to-RDK conversion are in:

```text
docs/Mac_RDK_X5_YOLO交接.md
```

Then update `ros2_ws/src/camera/camera/animal_detect_enable.py`:

```python
coco_names = ["picture_target", "special_target", "ring", "obstacle", "landing_h", "red_light", "blue_light"]
model = YOLO11_Detect(model_path, conf_thres, iou_thres, 7)
```

## Notes

The original repository is left untouched. This folder is a cleaned copy focused on the competition runtime, vision training, and board deployment path.
