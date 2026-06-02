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

Mac local QR -> YOLO test without ROS2/RDK dependencies:

```bash
python3 new_project/local_mission_vision.py --select-camera
```

To only list usable camera indices:

```bash
python3 new_project/local_mission_vision.py --list-cameras
```

The local script uses the selected camera first for QR recognition. After the
QR mission is stable for 3 frames, it switches the same camera stream into YOLO
recognition. If macOS blocks the camera, allow the terminal app running Python
in System Settings -> Privacy & Security -> Camera, then rerun the command.

RDK X5 camera stream to Mac over Ethernet:

```bash
# Do not change the RDK X5 eth0 static IP if it is bound to the MID360 radar.
# First check the RDK address without modifying it:
ip -br addr show eth0

# Mac side: add an address in the same subnet to the USB Ethernet adapter.
# Example if the RDK is 172.20.10.2/24:
sudo ifconfig en8 alias 172.20.10.1 netmask 255.255.255.0
ping 172.20.10.2

# Example if the RDK/radar subnet is 192.168.1.x/24:
# choose a free Mac address that does not duplicate the RDK or radar.
sudo ifconfig en8 alias 192.168.1.250 netmask 255.255.255.0
```

Start the RDK USB camera and MJPEG server:

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash

ros2 launch hobot_usb_cam hobot_usb_cam.launch.py \
  usb_video_device:=/dev/video0 \
  usb_image_width:=1280 \
  usb_image_height:=720 \
  usb_pixel_format:=mjpeg \
  usb_framerate:=30

# In another RDK terminal.
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
python3 /home/sunrise/project/rdk_deploy/mjpeg_view.py --port 8080 --fps 20 --quality 75
```

Then open `http://<RDK_IP>:8080` in the Mac browser, or run Mac-side QR ->
YOLO recognition from the RDK camera stream. For example, if the RDK is
`172.20.10.2`:

```bash
python3 new_project/local_mission_vision.py --source http://172.20.10.2:8080/stream
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
