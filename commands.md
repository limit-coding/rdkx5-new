# 调试命令

## 上传文件到板子（Mac 本地终端执行，不是 ssh）

```bash
scp /Users/limit/Desktop/2025-RDKx5-CODING-main/new_project/rdk_deploy/servo_drop.py sunrise@172.20.10.2:/home/sunrise/project/
```

```bash
scp /Users/limit/Desktop/2025-RDKx5-CODING-main/new_project/rdk_deploy/patch_files/start_all.sh sunrise@172.20.10.2:/home/sunrise/project/
```

```bash
scp /Users/limit/Desktop/2025-RDKx5-CODING-main/new_project/rdk_deploy/mjpeg_view.py sunrise@172.20.10.2:/home/sunrise/project/
```

---

## 板子上：source 环境

```bash
source /opt/ros/humble/setup.bash && source /home/sunrise/project/install/setup.bash
```

---

## 启动 mission_vision（调试用，直接看终端输出）

```bash
ros2 run camera mission_vision --ros-args -p image_topic:=/image -p cls_model_path:=/home/sunrise/project/camera/resource/cifar100_cls.onnx -p cls_names_path:=/home/sunrise/project/camera/resource/cifar100_names.txt
```

---

## 监听 QR 识别结果（另一个终端）

```bash
source /opt/ros/humble/setup.bash && ros2 topic echo /qr_task
```

---

## 检查摄像头是否有数据

```bash
source /opt/ros/humble/setup.bash && ros2 topic hz /image
```

没有数据时手动启动摄像头：

```bash
ros2 run hobot_usb_cam hobot_usb_cam --ros-args -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=480 &
```

---

## 摄像头浏览器预览（板子上运行后浏览器打开）

```bash
source /opt/ros/humble/setup.bash && source /home/sunrise/project/install/setup.bash && python3 /home/sunrise/project/mjpeg_view.py
```

浏览器打开：http://172.20.10.2:8080

---

## 查看日志

```bash
tail -f /tmp/flight_logs/mission_vision.log
```

```bash
tail -f /tmp/flight_logs/fc_bridge.log
```

```bash
tail -f /tmp/flight_logs/servo_drop.log
```
