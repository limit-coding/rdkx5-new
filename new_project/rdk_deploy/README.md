# RDK X5 Deployment Notes

This folder keeps board-side helper files and camera/radar debug notes.

```text
patch_files/     systemd/start scripts and board-side radar config snapshots
camera_check/    USB/MIPI camera inspection logs and notes
```

Main deployment path:

```text
ai_training best.pt
-> export ONNX
-> convert ONNX to RDK X5 .bin
-> copy .bin to ros2_ws/src/camera/resource/
-> update ros2_ws/src/camera/camera/animal_detect_enable.py
-> colcon build on board
```

Current YOLO11 detector deployment is parameterized in:

```text
ros2_ws/src/camera/camera/animal_detect_enable.py
```

Default model path on the board:

```text
/home/sunrise/ros2/diansai/ws/src/camera/resource/yolo11_det.bin
```

Run:

```bash
ros2 run camera animal_enable
```
