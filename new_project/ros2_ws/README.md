# ROS2 Workspace

Build:

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Important packages:

```text
src/lidar_fc_cpp    Radar, relative pose, flight-control bridge
src/camera          QR recognition and YOLO11 BPU inference
src/main            Mission logic, planner, controller
src/interfaces      Custom messages and services
src/communication   UART and Bluetooth
src/tf              TF publishing
src/launch          Runtime launch files
```

Useful commands:

```bash
ros2 launch launch real_pid_launch.py
ros2 launch lidar_fc_cpp auto_flight_cpp.launch.py
ros2 run camera qr_show
ros2 run camera animal_detect
```
