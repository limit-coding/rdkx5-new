# lidar_fc_cpp

C++ version of the Livox MID360 to flight-controller data link.

## Nodes

- `mid360_xy_cpp`: subscribes `/livox/lidar`, publishes `/mid360/xy_points`.
- `relative_pose_cpp`: subscribes `/Odometry`, publishes `/relative_pose`, `/position_error`, `/localization_valid`.
- `fc_bridge_cpp`: subscribes `/relative_pose` and `/localization_valid`, sends the XY+yaw frame to `/dev/ttyFC`.

The serial frame follows the ANO_LX_FC fixed-point positioning parser:

```text
AA FF 01 06 X_H X_L Y_H Y_L YAW_H YAW_L CHECKSUM
```

`X` and `Y` are signed int16 centimeters. `YAW` is signed int16 degrees. Multi-byte fields use high byte first to match the ANO_LX_FC `topoint()` parser. The default serial send frequency is `20Hz`.

## Build

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/lidar_ws/install/setup.bash
colcon build --packages-select lidar_fc_cpp
source install/setup.bash
```

## Run

```bash
ros2 launch lidar_fc_cpp auto_flight_cpp.launch.py
```

Check runtime frequencies:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /Odometry
ros2 topic hz /relative_pose
ros2 param get /fc_bridge_node send_freq
```

Key safety/tuning parameters:

```bash
ros2 param set /relative_pose_node max_relative_meters 10.0
ros2 param set /fc_bridge_node max_xy_meters 10.0
ros2 param set /fc_bridge_node clamp_xy_instead_of_zero true
```
