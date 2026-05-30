# lidar_fc_cpp

C++ version of the Livox MID360 to flight-controller data link.

## 中文阅读顺序

这套雷达飞控链路按下面顺序看最容易：

1. `launch/auto_flight_cpp.launch.py`
   - 这是总启动文件，决定 Livox 驱动、FAST-LIO、本项目 C++ 节点的启动顺序。
   - 先看它，可以知道每个节点什么时候启动、订阅和发布什么。

2. `src/mid360_xy_node.cpp`
   - 只做点云格式转换：`/livox/lidar` -> `/mid360/xy_points`。
   - 它不做定位、不做避障、不发飞控，只是把 Livox 自定义点云翻译成标准 `PointCloud2`。

3. `src/relative_pose_node.cpp`
   - 这是最关键的定位保护节点：`/Odometry` -> `/relative_pose`。
   - 它把第一次正常里程计当作原点，并默认把开机机头方向当作相对坐标系 `+X`。
   - 它还发布 `/localization_valid`，定位异常、跳变、断流、超范围时会变成 `false`。

4. `src/fc_bridge_node.cpp`
   - 这是飞控串口桥接：`/relative_pose` + `/localization_valid` -> `/dev/ttyFC`。
   - 定位有效时发送真实 `X/Y/YAW`；定位无效时发送 `0cm` 心跳帧，避免坏坐标进入飞控。

一句话数据流：

```text
MID360 雷达
  -> /livox/lidar
  -> mid360_xy_node
  -> /mid360/xy_points
  -> FAST-LIO
  -> /Odometry
  -> relative_pose_node
  -> /relative_pose + /localization_valid
  -> fc_bridge_node
  -> 串口 /dev/ttyFC
  -> 飞控 PID
```

如果要现场改方向，优先改 `relative_pose_node` 的参数：

```bash
ros2 param set /relative_pose_node swap_xy true
ros2 param set /relative_pose_node invert_x true
ros2 param set /relative_pose_node invert_y true
```

如果要改串口保护范围，优先改 `fc_bridge_node` 的参数：

```bash
ros2 param set /fc_bridge_node max_xy_meters 8.0
ros2 param set /fc_bridge_node clamp_xy_instead_of_zero true
```

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
