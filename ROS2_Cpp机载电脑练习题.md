# ROS2 C++ 机载电脑练习题

> 目标：让你在 RDK X5/机载电脑上，从会运行节点，逐步练到能写雷达安全节点、状态机节点和飞控保护逻辑。每道题都尽量能在真实项目里复用。

## 练习前准备

进入工作空间：

```bash
cd /home/sunrise/project
```

如果你的实际路径不同，以你机载电脑上的项目路径为准。

加载环境：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source install/setup.bash
```

常用检查命令：

```bash
ros2 topic list
ros2 node list
ros2 topic hz /Odometry
ros2 topic echo /relative_pose
ros2 topic echo /localization_valid
```

每道练习都要记录：

```text
你运行了什么命令
看到什么输出
哪里不符合预期
你怎么改的
```

## 第 1 组：ROS2 基础观察

### 练习 1：确认已有节点和话题

任务：

1. 启动现有 C++ 链路。
2. 查看节点列表。
3. 查看话题列表。
4. 找出 `/Odometry`、`/relative_pose`、`/localization_valid` 是否存在。

命令：

```bash
ros2 launch lidar_fc_cpp auto_flight_cpp.launch.py
```

另开终端：

```bash
ros2 node list
ros2 topic list
ros2 topic hz /Odometry
ros2 topic hz /relative_pose
ros2 topic echo /localization_valid
```

验收：

```text
能说清楚每个节点叫什么
能说清楚 /Odometry 和 /relative_pose 的频率
能看到 /localization_valid 什么时候 true/false
```

### 练习 2：观察相对坐标变化

任务：

1. 保持雷达/无人机静止，观察 `/relative_pose`。
2. 轻微移动设备或改变位置，观察 x/y 变化。
3. 记录哪个方向对应 x 增大，哪个方向对应 y 增大。

命令：

```bash
ros2 topic echo /relative_pose
```

记录表：

```text
向前移动：
x 变化：
y 变化：

向左移动：
x 变化：
y 变化：
```

验收：

```text
能确认当前 relative_pose 的 x/y 方向
能判断坐标轴是否和飞控期望一致
```

## 第 2 组：C++ 节点入门

### 练习 3：写一个心跳发布节点

任务：

新增：

```text
lidar_fc_cpp/src/heartbeat_node.cpp
```

功能：

```text
每 1 秒发布一次 /system_heartbeat
类型 std_msgs/msg/Bool
内容 true
```

验收命令：

```bash
colcon build --packages-select lidar_fc_cpp
source install/setup.bash
ros2 run lidar_fc_cpp heartbeat_cpp
ros2 topic echo /system_heartbeat
```

验收：

```text
/system_heartbeat 每秒输出 true
```

提示：

需要修改 `lidar_fc_cpp/CMakeLists.txt`：

```cmake
add_executable(heartbeat_cpp src/heartbeat_node.cpp)
ament_target_dependencies(heartbeat_cpp ${COMMON_DEPENDENCIES})
```

并加入 `install(TARGETS ...)`。

### 练习 4：写一个参数节点

任务：

修改上一个心跳节点，加入参数：

```text
publish_freq
```

默认 `1.0Hz`，可以命令行改成 `5.0Hz`。

运行：

```bash
ros2 run lidar_fc_cpp heartbeat_cpp --ros-args -p publish_freq:=5.0
ros2 topic hz /system_heartbeat
```

验收：

```text
默认约 1Hz
参数改后约 5Hz
```

## 第 3 组：定位有效性练习

### 练习 5：订阅 `/localization_valid`

任务：

写一个节点：

```text
localization_watch_node.cpp
```

功能：

```text
订阅 /localization_valid
valid=true 时每秒打印 “定位有效”
valid=false 时每秒打印 “定位无效”
```

验收：

```bash
ros2 run lidar_fc_cpp localization_watch_cpp
```

你要能看到当前定位状态。

### 练习 6：增加超时判断

任务：

在 `localization_watch_node.cpp` 中加入：

```text
如果 1 秒内没有收到 /localization_valid，就打印 “定位状态超时”
```

验收：

```text
停止 relative_pose 节点后，1 秒左右能看到超时提示
```

重点：

学会使用：

```cpp
std::optional<std::chrono::steady_clock::time_point>
std::chrono::steady_clock::now()
```

## 第 4 组：雷达安全节点

### 练习 7：统计雷达点数

任务：

写：

```text
lidar_safety_node.cpp
```

第一版只做：

```text
订阅 /livox/lidar
每秒打印点数
发布 /lidar_valid
```

验收：

```bash
ros2 run lidar_fc_cpp lidar_safety_cpp
ros2 topic echo /lidar_valid
```

要求：

```text
收到雷达数据 -> /lidar_valid true
超过 1 秒没有雷达数据 -> /lidar_valid false
```

### 练习 8：计算前方最近距离

任务：

在 `lidar_safety_node.cpp` 中计算前方区域最近距离。

先假设：

```text
x 是前方
y 是左右
z 是上下
```

区域：

```text
0.2 < x < 2.0
-0.5 < y < 0.5
-0.5 < z < 0.5
```

发布：

```text
/front_distance
类型 std_msgs/msg/Float32
```

验收：

```bash
ros2 topic echo /front_distance
```

拿纸箱或障碍物在雷达前方移动，距离应该平滑变化。

### 练习 9：加入滑动窗口滤波

任务：

给 `/front_distance` 加滑动窗口。

要求：

```text
保存最近 5 帧距离
发布中值或平均值
无有效点时不更新距离
```

验收：

```text
障碍物静止时，距离输出不明显乱跳
偶发跳点不会直接影响输出
```

### 练习 10：发布危险状态

任务：

发布：

```text
/obstacle_danger
类型 std_msgs/msg/Bool
```

逻辑：

```text
连续 3 帧 front_distance < 0.8m -> danger=true
连续 5 帧 front_distance > 1.0m -> danger=false
雷达超时 -> danger=true
```

验收：

```bash
ros2 topic echo /obstacle_danger
```

测试：

```text
障碍物靠近到 0.8m 内，danger 变 true
障碍物离开到 1.0m 外，danger 变 false
拔掉/停止雷达数据，danger 变 true
```

## 第 5 组：状态机练习

### 练习 11：写最小状态机

任务：

写：

```text
task_manager_node.cpp
```

状态：

```text
WAIT_START
SAFE_HOVER
FAILSAFE
```

订阅：

```text
/obstacle_danger
/localization_valid
```

逻辑：

```text
启动后进入 SAFE_HOVER
如果 obstacle_danger=true -> FAILSAFE
如果 localization_valid=false 超过 1 秒 -> FAILSAFE
```

发布：

```text
/task_state
类型 std_msgs/msg/String
```

验收：

```bash
ros2 topic echo /task_state
```

### 练习 12：加入二维码状态

任务：

扩展状态：

```text
SCAN_QR
QR_CONFIRMED
```

订阅：

```text
/qr_code/text
```

逻辑：

```text
连续 3 次收到相同且非空二维码文本 -> QR_CONFIRMED
10 秒没确认 -> FAILSAFE 或保持 SCAN_QR
```

测试可以先手动发布：

```bash
ros2 topic pub /qr_code/text std_msgs/msg/String "{data: 'man,apple,left'}"
```

验收：

```text
连续发布 3 次相同二维码后，/task_state 进入 QR_CONFIRMED
```

### 练习 13：解析二维码文本

任务：

解析：

```text
man,apple,left
```

得到：

```text
target_class_1 = man
target_class_2 = apple
landing_side = left
```

发布日志：

```text
QR confirmed: target1=man, target2=apple, landing=left
```

验收：

```text
错误格式不会让状态机确认
left/right 之外的内容判为非法
```

## 第 6 组：视觉结果接入练习

### 练习 14：模拟 YOLO 检测结果

如果还没有 YOLO 话题，先用简化话题模拟：

```text
/target_offset
类型 geometry_msgs/msg/Point
含义：
x = dx 像素
y = dy 像素
z = confidence
```

手动发布：

```bash
ros2 topic pub /target_offset geometry_msgs/msg/Point "{x: 20.0, y: -15.0, z: 0.85}"
```

任务：

状态机订阅 `/target_offset`。

判断：

```text
abs(dx) < 30
abs(dy) < 30
confidence > 0.6
```

满足连续 3 帧，则认为：

```text
target_aligned=true
```

验收：

```text
偏差小且置信度高时，状态机能进入目标对准状态
```

### 练习 15：加入视觉超时

任务：

如果 0.5 秒没有收到 `/target_offset`：

```text
target_found=false
target_aligned=false
```

验收：

```text
停止发布 /target_offset 后，状态机不再认为目标有效
```

## 第 7 组：飞控保护练习

### 练习 16：写速度限幅函数

任务：

在 C++ 里写函数：

```cpp
double limitSpeed(double value, double max_abs);
```

要求：

```text
输入 2.0, max_abs=0.5 -> 输出 0.5
输入 -2.0, max_abs=0.5 -> 输出 -0.5
输入 0.3, max_abs=0.5 -> 输出 0.3
```

验收：

在日志里打印测试结果。

### 练习 17：危险时发布悬停指令

任务：

写或扩展一个节点：

```text
safe_command_node.cpp
```

订阅：

```text
/obstacle_danger
/localization_valid
/target_offset
```

发布：

```text
/safe_velocity_cmd
类型 geometry_msgs/msg/Point
```

逻辑：

```text
如果 obstacle_danger=true -> 发布 0,0,0
如果 localization_valid=false -> 发布 0,0,0
否则根据 target_offset 发布小速度
速度必须限幅
```

验收：

```bash
ros2 topic echo /safe_velocity_cmd
```

重点：

先不要接真实飞控，只看话题输出是否安全。

## 第 8 组：联调练习

### 练习 18：完整模拟流程

目标：

不用起飞，只用 ROS2 话题模拟：

```text
定位有效
雷达安全
二维码确认
目标对准
状态机进入下一步
```

手动发布：

```bash
ros2 topic pub /localization_valid std_msgs/msg/Bool "{data: true}"
ros2 topic pub /obstacle_danger std_msgs/msg/Bool "{data: false}"
ros2 topic pub /qr_code/text std_msgs/msg/String "{data: 'man,apple,left'}"
ros2 topic pub /target_offset geometry_msgs/msg/Point "{x: 10.0, y: 5.0, z: 0.9}"
```

验收：

```text
/task_state 能按预期变化
/safe_velocity_cmd 不会输出危险大值
```

### 练习 19：模拟异常流程

测试这些情况：

```text
二维码格式错误
雷达 danger=true
定位 valid=false
视觉目标超时
target_offset 突然变很大
```

验收：

```text
状态机进入 FAILSAFE 或保持安全状态
速度指令变 0
日志能说明原因
```

## 第 9 组：真实雷达测试

### 练习 20：静止稳定性测试

任务：

雷达和障碍物都不动，记录 `/front_distance` 30 秒。

验收：

```text
距离波动范围最好小于 10cm
如果波动大，调整滤波窗口和裁剪区域
```

### 练习 21：靠近/远离测试

任务：

让障碍物从 2m 慢慢靠近到 0.5m，再远离。

验收：

```text
/front_distance 趋势正确
/obstacle_danger 在阈值附近不会来回疯狂跳
```

### 练习 22：坐标轴确认

任务：

分别把障碍物放在：

```text
前方
左方
右方
上方/下方
```

记录 x/y/z 的方向。必要时调整你的裁剪区域。

验收：

```text
能写清楚 MID360 当前安装方向下，x/y/z 分别代表什么
```

## 最终小项目

完成一个最小安全闭环：

```text
雷达检测前方障碍
  -> /obstacle_danger
  -> task_manager 判断状态
  -> safe_command 发布 0 速度或安全速度
```

最终验收：

```text
没有雷达数据：系统不发真实运动指令
障碍物靠近：系统进入安全状态
障碍物远离：系统恢复
定位无效：系统进入安全状态
所有状态变化都有日志
```

完成这个，你就已经不只是“调雷达”，而是在做无人机自主系统的安全底座。

