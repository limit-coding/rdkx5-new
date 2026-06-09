"""
Livox MID360 -> FAST_LIO -> C++相对定位 -> C++飞控串口桥接

启动顺序：
1. 先启动 Livox 驱动，让 /livox/lidar 开始有原始雷达点云。
2. 2 秒后启动 FAST-LIO，让它订阅雷达点云并输出 /Odometry。
3. 4 秒后启动本项目的 C++ 节点：点云转换、相对定位、飞控串口桥接、二维码检测。

这里的 TimerAction 不是为了“严格同步”，只是给前面的节点一点启动时间，
减少刚开机时后级节点一直报“没有数据”的情况。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Livox 官方驱动：MID360 原始数据入口，输出 /livox/lidar。
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py',
            )
        )
    )

    # FAST-LIO：用雷达点云做室内里程计/SLAM，核心输出是 /Odometry。
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fast_lio'),
                'launch',
                'mapping.launch.py',
            )
        ),
        launch_arguments={'rviz': 'false'}.items(),
    )

    # 点云格式转换：/livox/lidar -> /mid360/xy_points。
    # 这个节点不做控制，只是把 Livox 自定义点云翻译成标准 PointCloud2。
    mid360_xy_node = Node(
        package='lidar_fc_cpp',
        executable='mid360_xy_cpp',
        name='mid360_xy_node',
        output='screen',
    )

    # 相对定位：/Odometry -> /relative_pose + /localization_valid。
    # max_relative_meters 是定位安全边界，超过这个范围会认为定位不可信。
    relative_pose_node = Node(
        package='lidar_fc_cpp',
        executable='relative_pose_cpp',
        name='relative_pose_node',
        output='screen',
        parameters=[{
            'max_relative_meters': 10.0,
        }],
    )

    # 飞控桥接：/relative_pose -> 串口帧 -> 匿名凌霄飞控。
    # 这里发送的是“当前位置相对开机原点的坐标”，单位在 C++ 节点里转成 cm。
    fc_bridge_node = Node(
        package='lidar_fc_cpp',
        executable='fc_bridge_cpp',
        name='fc_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyFC',
            'baudrate': 115200,
            'send_freq': 20.0,
            'max_xy_meters': 10.0,
            'clamp_xy_instead_of_zero': True,
        }],
    )

    # 二维码节点和雷达定位链路并列运行，不参与雷达坐标计算。
    qr_detector_node = Node(
        package='lidar_fc_cpp',
        executable='qr_detector_cpp',
        name='qr_detector_cpp',
        output='screen',
        parameters=[{
            'image_topic': '/image',
            'confirm_frames': 3,
        }],
    )

    # 圆环激光雷达检测：订阅 /mid360/xy_points，RANSAC 圆拟合，
    # 发布 /ring/detected + /ring/offset（y=横向偏移m）供 fc_bridge 转发给飞控。
    ring_scripts_dir = os.path.join(
        get_package_share_directory('lidar_fc_cpp'),
        '..', '..', '..', '..', 'src', 'lidar_fc_cpp', 'scripts',
    )
    ring_lidar_node = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(ring_scripts_dir, 'ring_lidar_detector.py'),
        ],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
    )

    return LaunchDescription([
        livox_launch,
        TimerAction(period=2.0, actions=[fast_lio_launch]),
        TimerAction(period=4.0, actions=[
            mid360_xy_node,
            relative_pose_node,
            fc_bridge_node,
            qr_detector_node,
            ring_lidar_node,
        ]),
    ])
