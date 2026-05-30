from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    task_state_machine_node = Node(
        package='main',
        executable='task_state_machine',
        output='screen',
    )

    yolo_node = Node(
        package='camera',
        executable='animal_enable',
        output='screen',
    )

    usb_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hobot_usb_cam'),
                'launch',
                'hobot_usb_cam.launch.py'
            ])
        ]),
        launch_arguments={
            'usb_video_device': '/dev/video0',
            'usb_image_width': '1280',
            'usb_image_height': '720',
            'usb_pixel_format': 'mjpeg',
            'usb_framerate': '30',
        }.items(),
    )

    qr_node = Node(
        package='camera',
        executable='qr_show',
        name='qr_detector',
        output='screen',
        parameters=[{
            'image_topic': '/image',
        }],
    )

    uart_node = Node(
        package='communication',
        executable='uart',
        output='screen',
    )
        
    ld = LaunchDescription()
    ld.add_action(usb_camera_launch)
    ld.add_action(TimerAction(period=2.0, actions=[qr_node]))
    ld.add_action(yolo_node)
    ld.add_action(uart_node)
    ld.add_action(task_state_machine_node)
    
    return ld
