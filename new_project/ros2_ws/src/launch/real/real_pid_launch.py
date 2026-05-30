import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #main_ctrl
    main_ctrl_node = Node(
        package='main',
        executable='pid_test'
    )
    # main_ctrl_planner_node = Node(
    #     package='main',
    #     executable='main_ctrl_planner'
    # )
    #camera
    # camera_pkg_share = get_package_share_directory('orbbec_camera')
    # camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([camera_pkg_share, '/launch/gemini_e.launch.launch.py']),
        
    # )

    # camera_qr_node= Node(
    #     package='camera',
    #     executable='qr_show'
    # )
    #tf
    tf_publisher_node = Node(
        package='tf',
        executable='tf_publisher'
    )
    #communication
    controller_node = Node(
        package='communication',
        executable='uart'
    )
    # filter_odom_node = Node(
    #     package='communication',
    #     executable='odom_lowpassfilter'
    # )
    #vins_mono
    # feature_tracker_pkg_share = get_package_share_directory('feature_tracker')
    # feature_tracker_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([feature_tracker_pkg_share, '/launch/vins_feature_tracker.launch.py']),
        
    # )
    # vins_estimator_pkg_share = get_package_share_directory('vins_estimator')
    # vins_estimator_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([vins_estimator_pkg_share, '/launch/euroc.launch.py']),
        
    # )
    
    #ego_planner
    # advanced_param_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
    #     launch_arguments={
    #         'drone_id': drone_id,
    #         'map_size_x_': map_size_x,
    #         'map_size_y_': map_size_y,
    #         'map_size_z_': map_size_z,
    #         'odometry_topic': odom_topic,
    #         'obj_num_set': obj_num,
            
    #         'camera_pose_topic': 'pcl_render_node/camera_pose',
    #         'depth_topic': 'pcl_render_node/depth',
    #         'cloud_topic': 'pcl_render_node/cloud',
            
    #         'cx': str(321.04638671875),
    #         'cy': str(243.44969177246094),
    #         'fx': str(387.229248046875),
    #         'fy': str(387.229248046875),
    #         'max_vel': str(2.0),
    #         'max_acc': str(6.0),
    #         'planning_horizon': str(7.5),
    #         'use_distinctive_trajs': 'True',
    #         'flight_type': str(2),
    #         'point_num': str(4),
    #         'point0_x': str(15.0),
    #         'point0_y': str(0.0),
    #         'point0_z': str(1.0),
            
    #         'point1_x': str(-15.0),
    #         'point1_y': str(0.0),
    #         'point1_z': str(1.0),
            
    #         'point2_x': str(15.0),
    #         'point2_y': str(0.0),
    #         'point2_z': str(1.0),
            
    #         'point3_x': str(-15.0),
    #         'point3_y': str(0.0),
    #         'point3_z': str(1.0),
            
    #         'point4_x': str(15.0),
    #         'point4_y': str(0.0),
    #         'point4_z': str(1.0),
    #     }.items()
    # )
        
    # traj_server_node = Node(
    #     package='ego_planner',
    #     executable='traj_server',
    #     name=['drone_', drone_id, '_traj_server'],
    #     output='screen',
    #     remappings=[
    #         ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
    #         ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
    #     ],
    #     parameters=[
    #         {'traj_server/time_forward': 1.0}
    #     ]
    # )

    launch_fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mid.launch.py'
            ])
        ]),
    
    )

    launch_livox_ros_driver2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        ]),

    )
        
    ld = LaunchDescription()
    #lidar
    ld.add_action(launch_fast_lio)
    ld.add_action(launch_livox_ros_driver2)


    #camera
    #ld.add_action(camera_launch)
    #ld.add_action(camera_qr_node)
    
    #ego_planner
    #ld.add_action(advanced_param_include)
    #ld.add_action(traj_server_node)
    
    #vins_mono
    #ld.add_action(feature_tracker_launch)
    #ld.add_action(vins_estimator_launch)
    
    #communicator
    ld.add_action(controller_node)
    #ld.add_action(filter_odom_node)
    #main
    ld.add_action(main_ctrl_node)
    #ld.add_action(main_ctrl_planner_node)
    
    #tf
    ld.add_action(tf_publisher_node)
    
    return ld