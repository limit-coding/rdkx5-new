import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    ld = LaunchDescription()
    
    ego_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ego_planner'), 'launch', 'run_in_sim.launch.py')),
    )
    
    orbbec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'gemini_e.launch.py')),
    )
    
    vins_mono = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vins_mono'), 'launch', 'vins_rviz.launch.py')),
    )
    ld.add_action(ego_planner)
    ld.add_action(orbbec_camera)
    ld.add_action(vins_mono)
    return ld