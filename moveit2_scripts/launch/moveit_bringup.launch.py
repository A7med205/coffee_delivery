import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config_pkg = get_package_share_directory('real_moveit_config')
    
    move_group_launch_path = os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
    rviz_launch_path       = os.path.join(moveit_config_pkg, 'launch', 'moveit_rviz.launch.py')

    # Move group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch_path)
    )

    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path)
    )

    return LaunchDescription([
        move_group_launch,
        rviz_launch,
    ])
