import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Find the packages that contain your move_group and rviz launch files.
    #    Adjust package names and paths to match your setup.
    moveit_config_pkg = get_package_share_directory('real_moveit_config')
    
    # Paths to the two launch files you want to include
    move_group_launch_path = os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
    rviz_launch_path       = os.path.join(moveit_config_pkg, 'launch', 'moveit_rviz.launch.py')

    # 2) Include the move_group launch file
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch_path)
    )

    # 3) Include the rviz launch file
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path)
    )

    # 5) Return them all in a single LaunchDescription
    return LaunchDescription([
        move_group_launch,
        rviz_launch,
    ])
