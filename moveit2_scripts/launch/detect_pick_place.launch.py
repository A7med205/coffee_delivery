import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    # MoveItCpp API executable
    moveit_cpp_node = Node(
        name="pick_and_place",
        package="moveit2_scripts",
        executable="pick_and_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False},
        ],
    )

    # Slot detector
    slot_detector_node = Node(
        name="slot_detector",
        package="moveit2_scripts",
        executable="slot_detector.py",
        output="screen",
    )

    return LaunchDescription(
        [moveit_cpp_node,
        slot_detector_node]
    )