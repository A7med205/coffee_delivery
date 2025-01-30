import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # MoveItCpp API executable
    moveit_cpp_node = Node(
        name="pick_and_place",
        package="moveit2_scripts",
        executable="pick_and_place",
        output="screen",
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