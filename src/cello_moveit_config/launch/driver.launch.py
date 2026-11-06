"""DEPRECATED: legacy Python driver (robo_driver + cello_controller).

Use bringup.launch.py, which runs the starai_hardware ros2_control driver.
The two cannot run at the same time: the new driver locks the serial port.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    # Create and return the launch description
    return LaunchDescription(
        [
            Node(
                package="robo_driver",
                executable="driver",
                name="cello_driver",
                output="screen",
            ),
            Node(
                package="cello_controller",
                executable="controller",
                name="cello_controller",
                output="screen",
                parameters=[{"min_point_duration_sec": 0.02}],
            )        
        ]
    )
