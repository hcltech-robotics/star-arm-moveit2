"""DEPRECATED: legacy Python driver (robo_driver + cello_controller).

Use bringup.launch.py, which runs the starai_hardware ros2_control driver.
The two cannot run at the same time: the new driver locks the serial port.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    servo_port_name = LaunchConfiguration("servo_port_name")

    # Create and return the launch description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "servo_port_name",
                default_value="/dev/star_arm_viola",
                description="Serial device path for the robot servos",
            ),
            Node(
                package="robo_driver",
                executable="driver",
                name="cello_driver",
                output="screen",
                parameters=[{"servo_port_name": servo_port_name}],
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
