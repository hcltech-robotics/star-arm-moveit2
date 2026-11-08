"""Violin -> Viola teleop: move the Violin (leader) by hand, the Viola (follower) follows.

    ros2 launch starai_bringup teleop.launch.py follower_port:=/dev/ttyUSB0 leader_port:=/dev/ttyUSB1

The Violin runs the same driver read-only (torque off) in the /violin namespace and
publishes /violin/joint_states. The Viola is the normal bringup (MoveIt off by default).
At start the follower moves slowly to the leader's pose; see starai_bringup/teleop.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("starai_bringup")
    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "bringup.launch.py")),
        launch_arguments={
            "arm": "viola",
            "port": LaunchConfiguration("follower_port"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "startup_pose": "",  # teleop moves it to the leader's pose itself
            "use_moveit": LaunchConfiguration("use_moveit"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    leader_description = ParameterValue(Command([
        "xacro ", os.path.join(get_package_share_directory("viola_moveit_config"), "config",
                               "viola_description.urdf.xacro"),
        " leader:=true port:=", LaunchConfiguration("leader_port"),
        " use_mock_hardware:=", LaunchConfiguration("use_mock_hardware"),
    ]), value_type=str)
    leader = [
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace="violin",
            parameters=[{"robot_description": leader_description, "frame_prefix": "violin/"}],
        ),
        Node(
            package="controller_manager", executable="ros2_control_node", namespace="violin",
            parameters=[os.path.join(share, "config", "violin_controllers.yaml")],
            remappings=[("/violin/controller_manager/robot_description", "/violin/robot_description")],
            output="both",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager",
                       "/violin/controller_manager"],
        ),
    ]
    teleop = Node(package="starai_bringup", executable="teleop.py", output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("follower_port", default_value="/dev/ttyUSB0",
                              description="Viola (follower) servo bus"),
        DeclareLaunchArgument("leader_port", default_value="/dev/ttyUSB1",
                              description="Violin (leader) servo bus"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="true: simulated joints for both arms"),
        DeclareLaunchArgument("use_moveit", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        follower, *leader, teleop,
    ])
