"""StarAI arm bringup: ros2_control hardware + controllers + MoveIt (+ RViz).

Real arm (arm:=cello or arm:=viola):
    ros2 launch starai_bringup bringup.launch.py arm:=cello port:=/dev/ttyUSB0
Simulated joints:
    ros2 launch starai_bringup bringup.launch.py arm:=viola use_mock_hardware:=true
Simulated servo bus (exercises the real driver, no arm needed):
    ros2 run starai_hardware fake_fashionstar_bus.py --link /tmp/ttyFAKE &
    ros2 launch starai_bringup bringup.launch.py port:=/tmp/ttyFAKE

MoveIt and MoveIt Servo talk to the standard JointTrajectoryController, which streams to
the servos through starai_hardware/FashionStarSystem.
"""
import os
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


ARMS = ("cello", "viola")


def launch_setup(context):
    arm = LaunchConfiguration("arm").perform(context)
    if arm not in ARMS:
        raise RuntimeError(f"arm must be one of {ARMS}, got '{arm}'")
    use_mock = LaunchConfiguration("use_mock_hardware").perform(context)
    port = LaunchConfiguration("port").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)

    moveit_config = (
        MoveItConfigsBuilder(f"{arm}_description", package_name=f"{arm}_moveit_config")
        .robot_description(
            mappings={"use_mock_hardware": use_mock, "port": port, "baudrate": baudrate}
        )
        .to_moveit_configs()
    )
    controllers_yaml = str(moveit_config.package_path / "config" / "ros2_controllers.yaml")
    distro = os.environ.get("ROS_DISTRO", "jazzy")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Humble reads robot_description as a parameter; newer distros subscribe to the topic.
    cm_params = [controllers_yaml]
    if distro == "humble":
        cm_params.insert(0, moveit_config.robot_description)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=cm_params,
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "arm_controller", "hand_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="both",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "moveit_manage_controllers": False,
                "monitor_dynamics": False,
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_moveit")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config" / "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Leave the parking stops: move to a named SRDF pose once the arm controller is up.
    # MoveIt cannot plan from the stops (the folded arm counts as a self-collision).
    # Mock hardware starts at all zeros, too far to move, so it is skipped there.
    nodes = [robot_state_publisher, ros2_control_node, spawner, move_group, rviz]
    startup_pose = LaunchConfiguration("startup_pose").perform(context)
    if startup_pose and use_mock != "true":
        srdf = ET.fromstring(
            moveit_config.robot_description_semantic["robot_description_semantic"])
        state = next((g for g in srdf.iter("group_state")
                      if g.get("name") == startup_pose and g.get("group") == "arm"), None)
        if state is None:
            raise RuntimeError(f"startup_pose '{startup_pose}' is not an arm state in the SRDF")
        pose = [(j.get("name"), j.get("value")) for j in state.iter("joint")]
        nodes.append(Node(
            package="starai_hardware",
            executable="go_to_pose.py",
            output="screen",
            parameters=[{
                "joints": [j for j, _ in pose],
                "positions": [float(v) for _, v in pose],
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="cello", description="cello | viola"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="true: simulated joints, false: real servo bus"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0",
                              description="Servo bus serial port"),
        DeclareLaunchArgument("baudrate", default_value="1000000"),
        DeclareLaunchArgument(
            "startup_pose", default_value="rest",
            description="SRDF arm pose to move to after startup, only if every joint is "
                        "within 15 deg of it ('' = stay put)"),
        DeclareLaunchArgument("use_moveit", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
