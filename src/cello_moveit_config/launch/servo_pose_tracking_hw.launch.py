import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_driver = LaunchConfiguration("launch_driver")
    spawn_controllers = LaunchConfiguration("spawn_controllers")
    controller_manager_name = LaunchConfiguration("controller_manager_name")
    use_action_bridge = LaunchConfiguration("use_action_bridge")

    moveit_config = MoveItConfigsBuilder(
        "cello_description", package_name="cello_moveit_config"
    ).to_moveit_configs()

    pkg_share = get_package_share_directory("cello_moveit_config")
    servo_params_path = os.path.join(pkg_share, "config", "moveit_servo.yaml")
    pose_tracking_params_path = os.path.join(
        pkg_share, "config", "pose_tracking_settings.yaml"
    )
    rviz_config_path = os.path.join(pkg_share, "config", "moveit.rviz")
    driver_launch_path = os.path.join(pkg_share, "launch", "driver.launch.py")

    with open(servo_params_path, "r", encoding="utf-8") as f:
        servo_yaml = yaml.safe_load(f) or {}
    with open(pose_tracking_params_path, "r", encoding="utf-8") as f:
        pose_tracking_yaml = yaml.safe_load(f) or {}

    # PoseTracking publishes metric twists (m/s, rad/s), so Servo input must be speed_units.
    servo_yaml["command_in_type"] = "speed_units"

    merged_servo_params = dict(servo_yaml)
    merged_servo_params.update(pose_tracking_yaml)
    # Hardware-safe overrides: reduce loop pressure to avoid control-rate misses.
    merged_servo_params["publish_period"] = 0.02
    merged_servo_params["low_latency_mode"] = False
    merged_servo_params["scale"] = {
        "linear": 0.6,
        "rotational": 1.0,
        "joint": 1.0,
    }
    merged_servo_params["joint_limit_margin"] = 0.03
    merged_servo_params["x_proportional_gain"] = 8.0
    merged_servo_params["y_proportional_gain"] = 8.0
    merged_servo_params["z_proportional_gain"] = 8.0
    merged_servo_params["angular_proportional_gain"] = 2.0
    merged_servo_params["windup_limit"] = 0.05
    servo_params = {"moveit_servo": merged_servo_params}

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch_path),
        condition=IfCondition(launch_driver),
    )

    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "arm_controller", "-c", controller_manager_name],
        output="screen",
        condition=IfCondition(spawn_controllers),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    pose_tracking_node = Node(
        package="cello_servo_pose_tracking",
        executable="xr_pose_tracking_node",
        name="xr_pose_tracking",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            servo_params,
            {
                "use_sim_time": use_sim_time,
                "position_tolerance": 0.005,
                "orientation_tolerance": 0.01,
                "target_pose_timeout": 0.75,                
                "home_joint_positions": [0.1, -0.8, 0.8, -0.7, 0.5, 0.2],
                "home_trajectory_time_sec": 2.,
                "home_wait_for_subscribers_sec": 10.0,
                "enable_startup_home": True,
                "control_x_translation": merged_servo_params.get("control_x_translation", True),
                "control_y_translation": merged_servo_params.get("control_y_translation", True),
                "control_z_translation": merged_servo_params.get("control_z_translation", True),
                "control_x_rotation": merged_servo_params.get("control_x_rotation", True),
                "control_y_rotation": merged_servo_params.get("control_y_rotation", True),
                "control_z_rotation": merged_servo_params.get("control_z_rotation", True),
            },
        ],
    )

    wrist_to_target_node = Node(
        package="cello_servo_pose_tracking",
        executable="wrist_tf_to_target_pose_node",
        name="wrist_tf_to_target_pose",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "world_frame": "world",
                "wrist_frame": "right_wrist",
                "ee_frame": merged_servo_params.get("ee_frame_name", "link6"),
                "output_topic": "/xr_target_pose",
                "publish_rate_hz": 50.0,
                "reset_on_time_jump": True,
                "reset_on_forward_time_jump": False,
                "time_jump_threshold_sec": 1.0,
                "translation_scale_x": -.20,
                "translation_scale_y": -.20,
                "translation_scale_z": .12,
                "orientation_mode": "world_translation_only",
                "publish_markers": True,
                "marker_topic": "/xr_target_markers",
            }
        ],
    )

    action_bridge_node = Node(
        package="cello_servo_pose_tracking",
        executable="joint_trajectory_action_bridge_node",
        name="joint_trajectory_action_bridge",
        output="screen",
        condition=IfCondition(use_action_bridge),
        parameters=[
            {
                "input_topic": merged_servo_params.get("command_out_topic", "/arm_controller/joint_trajectory"),
                "action_name": "/arm_controller/follow_joint_trajectory",
                "queue_depth": 10,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS /clock time instead of system time",
            ),
            DeclareLaunchArgument(
                "launch_driver",
                default_value="true",
                description="Launch the real hardware driver stack from driver.launch.py",
            ),
            DeclareLaunchArgument(
                "spawn_controllers",
                default_value="false",
                description="Spawn ros2_control controllers on external controller_manager",
            ),
            DeclareLaunchArgument(
                "controller_manager_name",
                default_value="/controller_manager",
                description="Controller manager namespace when spawn_controllers is true",
            ),
            DeclareLaunchArgument(
                "use_action_bridge",
                default_value="true",
                description="Bridge JointTrajectory topic to FollowJointTrajectory action for hardware stacks",
            ),
            driver_launch,
            robot_state_publisher,
            controllers_spawner,
            pose_tracking_node,
            wrist_to_target_node,
            action_bridge_node,
            rviz_node,
        ]
    )
