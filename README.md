# Starai Arm 机械臂-ROS2 Moveit 使用教程 Starai Arm Manipulator - ROS2 MoveIt Guide

<div align="center">
  <div style="display: flex; gap: 1rem; justify-content: center; align-items: center;" >
    <img
      src="images\viola_and_violin.jpg"
      alt="SO-101 follower arm"
      title="SO-101 follower arm"
      style="width: 80%;"
    />
    <img
      src="images\cello.jpg"
      alt="SO-101 leader arm"
      title="SO-101 leader arm"
      style="width: 80%;"
    />
  </div>
</div>


## Setup

Ubuntu 24.04, ROS 2 Jazzy, MoveIt 2. No Python servo SDK is needed: the driver speaks the
Fashion Star protocol itself.

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers
mkdir -p ~/starai_ws/src && cd ~/starai_ws/src
git clone https://github.com/hcltech-robotics/star-arm-moveit2.git
cd ~/starai_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## Packages

| package | contents |
|---|---|
| [`starai_hardware`](src/starai_hardware/README.md) | `ros2_control` driver for the Fashion Star servo bus |
| `starai_bringup` | `bringup.launch.py` (any arm) and `teleop.launch.py` (Violin -> Cello or Viola) |
| `cello_description`, `viola_description` | URDF and meshes |
| `cello_moveit_config`, `viola_moveit_config` | MoveIt config, controllers, limits, named poses |

The Violin has no packages of its own: it is the Viola's description in read-only mode.

## Run an arm

```bash
ros2 launch starai_bringup bringup.launch.py arm:=cello port:=/dev/ttyUSB0
ros2 launch starai_bringup bringup.launch.py arm:=viola port:=/dev/ttyUSB0
```

This starts the driver, the arm and gripper controllers, MoveIt and RViz. If the arm is
within 15 deg of its `rest` pose (e.g. parked), it then moves there (`startup_pose:=''` to
stay put). Plan and execute in RViz, or use the `arm_controller` / `hand_controller`
actions directly.

Without an arm:

```bash
# simulated joints
ros2 launch starai_bringup bringup.launch.py arm:=viola use_mock_hardware:=true
# simulated servo bus: runs the real driver
ros2 run starai_hardware fake_fashionstar_bus.py --link /tmp/ttyFAKE &
ros2 launch starai_bringup bringup.launch.py arm:=viola port:=/tmp/ttyFAKE
```

## Teleop: Violin leads, Cello or Viola follows

Connect both arms, then:

```bash
ros2 launch starai_bringup teleop.launch.py arm:=cello follower_port:=/dev/ttyUSB0 leader_port:=/dev/ttyUSB1
```

* The Violin runs the same driver read-only, in the `/violin` namespace: torque stays off,
  no motion command is ever sent to it, and it publishes `/violin/joint_states`.
* The Violin's joint zeros and directions match the followers, so angles pass through
  unchanged (checked against the Cello).
* At start (and after the leader goes silent), the follower moves to the Violin's pose at
  0.3 rad/s, then follows at up to 1.5 rad/s per joint. There is no collision checking.
* The Violin's trigger is mapped linearly onto the follower's gripper
  (`leader_gripper_open/closed` and `follower_gripper_open/closed` on the `teleop` node).
  A grasped object is held where the fingers were blocked, so holding it does not
  overheat the gripper.
* MoveIt is off by default (`use_moveit:=true` to start it as well).

Use a stable device name per arm (udev rule, see the driver README) so the leader and
follower ports don't swap.

## Teach mode and recording

```bash
ros2 run starai_hardware teach_mode on CelloSystem    # torque off, joint states keep publishing
ros2 bag record /joint_states                        # record while moving the arm by hand
ros2 run starai_hardware teach_mode off CelloSystem   # hold the current pose again
```

The hardware component is `CelloSystem` or `ViolaSystem`.

## FAQ

如果rivz2界面出现频闪，可以尝试以下指令/
If you experience flickering in the RViz2 interface, try the following commands:

  ```bash
  export QT_AUTO_SCREEN_SCALE_FACTOR=0
  ```
<!-- markdownlint-enable MD033 -->