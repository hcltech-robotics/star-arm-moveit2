#!/usr/bin/env python3
"""Move the arm to a joint pose once at startup, straight through the trajectory controller.

Used by bringup to leave the parking stops: MoveIt refuses to plan from there (the folded
arm counts as a self-collision in the coarse meshes), so this skips MoveIt. Because it
also skips collision checking, it only moves when every joint already starts within
`max_start_deg` of the target; otherwise it logs a warning and leaves the arm alone.

    ros2 run starai_hardware go_to_pose.py --ros-args \
        -p joints:="[joint1, joint2]" -p positions:="[0.0, -1.5]"
"""
import math

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rclpy.init()
    node = Node('go_to_pose')
    log = node.get_logger()
    joints = node.declare_parameter('joints', ['']).value
    target = node.declare_parameter('positions', [0.0]).value
    action = node.declare_parameter('action', '/arm_controller/follow_joint_trajectory').value
    max_start = math.radians(node.declare_parameter('max_start_deg', 15.0).value)
    speed = math.radians(node.declare_parameter('speed_dps', 10.0).value)
    if len(joints) != len(target) or not joints[0]:
        log.error('joints and positions must be non-empty and the same length')
        return 1

    client = ActionClient(node, FollowJointTrajectory, action)
    while rclpy.ok() and not client.wait_for_server(timeout_sec=2.0):
        log.info(f'waiting for {action}')

    state = {}
    node.create_subscription(
        JointState, '/joint_states', lambda m: state.update(zip(m.name, m.position)), 10)
    while rclpy.ok() and not all(j in state for j in joints):
        rclpy.spin_once(node, timeout_sec=0.1)

    dist = [abs(t - state[j]) for j, t in zip(joints, target)]
    far = [f'{j} {math.degrees(d):.1f} deg' for j, d in zip(joints, dist) if d > max_start]
    if far:
        log.warning(f'not moving: too far from the target pose ({", ".join(far)}; limit '
                 f'{math.degrees(max_start):.0f} deg). Move it with MoveIt instead.')
        return 0

    duration = max(2.0, max(dist) / speed)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(joints)
    goal.trajectory.points = [JointTrajectoryPoint(
        positions=list(target), velocities=[0.0] * len(joints),
        time_from_start=Duration(seconds=duration).to_msg())]
    log.info(f'moving to the target pose in {duration:.1f} s')
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    if not send.result().accepted:
        log.error('goal rejected')
        return 1
    done = send.result().get_result_async()
    rclpy.spin_until_future_complete(node, done)
    code = done.result().result.error_code
    (log.info if code == 0 else log.error)(f'done (error_code {code})')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if code == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
