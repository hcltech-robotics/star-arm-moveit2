#!/usr/bin/env python3
"""Leader-follower teleop: a read-only leader arm (Violin) moved by hand drives a follower
arm (Viola) through its standard controllers.

    leader  /violin/joint_states  ->  follower /arm_controller/joint_trajectory (streamed)
                                               /hand_controller/gripper_cmd (on change)

Safety: every follower command is rate-limited per joint. At start, and whenever the leader
was silent, the follower first moves to the leader's pose at `approach_speed`, and follows
at `max_speed` only once it has caught up. There is no collision checking.
"""

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def step_toward(current, target, max_step):
    """Move each joint at most max_step toward its target."""
    return [c + max(-max_step, min(max_step, t - c)) for c, t in zip(current, target)]


def map_gripper(x, leader_open, leader_closed, follower_open, follower_closed):
    """Linear map of the leader's gripper reading onto the follower's range, clamped."""
    f = (x - leader_open) / (leader_closed - leader_open)
    f = max(0.0, min(1.0, f))
    return follower_open + f * (follower_closed - follower_open)


class Teleop(Node):

    def __init__(self):
        super().__init__('teleop')
        p = self.declare_parameter
        self.joints = p('joints', [f'joint{i}' for i in range(1, 7)]).value
        self.gripper = p('gripper_joint', 'joint7_left').value
        self.rate = p('rate', 50.0).value
        self.max_speed = p('max_speed', 1.5).value             # rad/s while following
        self.approach_speed = p('approach_speed', 0.3).value   # rad/s while catching up
        self.synced_tol = p('synced_tolerance', 0.05).value     # rad
        self.lead = p('lead_time', 0.1).value                   # s
        self.stale = p('leader_timeout', 0.2).value             # s
        self.g_leader = (p('leader_gripper_open', 0.0).value, p('leader_gripper_closed', -0.03).value)
        self.g_follower = (p('follower_gripper_open', 0.0).value,
                           p('follower_gripper_closed', -0.03).value)
        self.g_deadband = p('gripper_deadband', 0.001).value    # m

        self.leader = {}
        self.leader_stamp = None
        self.follower = {}
        self.cmd = None
        self.synced = False
        self.last_gripper = None
        self.create_subscription(
            JointState, p('leader_topic', '/violin/joint_states').value, self.on_leader, 10)
        self.create_subscription(
            JointState, p('follower_topic', '/joint_states').value, self.on_follower, 10)
        self.pub = self.create_publisher(
            JointTrajectory, p('trajectory_topic', '/arm_controller/joint_trajectory').value, 10)
        self.grip = ActionClient(
            self, GripperCommand, p('gripper_action', '/hand_controller/gripper_cmd').value)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info('waiting for leader and follower joint states')

    def on_leader(self, msg):
        self.leader.update(zip(msg.name, msg.position))
        self.leader_stamp = self.get_clock().now()

    def on_follower(self, msg):
        self.follower.update(zip(msg.name, msg.position))

    def tick(self):
        names = self.joints + [self.gripper]
        now = self.get_clock().now()
        fresh = (self.leader_stamp is not None
                 and (now - self.leader_stamp).nanoseconds * 1e-9 < self.stale)
        if not fresh or not all(j in self.leader for j in names):
            if self.synced:
                self.get_logger().warning('leader silent: holding; will re-approach slowly')
            self.synced = False
            self.cmd = None  # restart from the follower's measured pose
            return
        if self.cmd is None:
            if not all(j in self.follower for j in self.joints):
                return
            self.cmd = [self.follower[j] for j in self.joints]
            self.get_logger().info('approaching the leader pose')

        target = [self.leader[j] for j in self.joints]
        speed = self.max_speed if self.synced else self.approach_speed
        self.cmd = step_toward(self.cmd, target, speed / self.rate)
        if not self.synced and max(abs(t - c) for t, c in zip(target, self.cmd)) < self.synced_tol:
            self.synced = True
            self.get_logger().info('following the leader')

        traj = JointTrajectory()
        traj.joint_names = list(self.joints)
        # Positions only: the controller rejects a non-zero end velocity, and one would keep
        # the follower aimed ahead if the stream stopped. Its spline supplies the velocity.
        traj.points = [JointTrajectoryPoint(
            positions=self.cmd, time_from_start=Duration(seconds=self.lead).to_msg())]
        self.pub.publish(traj)

        g = map_gripper(self.leader[self.gripper], *self.g_leader, *self.g_follower)
        if self.synced and (self.last_gripper is None
                            or abs(g - self.last_gripper) > self.g_deadband):
            if self.grip.server_is_ready():
                goal = GripperCommand.Goal()
                goal.command.position = g
                self.grip.send_goal_async(goal)
                self.last_gripper = g


def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise  # a timer that fires during Ctrl-C fails to publish: not an error
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
