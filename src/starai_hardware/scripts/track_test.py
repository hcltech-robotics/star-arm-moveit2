#!/usr/bin/env python3
"""Trajectory tracking benchmark for the arm controller.

First moves slowly to a safe center pose (default: all zeros, the SRDF "home"), shows
the planned sweep and waits for Enter. Then sends a smooth multi-joint trajectory
(raised-cosine sweeps away from the center) to a FollowJointTrajectory action and measures how well /joint_states follows it:

  * RMS / max tracking error per joint (as executed)
  * pure delay: the time shift that best aligns measured with desired
  * RMS error after removing that delay (shape error)
  * final settling error, action result, joint_states rate

Works against the real arm, the fake bus, or mock hardware:

    ros2 run starai_hardware track_test.py --amplitude 0.3 --period 2.0 --cycles 2

There is no collision checking: keep the workspace around the center pose clear.
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def to_duration(t):
    sec = int(t)
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))


class TrackTest(Node):
    def __init__(self, args):
        super().__init__('track_test')
        self.args = args
        self.lock = threading.Lock()
        self.samples = []   # (t, {name: pos})
        self.latest = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.client = ActionClient(self, FollowJointTrajectory, args.action)

    def on_js(self, msg):
        t = time.monotonic()
        pos = dict(zip(msg.name, msg.position))
        if any(j not in pos for j in self.args.joints):
            return  # e.g. empty messages while a hardware component changes state
        with self.lock:
            self.latest = pos
            self.samples.append((t, self.latest))

    def desired(self, q0, t):
        """Raised-cosine sweep: zero velocity and acceleration at the ends of each cycle."""
        a, T = self.args.amplitude, self.args.period
        out_q, out_v, out_a = [], [], []
        for k, q in enumerate(q0):
            sign = 1.0 if k % 2 == 0 else -1.0       # alternate directions across joints
            w = 2.0 * math.pi / T
            amp = sign * a * (0.6 if k >= 3 else 1.0)
            out_q.append(q + amp * 0.5 * (1.0 - math.cos(w * t)))
            out_v.append(amp * 0.5 * w * math.sin(w * t))
            out_a.append(amp * 0.5 * w * w * math.cos(w * t))
        return out_q, out_v, out_a

    def run(self):
        args = self.args
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'action server {args.action} not available')
            return 1
        t_wait = time.monotonic() + 5.0
        while self.latest is None and time.monotonic() < t_wait:
            time.sleep(0.05)
        if self.latest is None:
            self.get_logger().error('no /joint_states received')
            return 1
        joints = args.joints
        center = args.center or [0.0] * len(joints)
        if len(center) != len(joints):
            self.get_logger().error('--center needs one value per joint')
            return 1
        q_now = [self.latest[j] for j in joints]

        q_end, _, _ = self.desired(center, args.period / 2.0)  # furthest point of a sweep
        d = 180.0 / math.pi
        print('planned motion (deg): now -> center -> sweep extreme')
        for j, a, c, e in zip(joints, q_now, center, q_end):
            print(f'  {j:8s} {a * d:7.1f} -> {c * d:7.1f} -> {e * d:7.1f}')
        if not args.yes:
            input('Workspace clear? Press Enter to move (Ctrl-C to abort) ')

        # Slow move to the center (<= 0.3 rad/s) so the sweep starts from a known pose.
        home_t = max(3.0, max(abs(a - c) for a, c in zip(q_now, center)) / 0.3)
        home = FollowJointTrajectory.Goal()
        home.trajectory.joint_names = joints
        home.trajectory.points.append(JointTrajectoryPoint(
            positions=center, velocities=[0.0] * len(joints), time_from_start=to_duration(home_t)))
        result, _, _ = self.execute(home)
        if result is None or result.error_code != 0:
            self.get_logger().error('could not reach the center pose')
            return 1
        time.sleep(args.settle)
        q0 = center

        total = args.period * args.cycles
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        n = int(total / args.dt)
        for i in range(1, n + 1):
            t = i * args.dt
            q, v, a = self.desired(q0, t)
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=q, velocities=v, accelerations=a, time_from_start=to_duration(t)))

        with self.lock:
            self.samples.clear()
        result, t0, send_t = self.execute(goal)
        if result is None:
            return 1
        time.sleep(args.settle)
        with self.lock:
            samples = list(self.samples)
        return self.report(joints, q0, samples, t0, send_t, total, result)

    def execute(self, goal):
        """Sends a goal and waits. Returns (result, accept_time, send_time); result None if rejected."""
        send_t = time.monotonic()
        fut = self.client.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.005)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('goal rejected')
            return None, 0.0, send_t
        t0 = time.monotonic()
        res_fut = gh.get_result_async()
        while not res_fut.done():
            time.sleep(0.01)
        return res_fut.result().result, t0, send_t

    def report(self, joints, q0, samples, t0, send_t, total, result):
        args = self.args
        traj = [(t - t0, s) for t, s in samples if 0.0 <= t - t0 <= total]
        if len(traj) < 10:
            print('not enough samples')
            return 1
        stamps = [t for t, _ in samples]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0

        def errors(shift):
            errs = [[] for _ in joints]
            for t, s in traj:
                qd, _, _ = self.desired(q0, max(0.0, t - shift))
                for k, j in enumerate(joints):
                    errs[k].append(s[j] - qd[k])
            return errs

        def rms(x):
            return math.sqrt(sum(e * e for e in x) / len(x))

        raw = errors(0.0)
        best_shift, best_cost = 0.0, sum(rms(e) for e in raw)
        for ms in range(2, 201, 2):
            cost = sum(rms(e) for e in errors(ms / 1000.0))
            if cost < best_cost:
                best_shift, best_cost = ms / 1000.0, cost
        shifted = errors(best_shift)
        final = samples[-1][1]
        qf, _, _ = self.desired(q0, total)

        d = 180.0 / math.pi
        print()
        print(f'trajectory: {args.cycles} x {args.period:.2f}s raised-cosine, '
              f'amplitude {args.amplitude:.2f} rad, {int(total / args.dt)} points')
        print(f'action result error_code={result.error_code} {result.error_string!r}; '
              f'goal accepted after {(t0 - send_t) * 1000:.1f} ms')
        print(f'/joint_states rate: {rate:.1f} Hz, {len(traj)} samples during motion')
        print(f'best-fit delay: {best_shift * 1000:.0f} ms')
        print(f'{"joint":10s} {"rms err":>9s} {"max err":>9s} {"rms@delay":>10s} {"final":>8s}  (deg)')
        for k, j in enumerate(joints):
            print(f'{j:10s} {rms(raw[k]) * d:9.3f} {max(abs(e) for e in raw[k]) * d:9.3f} '
                  f'{rms(shifted[k]) * d:10.3f} {(final[j] - qf[k]) * d:8.3f}')
        return 0 if result.error_code == 0 else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--action', default='/arm_controller/follow_joint_trajectory')
    ap.add_argument('--joints', nargs='+',
                    default=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
    ap.add_argument('--amplitude', type=float, default=0.3, help='rad (wrist joints use 60%%)')
    ap.add_argument('--center', type=float, nargs='+', help='sweep center (rad), default all zeros')
    ap.add_argument('--yes', action='store_true', help='do not ask for confirmation')
    ap.add_argument('--period', type=float, default=2.0, help='s per sweep')
    ap.add_argument('--cycles', type=int, default=2)
    ap.add_argument('--dt', type=float, default=0.05, help='trajectory point spacing (s)')
    ap.add_argument('--settle', type=float, default=0.5, help='wait after completion (s)')
    args, ros_args = ap.parse_known_args()
    if not 0.0 < args.amplitude <= 1.0:
        ap.error('--amplitude must be in (0, 1.0] rad')

    rclpy.init(args=ros_args)
    node = TrackTest(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        rc = node.run()
    finally:
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
