#!/usr/bin/env python3
"""Simulated Fashion Star servo bus on a pseudo-terminal.

Lets you run the real starai_hardware plugin (and anything else that speaks the
protocol) without an arm. The servos follow the motion commands with acceleration
limits, a little response delay, and a position deadband. Each reply is delayed by
the transmission time it would take at the given baud rate.

    ros2 run starai_hardware fake_fashionstar_bus.py --link /tmp/ttyFAKE
    ros2 launch cello_moveit_config bringup.launch.py port:=/tmp/ttyFAKE

Fault injection:
    --drop-id 3 --drop-after 10   servo 3 stops answering after 10 s
    --wrap-id 4                   servo 4 starts with its multi-turn counter one turn off
    --block-id 6 --block-at 40    servo 6 cannot go below 40 deg (gripper closed on an object)
"""
import argparse
import math
import os
import signal
import struct
import sys
import threading
import time
import tty

REQ_HEADER = b'\x12\x4c'
RESP_HEADER = b'\x05\x1c'


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def frame(cmd: int, payload: bytes) -> bytes:
    body = RESP_HEADER + bytes([cmd, len(payload)]) + payload
    return body + bytes([checksum(body)])


class Servo:
    def __init__(self, sid, pos_deg, args):
        self.id = sid
        self.pos = pos_deg   # true shaft angle (deg)
        self.vel = 0.0       # deg/s
        self.torque = False
        self.target = pos_deg
        self.speed = 0.0
        self.acc = 1000.0    # deg/s^2
        self.dec = 1000.0
        self.turn_offset = 0
        self.args = args
        self.pending = []    # (apply_time, fn) - models the servo's command latency

    def command_move(self, target, speed, t_acc_ms, t_dec_ms):
        speed = max(speed, 0.1)
        self.target = target
        self.speed = min(speed, self.args.max_speed)
        self.acc = self.speed / max(t_acc_ms, 1) * 1000.0
        self.dec = self.speed / max(t_dec_ms, 1) * 1000.0
        self.torque = True

    def step(self, dt):
        if not self.torque:
            self.vel = 0.0
            return
        err = self.target - self.pos
        if abs(err) < self.args.deadband:
            v_des = 0.0
        else:
            v_des = math.copysign(min(self.speed, math.sqrt(2.0 * self.dec * abs(err))), err)
        dv = v_des - self.vel
        lim = (self.acc if abs(v_des) > abs(self.vel) else self.dec) * dt
        self.vel += max(-lim, min(lim, dv))
        new = self.pos + self.vel * dt
        # do not cross the target because of the discrete step
        if (self.target - self.pos) * (self.target - new) < 0:
            new, self.vel = self.target, 0.0
        if self.args.block_id == self.id and new < self.args.block_at:
            new, self.vel = self.args.block_at, 0.0  # pressing on an obstacle
        self.pos = new

    def reported(self):
        return self.pos + 360.0 * self.turn_offset


class Bus:
    def __init__(self, args):
        self.args = args
        init = args.initial or [0.0] * len(args.ids)
        if len(init) != len(args.ids):
            sys.exit('--initial needs one value per id')
        self.servos = {sid: Servo(sid, p, args) for sid, p in zip(args.ids, init)}
        if args.wrap_id is not None:
            self.servos[args.wrap_id].turn_offset = 1
        self.lock = threading.Lock()
        self.stats = dict(frames=0, bad_checksum=0, monitor=0, sync_vel=0, sync_int=0,
                          stop=0, reset=0, ping=0, unknown=0)
        self.moves = {}  # motion commands received per servo id
        self.t0 = time.monotonic()
        self.byte_time = 10.0 / args.baud

    def alive(self, sid):
        if sid not in self.servos:
            return False
        if self.args.drop_id == sid and time.monotonic() - self.t0 > self.args.drop_after:
            return False
        return True

    # ---- request handling -------------------------------------------------
    def handle(self, cmd, p, out):
        s = self.stats
        if cmd == 1:  # ping
            s['ping'] += 1
            if self.alive(p[0]):
                out.append(frame(1, bytes([p[0]])))
        elif cmd == 22:  # single monitor
            s['monitor'] += 1
            self.monitor_reply(p[0], out)
        elif cmd == 24:  # stop on control
            s['stop'] += 1
            sid, mode, _power = struct.unpack('<BBH', p)
            for sv in self.targets(sid):
                if mode == 0x11:
                    sv.command_move(sv.pos, 1.0, 20, 20)
                    sv.vel = 0.0
                else:  # release / damping
                    sv.torque = False
        elif cmd == 17:  # reset multi-turn
            s['reset'] += 1
            for sv in self.targets(p[0]):
                sv.turn_offset = 0
                sv.pos = (sv.pos + 180.0) % 360.0 - 180.0
        elif cmd == 7:  # wheel mode stop (vendor SDK's disable_torque)
            for sv in self.targets(p[0]):
                sv.torque = False
        elif cmd == 15:  # single velocity move
            sid, ang, spd, ta, td, _pw = struct.unpack('<BiHHHH', p)
            self.apply_velocity(sid, ang, spd, ta, td)
        elif cmd == 14:
            sid, ang, itv, ta, td, _pw = struct.unpack('<BiIHHH', p)
            self.apply_interval(sid, ang, itv, ta, td)
        elif cmd == 25:  # sync
            sub, per, count = p[0], p[1], p[2]
            body = p[3:]
            if len(body) != per * count:
                s['unknown'] += 1
                print(f'[fake_bus] malformed sync frame sub={sub} per={per} count={count} '
                      f'len={len(body)}', file=sys.stderr)
                return
            for k in range(count):
                e = body[k * per:(k + 1) * per]
                if sub == 22:
                    self.monitor_reply(e[0], out)
                elif sub == 15:
                    sid, ang, spd, ta, td, _pw = struct.unpack('<BiHHHH', e)
                    self.apply_velocity(sid, ang, spd, ta, td)
                elif sub == 14:
                    sid, ang, itv, ta, td, _pw = struct.unpack('<BiIHHH', e)
                    self.apply_interval(sid, ang, itv, ta, td)
                elif sub == 13:
                    sid, ang, itv, _pw = struct.unpack('<BiIH', e)
                    self.apply_interval(sid, ang, itv, 20, 20)
                else:
                    s['unknown'] += 1
            if sub == 22:
                s['monitor'] += 1
            elif sub == 15:
                s['sync_vel'] += 1
            elif sub in (13, 14):
                s['sync_int'] += 1
        else:
            s['unknown'] += 1
            print(f'[fake_bus] unhandled command {cmd}', file=sys.stderr)

    def targets(self, sid):
        if sid == 0xFF:
            return list(self.servos.values())
        return [self.servos[sid]] if self.alive(sid) else []

    def later(self, sv, fn):
        sv.pending.append((time.monotonic() + self.args.latency, fn))

    def apply_velocity(self, sid, ang, spd, ta, td):
        self.moves[sid] = self.moves.get(sid, 0) + 1
        for sv in self.targets(sid):
            target = ang / 10.0 - 360.0 * sv.turn_offset
            self.later(sv, lambda sv=sv: sv.command_move(target, spd / 10.0, ta, td))

    def apply_interval(self, sid, ang, itv, ta, td):
        for sv in self.targets(sid):
            target = ang / 10.0 - 360.0 * sv.turn_offset
            cruise_t = max(itv - 0.5 * (ta + td), 1.0) / 1000.0
            speed = abs(target - sv.pos) / cruise_t
            self.later(sv, lambda sv=sv: sv.command_move(target, speed, ta, td))

    def monitor_reply(self, sid, out):
        if not self.alive(sid):
            return
        sv = self.servos[sid]
        angle = int(round(sv.reported() * 10.0))
        turns = sv.turn_offset
        current = int(min(abs(sv.vel) * 2.0 + 120, 3000))
        payload = struct.pack('<BHHHHBih', sid, 12000, current, current * 12, 2048,
                              1 if abs(sv.vel) > 0.1 else 0, angle, turns)
        out.append(frame(22, payload))

    # ---- simulation thread --------------------------------------------------
    def sim_loop(self, stop):
        dt = 0.001
        nxt = time.monotonic()
        trace = open(self.args.trace, 'w') if self.args.trace else None
        while not stop.is_set():
            now = time.monotonic()
            with self.lock:
                for sv in self.servos.values():
                    due = [fn for t, fn in sv.pending if t <= now]
                    sv.pending = [(t, fn) for t, fn in sv.pending if t > now]
                    for fn in due:
                        fn()
                    sv.step(dt)
                    if trace:
                        trace.write(f'{now - self.t0:.4f} {sv.id} {sv.pos:.4f} {sv.vel:.3f}\n')
            nxt += dt
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()


def serve(args):
    master, slave = os.openpty()
    tty.setraw(slave)
    slave_name = os.ttyname(slave)
    if args.link:
        try:
            os.unlink(args.link)
        except FileNotFoundError:
            pass
        os.symlink(slave_name, args.link)
    print(f'[fake_bus] {len(args.ids)} servos {args.ids} on {args.link or slave_name}', flush=True)

    bus = Bus(args)
    stop = threading.Event()
    threading.Thread(target=bus.sim_loop, args=(stop,), daemon=True).start()

    def shutdown(*_):
        stop.set()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    buf = bytearray()
    last_report = time.monotonic()
    while not stop.is_set():
        try:
            import select
            r, _, _ = select.select([master], [], [], 0.2)
            if not r:
                continue
            buf += os.read(master, 4096)
        except OSError:
            time.sleep(0.01)
            continue
        while True:
            i = buf.find(REQ_HEADER)
            if i < 0:
                del buf[:-1]
                break
            del buf[:i]
            if len(buf) < 4:
                break
            n = buf[3]
            if len(buf) < 5 + n:
                break
            pkt = bytes(buf[:5 + n])
            del buf[:5 + n]
            if checksum(pkt[:-1]) != pkt[-1]:
                bus.stats['bad_checksum'] += 1
                print(f'[fake_bus] BAD CHECKSUM {pkt.hex()}', file=sys.stderr)
                continue
            bus.stats['frames'] += 1
            out = []
            with bus.lock:
                bus.handle(pkt[2], pkt[4:-1], out)
            if out:
                data = b''.join(out)
                time.sleep(args.reply_delay + len(data) * bus.byte_time)
                os.write(master, data)
        if args.report and time.monotonic() - last_report > args.report:
            last_report = time.monotonic()
            with bus.lock:
                pos = ' '.join(f'{s.id}:{s.reported():7.2f}' for s in bus.servos.values())
            print(f'[fake_bus] {bus.stats} moves {bus.moves} | {pos}', flush=True)

    print(f'[fake_bus] final stats {bus.stats}', flush=True)
    if args.link:
        try:
            os.unlink(args.link)
        except FileNotFoundError:
            pass
    return 0 if bus.stats['bad_checksum'] == 0 and bus.stats['unknown'] == 0 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--link', default='/tmp/ttyFAKE', help='symlink to create for the pty')
    ap.add_argument('--ids', type=int, nargs='+', default=[0, 1, 2, 3, 4, 5, 6])
    ap.add_argument('--initial', type=float, nargs='+', help='initial angles (deg), one per id')
    ap.add_argument('--baud', type=int, default=1000000)
    ap.add_argument('--max-speed', type=float, default=400.0, help='servo speed limit (deg/s)')
    ap.add_argument('--deadband', type=float, default=0.1, help='position deadband (deg)')
    ap.add_argument('--latency', type=float, default=0.002, help='command-to-motion delay (s)')
    ap.add_argument('--reply-delay', type=float, default=0.0003, help='turnaround per reply (s)')
    ap.add_argument('--drop-id', type=int)
    ap.add_argument('--drop-after', type=float, default=10.0)
    ap.add_argument('--wrap-id', type=int)
    ap.add_argument('--block-id', type=int)
    ap.add_argument('--block-at', type=float, default=0.0)
    ap.add_argument('--report', type=float, default=0.0, help='print stats every N seconds')
    ap.add_argument('--trace', help='write "t id pos vel" lines at 1 kHz to this file')
    args = ap.parse_args(sys.argv[1:] if '--ros-args' not in sys.argv
                         else sys.argv[1:sys.argv.index('--ros-args')])
    sys.exit(serve(args))


if __name__ == '__main__':
    main()
