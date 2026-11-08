# starai_hardware

`ros2_control` hardware interface for StarAI arms (Cello, Viola) built from Fashion Star
UART bus servos. MoveIt, MoveIt Servo and teleop talk to the standard
`JointTrajectoryController` and `GripperActionController`, and this plugin streams their
setpoints to the servos. An arm whose joints have no command interfaces runs read-only
(the Violin teleop leader).

## Quick start

```bash
# real arm (arm:=cello or arm:=viola)
ros2 launch starai_bringup bringup.launch.py arm:=cello port:=/dev/ttyUSB0

# no arm: simulated servo bus, real driver code path
ros2 run starai_hardware fake_fashionstar_bus.py --link /tmp/ttyFAKE &
ros2 launch starai_bringup bringup.launch.py port:=/tmp/ttyFAKE

# joints only (MoveIt's mock hardware, no driver)
ros2 launch starai_bringup bringup.launch.py use_mock_hardware:=true
```

Measure tracking on your arm (also a good regression check after tuning). It first moves
slowly to the all-zeros "home" pose (`--center` to change), prints the planned sweep and
waits for Enter. There is no collision checking, so keep that area clear:

```bash
ros2 run starai_hardware track_test.py --amplitude 0.3 --period 2.0 --cycles 2
```

Teach mode (torque off, joint states keep publishing; "off" re-engages without a jump):

```bash
ros2 run starai_hardware teach_mode on  [CelloSystem|ViolaSystem]
ros2 run starai_hardware teach_mode off [CelloSystem|ViolaSystem]
```

Robot descriptions build their `<ros2_control>` block from the macros in
[`urdf/fashionstar.ros2_control.xacro`](urdf/fashionstar.ros2_control.xacro)
(`fashionstar_hardware`, `fashionstar_joint`, `fashionstar_gripper`); see
`cello_moveit_config/config/cello_description.ros2_control.xacro`.

## How it works

Every control cycle (100 Hz by default):

* **read()** sends one sync monitor request and collects position, current, voltage,
  temperature and status from every servo. Velocity is estimated from timestamped samples.
  If a servo misses `max_consecutive_misses` replies in a row, read() fails, so the arm
  is stopped and the controllers are deactivated instead of running on stale data.
* **write()** sends one sync "multi-turn angle by velocity" frame. Each servo is aimed at
  `q + v * lead_time` (setpoint plus the controller's velocity feedforward) at the
  commanded speed, plus a correction that closes any setpoint-to-measured gap in
  `correction_time`: a lagging servo speeds up, a leading one slows down. The next frame
  arrives before the servo would start braking, so motion is continuous.
  The lead is 100 ms, not one control period: the servos resolve 0.1 deg, and in the slow
  last part of a move a short lead keeps the goal within one encoder step of the servo,
  so it moves in visible stop-go steps. On the Cello, 30 -> 100 ms cut the mean tracking
  lag from ~1 deg to ~0.2-0.35 deg with no added overshoot.
  Holding joints are not re-sent: a re-sent goal makes a gravity-loaded servo correct
  its sag and settle back, a visible twitch. A holding joint is re-sent only if it drifts
  more than `hold_tolerance_deg` off its goal (a lost frame, or the arm was pushed).
* **Gravity trim:** the servos sag under load (on the Cello, 0.7 deg at the shoulder with
  the arm stretched out: the gripper sat 13 mm low). Once a joint's setpoint has been still
  for 0.3 s, the driver slowly shifts that servo's goal by the remaining error, like the
  integral term of a PID controller, until the reading matches the command. On the Cello
  this cut the settled error to <= 0.2 deg (~2 mm at the gripper) without hunting. The
  trim is kept during the next move and re-converges at the next stop. It never pushes the
  goal past a joint limit, so a joint parked against a stop does not wind it up. Grippers
  are not trimmed. It works from the servo's own encoder, so play in the gearbox after the encoder
  (~0.5 deg on the Cello shoulder) is not corrected.

The legacy driver sent each trajectory point as a "move in N ms" command with
accel = decel = N/2. The servo braked to a stop at every waypoint, which made motion jerky.
On the simulated bus the same trajectory measures:

| | servo accel RMS | time moving (of 4 s) | position RMS |
|---|---|---|---|
| legacy pattern | 2838 deg/s² | 2.6 s (stop-and-go) | 0.67 deg |
| this driver | 420 deg/s² | 3.9 s (continuous) | 0.15 deg |

Other safety behaviour:
* **Stopping the driver (Ctrl-C):** sends only `shutdown_behavior` (hold). The
  `deactivate_behavior` (release, for teach mode) is applied one cycle later, so it never
  reaches the servos during a shutdown.
* **Activation:** holds the current pose. It never drives to a fixed startup pose. It is
  refused if a joint is more than 3 servo degrees outside its limits, since the clamped
  command would snap it back at full speed. Move that joint back by hand first.
* **Configure:** pings every servo and lists missing IDs. It also fixes wrapped
  multi-turn counters, which would otherwise make a joint spin a full turn.
* **Exclusive port lock:** a second driver (e.g. the legacy one) cannot share the bus.
* **Command clamping:** commands are clamped to the `min`/`max` of each position command
  interface.
* **Read-only arms:** if no joint has a command interface (`read_only:=true` in the
  xacro macros), the driver releases torque at configure and never sends motion, also not
  on stop, shutdown or error. Positions are still published.
* **Fault logging:** stall, voltage, current, power and temperature faults are logged
  when they appear and when they clear. A flickering fault is logged at most every 2 s.

## Parameters

Hardware (`<hardware><param>`):

| name | default | meaning |
|---|---|---|
| `port` | `/dev/ttyUSB0` | serial device |
| `baudrate` | `1000000` | bus baud rate |
| `lead_time` | `0.1` | s the servo is aimed ahead. Raise if slow motion stutters, lower if it overshoots |
| `correction_time` | `0.1` | s to close a gap between setpoint and measured angle (0.2 made slow moves rougher on the Cello) |
| `trim_time` | `1.0` | s, gravity trim time constant; `0` turns it off |
| `trim_deadband_deg`, `trim_max_deg` | `0.2`, `3.0` | errors within the deadband are not trimmed; trim limit per joint |
| `t_acc_ms` / `t_dec_ms` | `20` | servo accel/decel ramp per command |
| `min_speed_dps` / `max_speed_dps` | `2` / `360` | cruise speed clamp (deg/s) |
| `power_mw` / `hold_power_mw` | `8000` | power limit sent with motion / hold commands (hold never exceeds a joint's `power_mw`) |
| `read_timeout_ms` | `8` | per-cycle wait for monitor replies |
| `max_consecutive_misses` | `5` | missed replies before the hardware errors out (also per joint) |
| `velocity_filter_hz` | `15` | low-pass cutoff of the velocity estimate |
| `goal_deadband_deg` | `0.05` | goal change that counts as a new goal |
| `hold_tolerance_deg`, `resend_s` | `1.5`, `1.0` | re-send a holding joint that drifted this far off its goal, at most this often |
| `reset_multi_turn` | `true` | fix wrapped multi-turn counters at configure |
| `deactivate_behavior` | `release` | `hold`, `release`, `damping` or `none` |
| `shutdown_behavior` | `hold` | same options as above |
| `error_behavior` | `hold` | same options as above |

Joint (`<joint><param>`): `id` (required), `scale` (joint units per servo degree,
default π/180, negative to invert), `offset_deg` (servo angle at joint zero),
`max_speed_dps`, `power_mw`, `max_consecutive_misses`, `feedforward` (`false` for step-commanded joints such as grippers),
`grip_squeeze` (grippers, joint units: hold this far past where the fingers were blocked;
0 = right there; default off).

State interfaces: `position`, `velocity`, and optionally `current` (A), `voltage` (V),
`temperature` (°C), `status` (raw bits). Command interfaces: `position`, optional `velocity`.

## Tuning on the real arm

1. **USB latency:** this dominates the cycle time. FTDI adapters buffer 16 ms by default.
   The driver requests low-latency mode. If it warns that it couldn't, run:
   `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`.
   The driver also warns when a bus read takes more than 60% of the cycle.
2. **Stable device name:** add a udev rule, e.g.
   `SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="cello_arm"`, then use `port:=/dev/cello_arm`.
3. **Measure, then tune:** run `track_test.py`. A large "best-fit delay" means
   `lead_time` should increase. Ringing or overshoot at the end means it should decrease.
   Then try `update_rate: 150` in `ros2_controllers.yaml`.
4. **Grasping:** a gripper that stalls on an object would push toward its "closed" goal
   at its power limit for as long as it holds. On the Cello that flickered OVER_POWER
   every cycle (at 2, 1.5 and 1 W alike: the flag means "at the limit"), the supply
   dipped, and the wrist servos dropped replies until the driver stopped the arm. The
   jiggling fingers also kept the gripper controller from ever reporting the grasp done.
   So once the fingers have been stopped for 0.2 s short of the goal, the driver holds
   them `grip_squeeze` past that point. The Cello uses 0 (hold where blocked): the object
   stays pressed as it was when the fingers stopped. 1 mm past contact already pushed at
   the full 1 W. The gripper controller sees a stalled grasp, so MoveIt and
   GripperCommand work as before. A new command (e.g. open) clears it. Closing again
   without opening does not re-grip (the command is unchanged): open first.

## Limits

These servos are position-controlled hobby actuators with 0.1° resolution, gear
backlash and no torque control, on a 1 Mbaud half-duplex bus. This driver removes the
software causes of sloppiness (stop-and-go streaming, open-loop timing, no feedback or
fault handling, slow polling). It cannot add torque control or kHz-rate loops, and it cannot
see gearbox backlash past the servo's encoder.
