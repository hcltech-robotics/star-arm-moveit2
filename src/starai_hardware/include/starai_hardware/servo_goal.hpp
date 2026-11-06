// The per-joint control law, kept free of I/O so it can be unit tested.
#pragma once

#include <algorithm>
#include <cmath>

namespace starai_hardware
{

struct JointMap
{
  double scale;       // joint units per servo degree (sign = direction)
  double offset_deg;  // servo angle at joint zero
  double min_pos;     // joint units; NaN = unlimited
  double max_pos;
};

struct ServoGoal
{
  double goal_deg;   // where the servo is aimed
  double speed_dps;  // cruise speed to get there
};

inline double clamp_to_limits(double q, const JointMap & m)
{
  if (!std::isnan(m.min_pos)) q = std::max(q, m.min_pos);
  if (!std::isnan(m.max_pos)) q = std::min(q, m.max_pos);
  return q;
}

// How far (servo degrees) a joint may sit outside its limits and still be activated.
// Commands are clamped, so up to this much is a small nudge back, not a jump.
constexpr double kLimitMarginDeg = 3.0;

inline bool outside_limits(double q, const JointMap & m, double margin_deg)
{
  const double margin = margin_deg * std::abs(m.scale);
  return (!std::isnan(m.min_pos) && q < m.min_pos - margin) ||
         (!std::isnan(m.max_pos) && q > m.max_pos + margin);
}

// Aims the servo `lead` seconds ahead of the setpoint (q, v in joint units). The cruise
// speed is the setpoint speed plus a correction that closes the gap between setpoint and
// measured angle in `correction_time`: a lagging servo speeds up, a leading one slows down.
inline ServoGoal compute_servo_goal(
  double q, double v, double measured_deg, const JointMap & m, double lead,
  double correction_time, double min_speed_dps, double max_speed_dps)
{
  const double q_set = clamp_to_limits(q, m);
  const double set_deg = q_set / m.scale + m.offset_deg;
  const double goal_deg = clamp_to_limits(q_set + v * lead, m) / m.scale + m.offset_deg;
  const double ahead = goal_deg - set_deg;  // feedforward travel during `lead`
  const double gap = set_deg - measured_deg;
  const double speed = ahead == 0.0 ?
    std::abs(gap) / correction_time :
    std::abs(ahead) / lead + (ahead > 0 ? gap : -gap) / correction_time;
  return {goal_deg, std::clamp(speed, min_speed_dps, max_speed_dps)};
}

// One step of the gravity trim (servo degrees). While a joint holds still, the trim moves
// toward the remaining error (setpoint - measured) with time constant `trim_time`, so a
// loaded joint ends where it was commanded instead of where it sags to. Errors within
// `deadband` are left alone: the servos only resolve 0.1 deg and stop short by
// themselves, and integrating that noise would make the joint hunt.
inline double update_trim(
  double trim, double error_deg, double dt, double trim_time, double deadband, double max_trim)
{
  if (trim_time <= 0.0 || std::abs(error_deg) <= deadband) {
    return trim;
  }
  return std::clamp(trim + error_deg * dt / trim_time, -max_trim, max_trim);
}

// Keeps the trimmed setpoint (q in joint units, plus `trim_deg`) inside the joint limits.
// At a limit the goal is clamped anyway, so a servo stopped short there (e.g. by a parking
// stop) would otherwise wind the trim up to its maximum and land that far off after the
// next move.
inline double limit_trim(double trim_deg, double q, const JointMap & m)
{
  return (clamp_to_limits(q + trim_deg * m.scale, m) - q) / m.scale;
}

// Whether to send a joint's goal this cycle. Holding joints are not re-sent: a re-sent
// goal makes a gravity-loaded servo correct its sag and settle back, a visible twitch.
// They are re-sent only when the goal moves, or when the servo has drifted off the goal
// (a lost frame, or the arm was pushed) and `interval_passed` (the resend interval).
// With `retry_once` (grippers) a drifted goal is re-sent only once until the goal moves:
// a gripper holding an object sits short of its goal by design, and re-sending every
// interval would squeeze the object again and again. `retried` is that per-joint state.
inline bool should_send(
  bool goal_moved, double goal_deg, double measured_deg, double hold_tolerance_deg,
  bool interval_passed, bool retry_once, bool & retried)
{
  if (goal_moved) {
    retried = false;
    return true;
  }
  if (!interval_passed || (retry_once && retried) ||
    std::abs(measured_deg - goal_deg) <= hold_tolerance_deg)
  {
    return false;
  }
  retried = true;
  return true;
}

}  // namespace starai_hardware
