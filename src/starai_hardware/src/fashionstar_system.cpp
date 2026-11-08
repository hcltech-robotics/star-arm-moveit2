#include "starai_hardware/fashionstar_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "starai_hardware/servo_goal.hpp"

namespace starai_hardware
{
namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kTrimSettleS = 0.3;  // s the setpoint must be still before trimming

const std::string * find_param(
  const std::unordered_map<std::string, std::string> & m, const std::string & key)
{
  auto it = m.find(key);
  return it == m.end() ? nullptr : &it->second;
}

bool parse_double(const std::string & s, double & out)
{
  try {
    size_t idx = 0;
    out = std::stod(s, &idx);
    return idx == s.size() || s.find_first_not_of(" \t", idx) == std::string::npos;
  } catch (...) {
    return false;
  }
}

template<typename T>
bool get_number(
  const std::unordered_map<std::string, std::string> & m, const std::string & key, T & out,
  std::string & error, double lo = -std::numeric_limits<double>::infinity(),
  double hi = std::numeric_limits<double>::infinity())
{
  const auto * s = find_param(m, key);
  if (!s) {
    return true;  // keep default
  }
  double v;
  if (!parse_double(*s, v) || !std::isfinite(v) || v < lo || v > hi) {
    error = "invalid value '" + *s + "' for parameter '" + key + "'";
    return false;
  }
  out = static_cast<T>(v);
  return true;
}

bool parse_behavior(const std::string & s, int & out)
{
  if (s == "hold") { out = 0; return true; }
  if (s == "release") { out = 1; return true; }
  if (s == "damping") { out = 2; return true; }
  if (s == "none") { out = 3; return true; }
  return false;
}

std::string describe_status(uint8_t s)
{
  std::ostringstream os;
  if (s & fsus::kStatusStall) os << " STALL";
  if (s & fsus::kStatusOverVoltage) os << " OVER_VOLTAGE";
  if (s & fsus::kStatusUnderVoltage) os << " UNDER_VOLTAGE";
  if (s & fsus::kStatusOverCurrent) os << " OVER_CURRENT";
  if (s & fsus::kStatusOverPower) os << " OVER_POWER";
  if (s & fsus::kStatusOverTemperature) os << " OVER_TEMPERATURE";
  return os.str();
}
}  // namespace

FashionStarSystem::~FashionStarSystem()
{
  // Best effort if the component is destroyed without a lifecycle shutdown.
  if (bus_.is_open()) {
    stop_all(params_.shutdown_behavior);
    bus_.close();
  }
}

#if STARAI_HW_HAS_INTERFACE_PARAMS
FashionStarSystem::CallbackReturn FashionStarSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return init_from_info();
}
#else
FashionStarSystem::CallbackReturn FashionStarSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return init_from_info();
}
#endif

FashionStarSystem::CallbackReturn FashionStarSystem::init_from_info()
{
  logger_ = rclcpp::get_logger("starai_hardware." + info_.name);
  std::string error;
  if (!parse_params(error)) {
    RCLCPP_FATAL(logger_, "%s", error.c_str());
    return CallbackReturn::ERROR;
  }

  const size_t n = joints_.size();
  ids_.resize(n);
  for (size_t i = 0; i < n; ++i) {
    ids_[i] = joints_[i].id;
  }
  monitor_.resize(n);
  monitor_valid_.resize(n);
  velocity_targets_.reserve(n);
  target_joint_.reserve(n);

  std::ostringstream os;
  for (const auto & j : joints_) {
    os << "\n  " << j.name << ": id=" << int(j.id) << " scale=" << j.scale
       << " offset_deg=" << j.offset_deg << " limits=[" << j.min_pos << ", " << j.max_pos
       << "] max_speed=" << j.max_speed_dps << "deg/s power=" << j.power_mw << "mW"
       << (j.feedforward ? "" : " (no feedforward)");
  }
  RCLCPP_INFO(
    logger_, "Fashion Star bus on %s @ %d baud, lead=%.0fms%s", params_.port.c_str(),
    params_.baudrate, params_.lead_time * 1000.0, os.str().c_str());
  return CallbackReturn::SUCCESS;
}

bool FashionStarSystem::parse_params(std::string & error)
{
  const auto & hp = info_.hardware_parameters;
  Params p;
  if (const auto * s = find_param(hp, "port")) p.port = *s;
  if (!get_number(hp, "baudrate", p.baudrate, error, 9600, 4000000) ||
    !get_number(hp, "lead_time", p.lead_time, error, 0.005, 0.5) ||
    !get_number(hp, "correction_time", p.correction_time, error, 0.005, 5.0) ||
    !get_number(hp, "trim_time", p.trim_time, error, 0.0, 60.0) ||
    !get_number(hp, "trim_deadband_deg", p.trim_deadband_deg, error, 0.0, 5.0) ||
    !get_number(hp, "trim_max_deg", p.trim_max_deg, error, 0.0, 20.0) ||
    !get_number(hp, "min_speed_dps", p.min_speed_dps, error, 0.1, 6000) ||
    !get_number(hp, "max_speed_dps", p.max_speed_dps, error, 1, 6000) ||
    !get_number(hp, "t_acc_ms", p.t_acc_ms, error, 0, 10000) ||
    !get_number(hp, "t_dec_ms", p.t_dec_ms, error, 0, 10000) ||
    !get_number(hp, "power_mw", p.power_mw, error, 0, 65535) ||
    !get_number(hp, "hold_power_mw", p.hold_power_mw, error, 0, 65535) ||
    !get_number(hp, "read_timeout_ms", p.read_timeout_ms, error, 1, 100) ||
    !get_number(hp, "max_consecutive_misses", p.max_consecutive_misses, error, 1, 1000) ||
    !get_number(hp, "velocity_filter_hz", p.velocity_filter_hz, error, 0.1, 1000) ||
    !get_number(hp, "goal_deadband_deg", p.goal_deadband_deg, error, 0, 10) ||
    !get_number(hp, "hold_tolerance_deg", p.hold_tolerance_deg, error, 0, 90) ||
    !get_number(hp, "resend_s", p.resend_s, error, 0.01, 60))
  {
    return false;
  }
  if (const auto * s = find_param(hp, "reset_multi_turn")) {
    p.reset_multi_turn = (*s == "true" || *s == "True" || *s == "1");
  }
  for (auto [key, target] : {
      std::pair<const char *, StopBehavior *>{"deactivate_behavior", &p.deactivate_behavior},
      {"shutdown_behavior", &p.shutdown_behavior},
      {"error_behavior", &p.error_behavior}})
  {
    if (const auto * s = find_param(hp, key)) {
      int b;
      if (!parse_behavior(*s, b)) {
        error = std::string(key) + " must be hold|release|damping|none, got '" + *s + "'";
        return false;
      }
      *target = static_cast<StopBehavior>(b);
    }
  }
  // StopBehavior enum order is kHold, kRelease, kDamping, kNone (matches parse_behavior).

  if (info_.joints.empty()) {
    error = "no joints defined in <ros2_control>";
    return false;
  }
  joints_.clear();
  has_velocity_command_ = false;
  bool any_velocity_cmd = false;
  bool all_velocity_cmd = true;
  for (const auto & ji : info_.joints) {
    Joint j;
    j.name = ji.name;
    const auto & jp = ji.parameters;
    double id = -1;
    if (!find_param(jp, "id")) {
      error = "joint '" + ji.name + "' is missing <param name=\"id\">";
      return false;
    }
    if (!get_number(jp, "id", id, error, 0, 253)) {
      return false;
    }
    j.id = static_cast<uint8_t>(id);
    j.scale = kDegToRad;
    j.max_speed_dps = p.max_speed_dps;
    j.power_mw = p.power_mw;
    j.max_misses = p.max_consecutive_misses;
    if (!get_number(jp, "scale", j.scale, error) ||
      !get_number(jp, "offset_deg", j.offset_deg, error) ||
      !get_number(jp, "max_speed_dps", j.max_speed_dps, error, 1, 6000) ||
      !get_number(jp, "power_mw", j.power_mw, error, 0, 65535) ||
      !get_number(jp, "max_consecutive_misses", j.max_misses, error, 1, 1000))
    {
      error = "joint '" + ji.name + "': " + error;
      return false;
    }
    if (const auto * s = find_param(jp, "feedforward")) {
      j.feedforward = !(*s == "false" || *s == "False" || *s == "0");
    }
    if (j.scale == 0.0) {
      error = "joint '" + ji.name + "': scale must be non-zero";
      return false;
    }

    bool has_pos_cmd = false;
    bool has_vel_cmd = false;
    j.min_pos = kNaN;
    j.max_pos = kNaN;
    for (const auto & ci : ji.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        has_pos_cmd = true;
        double v;
        if (!ci.min.empty() && parse_double(ci.min, v)) j.min_pos = v;
        if (!ci.max.empty() && parse_double(ci.max, v)) j.max_pos = v;
      } else if (ci.name == hardware_interface::HW_IF_VELOCITY) {
        has_vel_cmd = true;
      } else {
        error = "joint '" + ji.name + "': unsupported command interface '" + ci.name + "'";
        return false;
      }
    }
    if (!has_pos_cmd) {
      error = "joint '" + ji.name + "' needs a 'position' command interface";
      return false;
    }
    any_velocity_cmd |= has_vel_cmd;
    all_velocity_cmd &= has_vel_cmd;

    bool has_pos_state = false;
    for (const auto & si : ji.state_interfaces) {
      static const std::vector<std::string> kSupported = {
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY, "current",
        "voltage", "temperature", "status"};
      if (std::find(kSupported.begin(), kSupported.end(), si.name) == kSupported.end()) {
        error = "joint '" + ji.name + "': unsupported state interface '" + si.name + "'";
        return false;
      }
      has_pos_state |= si.name == hardware_interface::HW_IF_POSITION;
    }
    if (!has_pos_state) {
      error = "joint '" + ji.name + "' needs a 'position' state interface";
      return false;
    }
    for (const auto & other : joints_) {
      if (other.id == j.id) {
        error = "joints '" + other.name + "' and '" + ji.name + "' share servo id " +
          std::to_string(j.id);
        return false;
      }
    }
    joints_.push_back(j);
  }
  if (any_velocity_cmd && !all_velocity_cmd) {
    error = "either all joints or none must have a 'velocity' command interface";
    return false;
  }
  has_velocity_command_ = all_velocity_cmd;

  params_ = p;
  return true;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
std::vector<hardware_interface::StateInterface> FashionStarSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < joints_.size(); ++i) {
    auto & j = joints_[i];
    for (const auto & si : info_.joints[i].state_interfaces) {
      double * ptr = nullptr;
      if (si.name == hardware_interface::HW_IF_POSITION) ptr = &j.pos;
      else if (si.name == hardware_interface::HW_IF_VELOCITY) ptr = &j.vel;
      else if (si.name == "current") ptr = &j.current;
      else if (si.name == "voltage") ptr = &j.voltage;
      else if (si.name == "temperature") ptr = &j.temperature;
      else if (si.name == "status") ptr = &j.status;
      if (ptr) {
        out.emplace_back(j.name, si.name, ptr);
      }
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> FashionStarSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.cmd_pos);
    if (has_velocity_command_) {
      out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.cmd_vel);
    }
  }
  return out;
}
#pragma GCC diagnostic pop

FashionStarSystem::CallbackReturn FashionStarSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::string error;
  if (!bus_.open(params_.port, params_.baudrate, error)) {
    RCLCPP_ERROR(logger_, "%s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  for (const auto & w : bus_.open_warnings()) {
    RCLCPP_WARN(logger_, "%s", w.c_str());
  }

  std::vector<std::string> missing;
  for (const auto & j : joints_) {
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
      ok = bus_.ping(j.id, std::chrono::milliseconds(20));
    }
    if (!ok) {
      missing.push_back(j.name + " (id " + std::to_string(j.id) + ")");
    }
  }
  if (!missing.empty()) {
    std::string list;
    for (const auto & m : missing) list += "\n  " + m;
    RCLCPP_ERROR(
      logger_, "No reply from servos:%s\nCheck power, cabling, ids and baud rate.", list.c_str());
    bus_.close();
    return CallbackReturn::FAILURE;
  }

  if (!read_all(5)) {
    RCLCPP_ERROR(logger_, "Servos answer pings but not monitor queries");
    bus_.close();
    return CallbackReturn::FAILURE;
  }

  // Multi-turn counters survive power cycles; a servo that was spun past +-180 deg would
  // report an angle a full turn away from the real joint position and be driven there.
  if (params_.reset_multi_turn) {
    bool reset_any = false;
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto & m = monitor_[i];
      // `turns` is the encoder's internal wrap count and is often non-zero for valid
      // angles (e.g. -0.9 deg with turns=-1); only the reported angle matters.
      if (std::abs(m.angle_deg()) > 180.0) {
        RCLCPP_WARN(
          logger_, "%s: multi-turn angle %.1f deg (turns=%d), resetting counter "
          "(servo torque is released briefly)", joints_[i].name.c_str(), m.angle_deg(), m.turns);
        bus_.stop(joints_[i].id, fsus::StopMode::kRelease, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        bus_.reset_multi_turn(joints_[i].id);
        reset_any = true;
      }
    }
    if (reset_any) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (!read_all(5)) {
        RCLCPP_ERROR(logger_, "Lost servos after multi-turn reset");
        bus_.close();
        return CallbackReturn::FAILURE;
      }
      for (size_t i = 0; i < joints_.size(); ++i) {
        if (std::abs(monitor_[i].angle_deg()) > 180.0) {
          RCLCPP_ERROR(
            logger_, "%s still reports %.1f deg after the multi-turn reset; refusing to start",
            joints_[i].name.c_str(), monitor_[i].angle_deg());
          bus_.close();
          return CallbackReturn::FAILURE;
        }
      }
    }
  }

  for (auto & j : joints_) {
    if (outside_limits(j.pos, JointMap{j.scale, j.offset_deg, j.min_pos, j.max_pos},
        kLimitMarginDeg))
    {
      RCLCPP_WARN(
        logger_, "%s is at %.3f, outside its limits [%.3f, %.3f]. Activation will be refused "
        "until it is moved back by hand; if it is not out of range, check the zero calibration.",
        j.name.c_str(), j.pos, j.min_pos, j.max_pos);
    }
    // NaN = "no command yet": controllers that interpolate from the last command fall back
    // to the measured state on activation.
    j.cmd_pos = kNaN;
    j.cmd_vel = kNaN;
  }
  RCLCPP_INFO(logger_, "Configured: %zu servos online", joints_.size());
  return CallbackReturn::SUCCESS;
}

FashionStarSystem::CallbackReturn FashionStarSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  pending_stop_ = StopBehavior::kNone;
  bus_.close();
  return CallbackReturn::SUCCESS;
}

FashionStarSystem::CallbackReturn FashionStarSystem::on_activate(const rclcpp_lifecycle::State &)
{
  pending_stop_ = StopBehavior::kNone;
  if (!read_all(5)) {
    RCLCPP_ERROR(logger_, "Cannot read servo positions; refusing to activate");
    return CallbackReturn::FAILURE;
  }
  // Commands are clamped to the limits, so a joint far outside them would snap back at
  // full speed on the first cycle. Make the user move it back by hand instead.
  for (const auto & j : joints_) {
    if (outside_limits(j.pos, JointMap{j.scale, j.offset_deg, j.min_pos, j.max_pos},
        kLimitMarginDeg))
    {
      RCLCPP_ERROR(
        logger_, "%s is at %.3f, outside its limits [%.3f, %.3f]; refusing to activate. "
        "Move it back within its limits by hand (torque is off), then activate again.",
        j.name.c_str(), j.pos, j.min_pos, j.max_pos);
      return CallbackReturn::FAILURE;
    }
  }
  // Engage torque exactly where the arm is: no startup jump.
  velocity_targets_.clear();
  const auto now = Clock::now();
  for (auto & j : joints_) {
    j.cmd_pos = j.pos;
    j.cmd_vel = kNaN;  // set by controllers that output velocity
    j.have_prev_cmd = false;
    j.misses = 0;
    j.trim_deg = 0.0;
    j.still_s = 0.0;
    j.drift_resent = false;
    j.last_goal_deg = j.servo_deg;
    j.last_send = now;
    j.sent_once = true;
    const int32_t goal = static_cast<int32_t>(std::lround(j.servo_deg * 10.0));
    velocity_targets_.push_back(
      {j.id, goal, static_cast<uint16_t>(params_.min_speed_dps * 10.0), params_.t_acc_ms,
        params_.t_dec_ms, j.power_mw});
  }
  if (!bus_.write_sync_velocity(velocity_targets_)) {
    RCLCPP_ERROR(logger_, "Failed to send hold command");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(logger_, "Activated: torque on, holding current pose");
  return CallbackReturn::SUCCESS;
}

FashionStarSystem::CallbackReturn FashionStarSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Applied on the next read() instead of here: on Ctrl-C, ros2_control deactivates and
  // then shuts down in the same instant, and only the shutdown behavior should reach the
  // servos (otherwise the default "release" would drop the arm before "hold" arrives).
  // read() keeps running while inactive, so teach mode still releases ~1 cycle later.
  pending_stop_ = params_.deactivate_behavior;
  for (auto & j : joints_) {
    j.cmd_pos = kNaN;
    j.cmd_vel = kNaN;
  }
  RCLCPP_INFO(logger_, "Deactivated");
  return CallbackReturn::SUCCESS;
}

FashionStarSystem::CallbackReturn FashionStarSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  pending_stop_ = StopBehavior::kNone;
  if (bus_.is_open()) {
    stop_all(params_.shutdown_behavior);
    bus_.close();
  }
  return CallbackReturn::SUCCESS;
}

FashionStarSystem::CallbackReturn FashionStarSystem::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(logger_, "Hardware error; stopping servos");
  pending_stop_ = StopBehavior::kNone;
  if (bus_.is_open()) {
    stop_all(params_.error_behavior);
    bus_.close();
  }
  return CallbackReturn::SUCCESS;
}

void FashionStarSystem::stop_all(StopBehavior behavior)
{
  if (!bus_.is_open() || behavior == StopBehavior::kNone) {
    return;
  }
  fsus::StopMode mode = fsus::StopMode::kHold;
  if (behavior == StopBehavior::kRelease) mode = fsus::StopMode::kRelease;
  if (behavior == StopBehavior::kDamping) mode = fsus::StopMode::kDamping;
  // Repeat: these frames are unacknowledged and this is the last word we send.
  for (int r = 0; r < 2; ++r) {
    for (const auto & j : joints_) {
      // Never hold harder than the joint's own limit: a gripper holding an object
      // at the 8 W default heats up fast (48 C in 8 minutes on the Cello).
      bus_.stop(j.id, mode, std::min(params_.hold_power_mw, j.power_mw));
    }
  }
}

bool FashionStarSystem::read_all(int attempts)
{
  const auto timeout = std::chrono::microseconds(static_cast<int64_t>(params_.read_timeout_ms * 1000));
  for (int a = 0; a < attempts; ++a) {
    // Allow longer than the control-loop timeout here; this runs outside the RT loop.
    const size_t got = bus_.read_monitor(ids_, monitor_, monitor_valid_, timeout * 4);
    const auto stamp = Clock::now();
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (monitor_valid_[i]) {
        joints_[i].have_sample = false;  // do not differentiate across this gap
        apply_sample(joints_[i], monitor_[i], stamp);
        joints_[i].vel = 0.0;
      }
    }
    if (got == joints_.size()) {
      return true;
    }
  }
  return false;
}

void FashionStarSystem::apply_sample(Joint & j, const fsus::MonitorData & m, Clock::time_point stamp)
{
  const double deg = m.angle_deg();
  if (j.have_sample) {
    const double dt = std::chrono::duration<double>(stamp - j.stamp).count();
    if (dt > 1e-4) {
      const double raw_vel = j.to_joint(deg) - j.to_joint(j.servo_deg);
      const double rc = 1.0 / (2.0 * M_PI * params_.velocity_filter_hz);
      const double alpha = dt / (rc + dt);
      j.vel += alpha * (raw_vel / dt - j.vel);
    }
  }
  j.servo_deg = deg;
  j.stamp = stamp;
  j.have_sample = true;
  j.pos = j.to_joint(deg);
  j.current = m.current_ma / 1000.0;
  j.voltage = m.voltage_mv / 1000.0;
  j.temperature = m.temperature_c();
  j.status = m.status;
  log_status_change(j, m.status);
}

void FashionStarSystem::log_status_change(Joint & j, uint8_t status)
{
  // A flag can flicker every cycle (a gripper squeezing an object toggles OVER_POWER), so
  // repeats and clears are logged at most every 2 s per servo. A new kind of fault is
  // always logged at once.
  const uint8_t faults = status & fsus::kStatusFaultMask;
  const auto now = Clock::now();
  const bool new_fault = faults & ~j.logged_faults;
  if (faults == j.logged_faults ||
    (!new_fault && now - j.last_fault_log < std::chrono::seconds(2)))
  {
    return;
  }
  j.logged_faults = faults;
  j.last_fault_log = now;
  if (faults) {
    RCLCPP_WARN(
      logger_, "%s (id %d) reports:%s (temp %.1fC, %.2fA, %.2fV)", j.name.c_str(), j.id,
      describe_status(faults).c_str(), j.temperature, j.current, j.voltage);
  } else {
    RCLCPP_INFO(logger_, "%s (id %d) fault cleared", j.name.c_str(), j.id);
  }
}

FashionStarSystem::return_type FashionStarSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!bus_.is_open()) {
    return return_type::ERROR;
  }
  if (pending_stop_ != StopBehavior::kNone) {
    stop_all(pending_stop_);
    pending_stop_ = StopBehavior::kNone;
  }
  const auto t0 = Clock::now();
  const auto timeout = std::chrono::microseconds(static_cast<int64_t>(params_.read_timeout_ms * 1000));
  bus_.read_monitor(ids_, monitor_, monitor_valid_, timeout);
  const auto t1 = Clock::now();
  const double read_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  for (size_t i = 0; i < joints_.size(); ++i) {
    auto & j = joints_[i];
    if (monitor_valid_[i]) {
      apply_sample(j, monitor_[i], t1);
      j.misses = 0;
    } else if (++j.misses >= j.max_misses) {
      RCLCPP_ERROR(
        logger_, "%s (id %d) missed %d consecutive reads; stopping (bus %s)", j.name.c_str(),
        j.id, j.misses, params_.port.c_str());
      return return_type::ERROR;
    } else {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 1000, "%s (id %d) did not reply (%d in a row)", j.name.c_str(),
        j.id, j.misses);
    }
  }

  const double period_ms = period.seconds() * 1000.0;
  if (period_ms > 0.0 && read_ms > 0.6 * period_ms) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000,
      "Bus read took %.1f ms of a %.1f ms cycle; lower update_rate or check the USB adapter "
      "latency (low_latency / latency_timer)", read_ms, period_ms);
  }
  if (bus_.checksum_errors() != last_checksum_errors_) {
    last_checksum_errors_ = bus_.checksum_errors();
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000, "%u corrupt frames on the bus so far; check cabling and power",
      last_checksum_errors_);
  }
  return return_type::OK;
}

void FashionStarSystem::fill_targets(double period_s, Clock::time_point now)
{
  velocity_targets_.clear();
  target_joint_.clear();
  const double lead = params_.lead_time;
  const auto resend = std::chrono::duration<double>(params_.resend_s);

  for (size_t i = 0; i < joints_.size(); ++i) {
    auto & j = joints_[i];
    if (!std::isfinite(j.cmd_pos)) {
      j.have_prev_cmd = false;
      continue;  // no controller is commanding this joint
    }
    const JointMap map{j.scale, j.offset_deg, j.min_pos, j.max_pos};
    const double q = clamp_to_limits(j.cmd_pos, map);

    double v = 0.0;
    if (!j.feedforward) {
      // Step-commanded joints (grippers): go straight to the target at max speed.
    } else if (has_velocity_command_ && std::isfinite(j.cmd_vel)) {
      v = j.cmd_vel;
    } else if (j.have_prev_cmd && period_s > 1e-4) {
      v = (q - j.prev_cmd_pos) / period_s;  // controller without velocity output
    }
    const bool still = j.have_prev_cmd && std::abs(v / j.scale) < 0.05 &&  // deg/s
      std::abs((q - j.prev_cmd_pos) / j.scale) < 0.01;                     // deg
    j.prev_cmd_pos = q;
    j.have_prev_cmd = true;

    // Gravity trim: once the setpoint has been still for a moment (the servo has settled),
    // shift the goal by the remaining error. Kept during motion: the load changes slowly.
    j.still_s = still ? j.still_s + period_s : 0.0;
    if (j.feedforward && j.still_s > kTrimSettleS) {
      j.trim_deg = limit_trim(
        update_trim(
          j.trim_deg, q / j.scale + j.offset_deg - j.servo_deg, period_s, params_.trim_time,
          params_.trim_deadband_deg, params_.trim_max_deg),
        q, map);
    }

    // Aim ahead of the setpoint so the servo is still cruising when the next frame arrives.
    // Step-commanded joints (grippers) close the whole gap quickly. The trim shifts goal and
    // measurement alike, so the speed law sees the trimmed gap.
    const ServoGoal g = compute_servo_goal(
      q, v, j.servo_deg - j.trim_deg, map, lead,
      j.feedforward ? params_.correction_time : lead, params_.min_speed_dps, j.max_speed_dps);
    const double goal_deg =
      clamp_to_limits(j.to_joint(g.goal_deg + j.trim_deg), map) / j.scale + j.offset_deg;

    const bool goal_moved = !j.sent_once ||
      std::abs(v / j.scale) > 0.5 ||  // > 0.5 deg/s
      std::abs(goal_deg - j.last_goal_deg) > params_.goal_deadband_deg;
    // Grippers (no feedforward) are re-sent at most once after drifting: see should_send.
    if (!should_send(
        goal_moved, goal_deg, j.servo_deg, params_.hold_tolerance_deg,
        now - j.last_send >= resend, !j.feedforward, j.drift_resent))
    {
      continue;
    }

    const int32_t goal_ddeg = static_cast<int32_t>(std::lround(goal_deg * 10.0));
    velocity_targets_.push_back(
      {j.id, goal_ddeg, static_cast<uint16_t>(std::lround(g.speed_dps * 10.0)), params_.t_acc_ms,
        params_.t_dec_ms, j.power_mw});
    target_joint_.push_back(i);
    j.last_goal_deg = goal_deg;
  }
}

FashionStarSystem::return_type FashionStarSystem::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!bus_.is_open()) {
    return return_type::ERROR;
  }
  const auto now = Clock::now();
  fill_targets(period.seconds(), now);
  if (!bus_.write_sync_velocity(velocity_targets_)) {
    RCLCPP_ERROR(logger_, "Serial write failed on %s", params_.port.c_str());
    return return_type::ERROR;
  }
  for (size_t idx : target_joint_) {
    joints_[idx].last_send = now;
    joints_[idx].sent_once = true;
  }
  return return_type::OK;
}

}  // namespace starai_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(starai_hardware::FashionStarSystem, hardware_interface::SystemInterface)
