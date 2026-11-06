// ros2_control SystemInterface for arms built from Fashion Star UART bus servos
// (StarAI Cello / Viola).
//
// Every control cycle:
//   read():  one sync monitor request -> position, filtered velocity, current, voltage,
//            temperature and status of every servo.
//   write(): one sync "multi-turn angle by velocity" frame. The servo is aimed slightly
//            ahead of the commanded position (q + v * lead_time) at the commanded speed,
//            plus a correction for any lag (correction_time). Successive commands
//            blend into continuous motion instead of stopping at every waypoint.
#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "starai_hardware/fashionstar_bus.hpp"

#if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
#define STARAI_HW_HAS_INTERFACE_PARAMS 1
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#else
#define STARAI_HW_HAS_INTERFACE_PARAMS 0
#endif

namespace starai_hardware
{

class FashionStarSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using return_type = hardware_interface::return_type;

  ~FashionStarSystem() override;

#if STARAI_HW_HAS_INTERFACE_PARAMS
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
#else
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // Interfaces are exported with raw pointers to member storage: no per-cycle lookups.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
#pragma GCC diagnostic pop

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;

  enum class StopBehavior { kHold, kRelease, kDamping, kNone };

  struct Params
  {
    std::string port = "/dev/ttyUSB0";
    int baudrate = 1000000;
    double lead_time = 0.1;           // s, how far ahead of the setpoint the servo is aimed
    double correction_time = 0.1;     // s, time to close a setpoint-to-measured gap
    double trim_time = 1.0;           // s, gravity trim time constant; 0 = off
    double trim_deadband_deg = 0.2;   // errors within this are not trimmed
    double trim_max_deg = 3.0;        // trim limit per joint
    double min_speed_dps = 2.0;       // floor for the cruise speed sent to the servo
    double max_speed_dps = 360.0;     // default per-joint ceiling
    uint16_t t_acc_ms = 20;
    uint16_t t_dec_ms = 20;
    uint16_t power_mw = 8000;         // default per-joint power limit sent with commands
    uint16_t hold_power_mw = 8000;    // power used by "hold" stop commands
    double read_timeout_ms = 8.0;
    int max_consecutive_misses = 5;   // per servo; exceeded -> read() returns ERROR
    double velocity_filter_hz = 15.0;
    double goal_deadband_deg = 0.05;  // holding joints are re-sent only when the goal moves
    double hold_tolerance_deg = 1.5;  // ...or the servo drifted this far off it,
    double resend_s = 1.0;            // checked at most this often
    bool reset_multi_turn = true;     // fix wrapped multi-turn counters at configure
    StopBehavior deactivate_behavior = StopBehavior::kRelease;
    StopBehavior shutdown_behavior = StopBehavior::kHold;
    StopBehavior error_behavior = StopBehavior::kHold;
  };

  struct Joint
  {
    std::string name;
    uint8_t id = 0;
    double scale = 0.0;        // joint units per servo degree (sign = direction)
    double offset_deg = 0.0;   // servo angle at joint zero
    double min_pos = 0.0;      // joint units; NaN = unlimited
    double max_pos = 0.0;
    double max_speed_dps = 0.0;
    uint16_t power_mw = 0;
    bool feedforward = true;   // aim ahead using the commanded/estimated velocity

    // Exported state
    double pos = 0.0;
    double vel = 0.0;
    double current = 0.0;      // A
    double voltage = 0.0;      // V
    double temperature = 0.0;  // degC
    double status = 0.0;       // raw status bits
    // Exported commands
    double cmd_pos = 0.0;
    double cmd_vel = 0.0;

    // Internal
    double servo_deg = 0.0;
    Clock::time_point stamp{};
    bool have_sample = false;
    double prev_cmd_pos = 0.0;
    bool have_prev_cmd = false;
    double last_goal_deg = 0.0;
    Clock::time_point last_send{};
    bool sent_once = false;
    int misses = 0;
    uint8_t last_status = 0;
    double trim_deg = 0.0;     // gravity trim added to the servo goal
    double still_s = 0.0;      // how long the setpoint has been still
    bool drift_resent = false; // current goal already re-sent because the servo drifted

    double to_joint(double deg) const { return (deg - offset_deg) * scale; }
  };

  CallbackReturn init_from_info();
  bool parse_params(std::string & error);
  bool read_all(int attempts);  // synchronous full read used outside the control loop
  void apply_sample(Joint & j, const fsus::MonitorData & m, Clock::time_point stamp);
  void stop_all(StopBehavior behavior);
  void log_status_change(Joint & j, uint8_t status);
  void fill_targets(double period_s, Clock::time_point now);

  rclcpp::Logger logger_ = rclcpp::get_logger("FashionStarSystem");
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  Params params_;
  std::vector<Joint> joints_;
  bool has_velocity_command_ = false;

  FashionStarBus bus_;
  std::vector<uint8_t> ids_;
  std::vector<fsus::MonitorData> monitor_;
  std::vector<uint8_t> monitor_valid_;
  std::vector<fsus::VelocityTarget> velocity_targets_;
  std::vector<size_t> target_joint_;  // joint index of each queued target

  StopBehavior pending_stop_ = StopBehavior::kNone;  // deactivate stop, applied in read()
  uint32_t last_checksum_errors_ = 0;
};

}  // namespace starai_hardware
