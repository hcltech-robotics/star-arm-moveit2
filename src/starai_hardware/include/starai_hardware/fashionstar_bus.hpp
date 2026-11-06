// Fashion Star servo bus over a POSIX serial port.
//
// All calls are synchronous with bounded timeouts so they can be used from the
// ros2_control read()/write() loop. Buffers are preallocated in open(); the
// steady-state read/write path does not allocate.
#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "starai_hardware/fashionstar_protocol.hpp"

namespace starai_hardware
{

class FashionStarBus
{
public:
  using Clock = std::chrono::steady_clock;

  FashionStarBus() = default;
  ~FashionStarBus();
  FashionStarBus(const FashionStarBus &) = delete;
  FashionStarBus & operator=(const FashionStarBus &) = delete;

  // Opens the port in raw 8N1 mode, takes an exclusive lock and enables low-latency
  // mode when the driver supports it. Returns false and fills `error` on failure.
  bool open(const std::string & port, int baudrate, std::string & error);
  void close();
  bool is_open() const { return fd_ >= 0; }
  // Non-fatal notes collected during open() (e.g. low-latency mode unsupported).
  const std::vector<std::string> & open_warnings() const { return open_warnings_; }

  bool ping(uint8_t id, std::chrono::microseconds timeout);

  // Sends one sync monitor request for `ids` and collects replies until all have
  // arrived or `timeout` expires. `out[i]` / `valid[i]` correspond to `ids[i]`.
  // Returns the number of servos that replied.
  size_t read_monitor(
    const std::vector<uint8_t> & ids, std::vector<fsus::MonitorData> & out,
    std::vector<uint8_t> & valid, std::chrono::microseconds timeout);

  bool write_sync_velocity(const std::vector<fsus::VelocityTarget> & targets);
  bool stop(uint8_t id, fsus::StopMode mode, uint16_t power_mw);
  bool reset_multi_turn(uint8_t id);

  uint32_t checksum_errors() const { return parser_.checksum_errors(); }

private:
  bool write_frame();
  void drain_input();
  // Reads available bytes until `deadline`; returns false on timeout or I/O error.
  bool wait_readable(Clock::time_point deadline);

  int fd_ = -1;
  std::string port_;
  std::vector<uint8_t> tx_;
  std::vector<uint8_t> rx_;
  fsus::ResponseParser parser_;
  std::vector<std::string> open_warnings_;
};

}  // namespace starai_hardware
