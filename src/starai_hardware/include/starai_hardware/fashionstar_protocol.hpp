// Fashion Star UART bus-servo protocol: packet encoding and decoding.
//
// Pure functions without I/O so they can be unit tested. Wire format (little endian):
//   request : 0x12 0x4C | cmd | len | payload[len] | checksum
//   response: 0x05 0x1C | cmd | len | payload[len] | checksum
//   checksum = (sum of all preceding bytes) mod 256
//
// Units used by the servo: angle 0.1 deg, speed 0.1 deg/s, time ms, power mW,
// voltage mV, current mA.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace starai_hardware::fsus
{

constexpr uint8_t kRequestHeader0 = 0x12;
constexpr uint8_t kRequestHeader1 = 0x4C;
constexpr uint8_t kResponseHeader0 = 0x05;
constexpr uint8_t kResponseHeader1 = 0x1C;
constexpr size_t kMaxPayload = 254;  // a length byte of 0xFF announces the extended format

enum Command : uint8_t
{
  kPing = 1,
  kSetAngleMturn = 13,
  kSetAngleMturnByVelocity = 15,
  kResetMultiTurn = 17,
  kQueryMonitor = 22,
  kStopOnControl = 24,
  kSyncCommand = 25,
};

enum class StopMode : uint8_t
{
  kRelease = 0x10,  // stop and release torque (limp)
  kHold = 0x11,     // stop and hold position
  kDamping = 0x12,  // stop and go into damping mode
};

// Status byte bits reported in monitor data.
enum StatusBit : uint8_t
{
  kStatusBusy = 1 << 0,
  kStatusCommandError = 1 << 1,
  kStatusStall = 1 << 2,
  kStatusOverVoltage = 1 << 3,
  kStatusUnderVoltage = 1 << 4,
  kStatusOverCurrent = 1 << 5,
  kStatusOverPower = 1 << 6,
  kStatusOverTemperature = 1 << 7,
};
constexpr uint8_t kStatusFaultMask = static_cast<uint8_t>(~(kStatusBusy | kStatusCommandError));

struct VelocityTarget
{
  uint8_t id;
  int32_t angle_ddeg;   // target angle, 0.1 deg (multi-turn)
  uint16_t speed_ddps;  // cruise speed, 0.1 deg/s
  uint16_t t_acc_ms;
  uint16_t t_dec_ms;
  uint16_t power_mw;
};

struct MonitorData
{
  uint8_t id = 0;
  uint16_t voltage_mv = 0;
  uint16_t current_ma = 0;
  uint16_t power_mw = 0;
  uint16_t temperature_raw = 0;
  uint8_t status = 0;
  int32_t angle_ddeg = 0;  // multi-turn angle, 0.1 deg
  int16_t turns = 0;

  double angle_deg() const { return angle_ddeg / 10.0; }
  double temperature_c() const;
};

struct Packet
{
  uint8_t cmd = 0;
  uint8_t len = 0;
  std::array<uint8_t, kMaxPayload> payload{};
};

// Encoders append one complete request frame to `out`. They return false (and leave
// `out` unchanged) if the payload would not fit in a single frame.
bool encode_request(uint8_t cmd, const uint8_t * payload, size_t len, std::vector<uint8_t> & out);
bool encode_ping(uint8_t id, std::vector<uint8_t> & out);
bool encode_reset_multi_turn(uint8_t id, std::vector<uint8_t> & out);
bool encode_stop(uint8_t id, StopMode mode, uint16_t power_mw, std::vector<uint8_t> & out);
bool encode_sync_monitor(const uint8_t * ids, size_t count, std::vector<uint8_t> & out);
bool encode_sync_velocity(const VelocityTarget * targets, size_t count, std::vector<uint8_t> & out);

bool decode_monitor(const Packet & pkt, MonitorData & out);

// Incremental response parser. Feed raw bytes; complete frames with a valid checksum
// are returned through `next()`. Garbage, request echoes and corrupt frames are
// skipped by resynchronising on the response header.
class ResponseParser
{
public:
  void reset();
  // Consumes one byte. Returns true if a complete valid packet is now available in `packet()`.
  bool push(uint8_t byte);
  const Packet & packet() const { return packet_; }
  uint32_t checksum_errors() const { return checksum_errors_; }

private:
  enum class State { kHeader0, kHeader1, kCmd, kLen, kPayload, kChecksum };
  State state_ = State::kHeader0;
  Packet packet_;
  size_t received_ = 0;
  uint32_t sum_ = 0;
  uint32_t checksum_errors_ = 0;
};

}  // namespace starai_hardware::fsus
