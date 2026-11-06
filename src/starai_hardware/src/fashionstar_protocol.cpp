#include "starai_hardware/fashionstar_protocol.hpp"

#include <cmath>

namespace starai_hardware::fsus
{
namespace
{
void put_u8(std::vector<uint8_t> & b, uint8_t v) { b.push_back(v); }
void put_u16(std::vector<uint8_t> & b, uint16_t v)
{
  b.push_back(static_cast<uint8_t>(v & 0xFF));
  b.push_back(static_cast<uint8_t>(v >> 8));
}
void put_u32(std::vector<uint8_t> & b, uint32_t v)
{
  for (int i = 0; i < 4; ++i) {
    b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
  }
}
uint16_t get_u16(const uint8_t * p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t get_u32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

// Payload scratch buffer; frames are tiny so a fixed-capacity thread_local avoids
// allocating in the control loop after the first call.
std::vector<uint8_t> & scratch()
{
  thread_local std::vector<uint8_t> buf = [] {
      std::vector<uint8_t> v;
      v.reserve(kMaxPayload);
      return v;
    }();
  buf.clear();
  return buf;
}
}  // namespace

double MonitorData::temperature_c() const
{
  // NTC conversion from the vendor SDK (B = 3435, R25 = 10k, 12-bit ADC).
  const double raw = temperature_raw;
  if (raw <= 0.0 || raw >= 4096.0) {
    return std::nan("");
  }
  return 1.0 / (std::log(raw / (4096.0 - raw)) / 3435.0 + 1.0 / (273.15 + 25.0)) - 273.15;
}

bool encode_request(uint8_t cmd, const uint8_t * payload, size_t len, std::vector<uint8_t> & out)
{
  if (len > kMaxPayload) {
    return false;
  }
  const size_t start = out.size();
  out.push_back(kRequestHeader0);
  out.push_back(kRequestHeader1);
  out.push_back(cmd);
  out.push_back(static_cast<uint8_t>(len));
  out.insert(out.end(), payload, payload + len);
  uint32_t sum = 0;
  for (size_t i = start; i < out.size(); ++i) {
    sum += out[i];
  }
  out.push_back(static_cast<uint8_t>(sum & 0xFF));
  return true;
}

bool encode_ping(uint8_t id, std::vector<uint8_t> & out)
{
  return encode_request(kPing, &id, 1, out);
}

bool encode_reset_multi_turn(uint8_t id, std::vector<uint8_t> & out)
{
  return encode_request(kResetMultiTurn, &id, 1, out);
}

bool encode_stop(uint8_t id, StopMode mode, uint16_t power_mw, std::vector<uint8_t> & out)
{
  auto & p = scratch();
  put_u8(p, id);
  put_u8(p, static_cast<uint8_t>(mode));
  put_u16(p, power_mw);
  return encode_request(kStopOnControl, p.data(), p.size(), out);
}

bool encode_sync_monitor(const uint8_t * ids, size_t count, std::vector<uint8_t> & out)
{
  if (count == 0 || 3 + count > kMaxPayload) {
    return false;
  }
  auto & p = scratch();
  put_u8(p, kQueryMonitor);  // sub-command
  put_u8(p, 1);              // bytes per servo
  put_u8(p, static_cast<uint8_t>(count));
  p.insert(p.end(), ids, ids + count);
  return encode_request(kSyncCommand, p.data(), p.size(), out);
}

bool encode_sync_velocity(const VelocityTarget * t, size_t count, std::vector<uint8_t> & out)
{
  constexpr size_t kPerServo = 13;
  if (count == 0 || 3 + count * kPerServo > kMaxPayload) {
    return false;
  }
  auto & p = scratch();
  put_u8(p, kSetAngleMturnByVelocity);
  put_u8(p, kPerServo);
  put_u8(p, static_cast<uint8_t>(count));
  for (size_t i = 0; i < count; ++i) {
    put_u8(p, t[i].id);
    put_u32(p, static_cast<uint32_t>(t[i].angle_ddeg));
    put_u16(p, t[i].speed_ddps);
    put_u16(p, t[i].t_acc_ms);
    put_u16(p, t[i].t_dec_ms);
    put_u16(p, t[i].power_mw);
  }
  return encode_request(kSyncCommand, p.data(), p.size(), out);
}

bool decode_monitor(const Packet & pkt, MonitorData & out)
{
  if (pkt.cmd != kQueryMonitor || pkt.len != 16) {
    return false;
  }
  const uint8_t * p = pkt.payload.data();
  out.id = p[0];
  out.voltage_mv = get_u16(p + 1);
  out.current_ma = get_u16(p + 3);
  out.power_mw = get_u16(p + 5);
  out.temperature_raw = get_u16(p + 7);
  out.status = p[9];
  out.angle_ddeg = static_cast<int32_t>(get_u32(p + 10));
  out.turns = static_cast<int16_t>(get_u16(p + 14));
  return true;
}

void ResponseParser::reset()
{
  state_ = State::kHeader0;
  received_ = 0;
  sum_ = 0;
}

bool ResponseParser::push(uint8_t b)
{
  switch (state_) {
    case State::kHeader0:
      if (b == kResponseHeader0) {
        sum_ = b;
        state_ = State::kHeader1;
      }
      return false;
    case State::kHeader1:
      if (b == kResponseHeader1) {
        sum_ += b;
        state_ = State::kCmd;
      } else if (b == kResponseHeader0) {
        sum_ = b;  // stay: could be the real start of a header
      } else {
        state_ = State::kHeader0;
      }
      return false;
    case State::kCmd:
      packet_.cmd = b;
      sum_ += b;
      state_ = State::kLen;
      return false;
    case State::kLen:
      if (b > kMaxPayload) {  // extended-length frames are never sent by the servos we use
        reset();
        return false;
      }
      packet_.len = b;
      sum_ += b;
      received_ = 0;
      state_ = b == 0 ? State::kChecksum : State::kPayload;
      return false;
    case State::kPayload:
      packet_.payload[received_++] = b;
      sum_ += b;
      if (received_ == packet_.len) {
        state_ = State::kChecksum;
      }
      return false;
    case State::kChecksum: {
      const bool ok = (sum_ & 0xFF) == b;
      reset();
      if (!ok) {
        ++checksum_errors_;
      }
      return ok;
    }
  }
  return false;
}

}  // namespace starai_hardware::fsus
