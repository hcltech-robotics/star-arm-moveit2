// Protocol tests. Golden frames were produced with the vendor Python SDK
// (fashionstar_uart_sdk 1.3.8) so the encoder is checked against the reference
// implementation byte for byte.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "starai_hardware/fashionstar_protocol.hpp"

using namespace starai_hardware::fsus;

namespace
{
std::vector<uint8_t> hex(const std::string & s)
{
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < s.size(); i += 2) {
    out.push_back(static_cast<uint8_t>(std::stoul(s.substr(i, 2), nullptr, 16)));
  }
  return out;
}

std::vector<uint8_t> monitor_frame(uint8_t id, int32_t angle_ddeg, uint8_t status = 0)
{
  std::vector<uint8_t> f = {kResponseHeader0, kResponseHeader1, kQueryMonitor, 16, id,
    0x10, 0x2E,  // 11792 mV
    0x2C, 0x01,  // 300 mA
    0xB8, 0x0B,  // 3000 mW
    0x00, 0x08,  // temp raw 2048 -> 25 C
    status};
  for (int i = 0; i < 4; ++i) {
    f.push_back(static_cast<uint8_t>((static_cast<uint32_t>(angle_ddeg) >> (8 * i)) & 0xFF));
  }
  f.push_back(0x00);
  f.push_back(0x00);  // turns
  uint32_t sum = 0;
  for (auto b : f) sum += b;
  f.push_back(static_cast<uint8_t>(sum & 0xFF));
  return f;
}
}  // namespace

TEST(Encode, Ping)
{
  std::vector<uint8_t> out;
  ASSERT_TRUE(encode_ping(3, out));
  EXPECT_EQ(out, hex("124c01010363"));
}

TEST(Encode, SyncVelocityMatchesVendorSdk)
{
  VelocityTarget t[] = {{0, -1234, 300, 20, 20, 8000}, {1, 456, 3000, 20, 20, 8000}};
  std::vector<uint8_t> out;
  ASSERT_TRUE(encode_sync_velocity(t, 2, out));
  EXPECT_EQ(out, hex("124c191d0f0d02002efbffff2c0114001400401f01c8010000b80b14001400401fa1"));
}

TEST(Encode, StopResetMonitor)
{
  std::vector<uint8_t> out;
  ASSERT_TRUE(encode_stop(5, StopMode::kHold, 8000, out));
  EXPECT_EQ(out, hex("124c18040511401fef"));
  out.clear();
  ASSERT_TRUE(encode_reset_multi_turn(2, out));
  EXPECT_EQ(out, hex("124c11010272"));
  out.clear();
  const uint8_t ids[] = {0, 1, 2, 3, 4, 5, 6};
  ASSERT_TRUE(encode_sync_monitor(ids, 7, out));
  EXPECT_EQ(out, hex("124c190a16010700010203040506b4"));
}

TEST(Encode, RejectsOversizedSync)
{
  std::vector<VelocityTarget> t(20, VelocityTarget{0, 0, 0, 0, 0, 0});  // 3 + 20*13 > 254
  std::vector<uint8_t> out;
  EXPECT_FALSE(encode_sync_velocity(t.data(), t.size(), out));
  EXPECT_TRUE(out.empty());
}

TEST(Parser, DecodesMonitorFrames)
{
  ResponseParser p;
  MonitorData m;
  int packets = 0;
  for (auto b : monitor_frame(4, -1234, kStatusStall)) {
    if (p.push(b)) {
      ++packets;
      ASSERT_TRUE(decode_monitor(p.packet(), m));
    }
  }
  ASSERT_EQ(packets, 1);
  EXPECT_EQ(m.id, 4);
  EXPECT_DOUBLE_EQ(m.angle_deg(), -123.4);
  EXPECT_EQ(m.voltage_mv, 11792);
  EXPECT_EQ(m.current_ma, 300);
  EXPECT_EQ(m.status, kStatusStall);
  EXPECT_NEAR(m.temperature_c(), 25.0, 0.01);
}

TEST(Parser, ResyncsAfterGarbageEchoAndCorruption)
{
  std::vector<uint8_t> stream = {0x00, 0x05, 0x05, 0xFF};  // noise incl. partial headers
  // Echo of our own request (half-duplex adapters may echo) must be ignored.
  const uint8_t ids[] = {0, 1};
  encode_sync_monitor(ids, 2, stream);
  auto bad = monitor_frame(0, 100);
  bad[10] ^= 0x01;  // corrupt payload -> checksum mismatch
  stream.insert(stream.end(), bad.begin(), bad.end());
  auto good0 = monitor_frame(0, 100);
  auto good1 = monitor_frame(1, -900);
  stream.insert(stream.end(), good0.begin(), good0.end());
  stream.insert(stream.end(), good1.begin(), good1.end());

  ResponseParser p;
  std::vector<MonitorData> got;
  for (auto b : stream) {
    MonitorData m;
    if (p.push(b) && decode_monitor(p.packet(), m)) {
      got.push_back(m);
    }
  }
  ASSERT_EQ(got.size(), 2u);
  EXPECT_EQ(got[0].id, 0);
  EXPECT_EQ(got[0].angle_ddeg, 100);
  EXPECT_EQ(got[1].id, 1);
  EXPECT_EQ(got[1].angle_ddeg, -900);
  EXPECT_EQ(p.checksum_errors(), 1u);
}
