// Byte-for-byte agreement with docs/architecture/protocol.md section 10 and
// with tools/host_gui/protocol.py. If a vector here fails, do not touch the
// node: the codec is wrong.

#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

#include "robot_base/protocol.hpp"

namespace
{

using robot_base::protocol::FrameParser;

std::vector<uint8_t> hex(const std::string & text)
{
  std::vector<uint8_t> bytes;
  for (std::size_t i = 0; i + 1 < text.size(); i += 3) {
    bytes.push_back(static_cast<uint8_t>(std::stoul(text.substr(i, 2), nullptr, 16)));
  }
  return bytes;
}

void append_u16(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
}

void append_u32(std::vector<uint8_t> & out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xFFU));
}

const std::vector<uint8_t> kVectorVelocity = hex("AA 55 06 01 01 2C 01 F4 01 72 3F");
const std::vector<uint8_t> kVectorEnable = hex("AA 55 03 04 02 01 9F B5");
const std::vector<uint8_t> kVectorEstop = hex("AA 55 02 06 03 39 38");
const std::vector<uint8_t> kVectorParamSave = hex("AA 55 03 08 04 00 79 7A");
const std::vector<uint8_t> kVectorStorageQuery = hex("AA 55 02 0A 05 92 1D");
const std::vector<uint8_t> kVectorLogExport =
  hex("AA 55 08 0B 06 00 01 00 00 00 00 56 EC");

}  // namespace

TEST(CrcTests, CheckValue)
{
  const std::string data = "123456789";
  EXPECT_EQ(
    robot_base::protocol::crc16_ccitt(
      reinterpret_cast<const uint8_t *>(data.data()), data.size()),
    0x29B1U);
}

TEST(CrcTests, IncrementalMatchesOneShot)
{
  std::vector<uint8_t> data;
  for (int repeat = 0; repeat < 3; ++repeat) {
    for (int value = 0; value < 256; ++value) {
      data.push_back(static_cast<uint8_t>(value));
    }
  }

  uint16_t incremental = 0xFFFFU;
  for (uint8_t byte : data) {
    incremental = robot_base::protocol::crc16_update(incremental, byte);
  }
  EXPECT_EQ(
    incremental, robot_base::protocol::crc16_ccitt(data.data(), data.size()));
}

TEST(FrameVectorTests, SetVelocity)
{
  EXPECT_EQ(
    robot_base::protocol::build_frame(
      static_cast<uint8_t>(robot_base::protocol::Cmd::SET_VELOCITY), 1,
      robot_base::protocol::encode_set_velocity(300, 500)),
    kVectorVelocity);
}

TEST(FrameVectorTests, SetEnable)
{
  EXPECT_EQ(
    robot_base::protocol::build_frame(
      static_cast<uint8_t>(robot_base::protocol::Cmd::SET_ENABLE), 2,
      robot_base::protocol::encode_set_enable(true)),
    kVectorEnable);
}

TEST(FrameVectorTests, Estop)
{
  EXPECT_EQ(
    robot_base::protocol::build_frame(
      static_cast<uint8_t>(robot_base::protocol::Cmd::ESTOP), 3),
    kVectorEstop);
}

TEST(FrameVectorTests, ParseVelocityFields)
{
  FrameParser parser;
  const auto frames = parser.feed(kVectorVelocity);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(
    frames[0].cmd, static_cast<uint8_t>(robot_base::protocol::Cmd::SET_VELOCITY));
  EXPECT_EQ(frames[0].seq, 1U);
  EXPECT_EQ(frames[0].payload, hex("2C 01 F4 01"));
}

TEST(FrameVectorTests, ParseEstopShortestFrame)
{
  FrameParser parser;
  const auto frames = parser.feed(kVectorEstop);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].cmd, static_cast<uint8_t>(robot_base::protocol::Cmd::ESTOP));
  EXPECT_TRUE(frames[0].payload.empty());
}

TEST(ParserRobustnessTests, ByteAtATimeEqualsBulk)
{
  std::vector<uint8_t> stream = kVectorVelocity;
  stream.insert(stream.end(), kVectorEnable.begin(), kVectorEnable.end());
  stream.insert(stream.end(), kVectorEstop.begin(), kVectorEstop.end());

  FrameParser bulk_parser;
  const auto bulk = bulk_parser.feed(stream);

  FrameParser drip_parser;
  std::vector<robot_base::protocol::Frame> drip;
  for (uint8_t byte : stream) {
    const auto frames = drip_parser.feed(&byte, 1);
    drip.insert(drip.end(), frames.begin(), frames.end());
  }

  ASSERT_EQ(bulk.size(), drip.size());
  for (std::size_t i = 0; i < bulk.size(); ++i) {
    EXPECT_EQ(bulk[i].cmd, drip[i].cmd);
    EXPECT_EQ(bulk[i].seq, drip[i].seq);
    EXPECT_EQ(bulk[i].payload, drip[i].payload);
  }
}

TEST(ParserRobustnessTests, BadCrcIsDroppedAndNextFrameParses)
{
  auto bad = kVectorVelocity;
  bad.back() ^= 0xFFU;

  FrameParser parser;
  EXPECT_TRUE(parser.feed(bad).empty());
  EXPECT_EQ(parser.crc_errors(), 1U);
  EXPECT_EQ(parser.feed(kVectorEnable).size(), 1U);
}

TEST(ParserRobustnessTests, BadLenIsDroppedWithoutBreakingResync)
{
  const uint8_t lengths[] = {0x00, 0x01, 0x43, 0xFF};
  for (uint8_t length : lengths) {
    FrameParser parser;
    const std::vector<uint8_t> junk = {0xAA, 0x55, length, 0x01, 0x01};
    EXPECT_TRUE(parser.feed(junk).empty()) << "LEN=" << static_cast<int>(length);
    EXPECT_EQ(parser.feed(kVectorEstop).size(), 1U) <<
      "LEN=" << static_cast<int>(length) << " broke resync";
  }
}

TEST(ParserRobustnessTests, BadLenByteMayStartNextFrame)
{
  FrameParser parser;
  std::vector<uint8_t> stream = {0xAA, 0x55, 0xAA, 0x55};
  stream.insert(stream.end(), kVectorEstop.begin(), kVectorEstop.end());
  const auto frames = parser.feed(stream);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].cmd, static_cast<uint8_t>(robot_base::protocol::Cmd::ESTOP));
}

TEST(ParserRobustnessTests, RandomStreamNeverThrows)
{
  std::mt19937 rng(1234);
  std::uniform_int_distribution<int> byte_dist(0, 255);
  std::uniform_int_distribution<int> length_dist(1, 40);

  FrameParser parser;
  for (int i = 0; i < 2000; ++i) {
    std::vector<uint8_t> chunk(static_cast<std::size_t>(length_dist(rng)));
    for (auto & byte : chunk) {
      byte = static_cast<uint8_t>(byte_dist(rng));
    }
    parser.feed(chunk);
  }
  EXPECT_EQ(parser.feed(kVectorVelocity).size(), 1U);
}

TEST(ParserRobustnessTests, ResetDiscardsHalfFrame)
{
  FrameParser parser;
  const std::vector<uint8_t> half(kVectorVelocity.begin(), kVectorVelocity.begin() + 5);
  EXPECT_TRUE(parser.feed(half).empty());
  parser.reset();
  EXPECT_EQ(parser.feed(kVectorVelocity).size(), 1U);
}

TEST(AckAndReportTests, AckFields)
{
  const auto raw = robot_base::protocol::build_frame(
    static_cast<uint8_t>(robot_base::protocol::Cmd::SET_ENABLE) |
    robot_base::protocol::kAckFlag,
    7, std::vector<uint8_t>{2, static_cast<uint8_t>(robot_base::protocol::Result::OK)});

  FrameParser parser;
  const auto frames = parser.feed(raw);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_TRUE(frames[0].is_ack());
  EXPECT_EQ(frames[0].acked_cmd(), 0x04);
  EXPECT_EQ(frames[0].acked_seq(), 2U);
  EXPECT_EQ(frames[0].result(), 0U);
}

TEST(AckAndReportTests, StatusRoundTrip)
{
  const auto status = robot_base::protocol::decode_status(hex("02 01 05 E4 04 20 32 00"));
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->state, 2U);
  EXPECT_EQ(status->fault, 0x0501U);
  EXPECT_EQ(status->battery_mv, 0x04E4U);
  EXPECT_EQ(status->temperature_c, 32);
  EXPECT_EQ(status->jitter_us, 0x0032U);
}

TEST(AckAndReportTests, ArmRoundTrip)
{
  std::vector<uint8_t> payload;
  const int16_t angles[] = {0, -100, 200, -300, 400, -500};
  for (int16_t angle : angles) {
    append_u16(payload, static_cast<uint16_t>(angle));
  }
  append_u16(payload, 850U);
  payload.push_back(1U);

  const auto arm = robot_base::protocol::decode_arm(payload);
  ASSERT_TRUE(arm.has_value());
  EXPECT_EQ(arm->angles_centi_deg[0], 0);
  EXPECT_EQ(arm->angles_centi_deg[1], -100);
  EXPECT_EQ(arm->angles_centi_deg[5], -500);
  EXPECT_EQ(arm->gripper_pos, 850);
  EXPECT_EQ(arm->gripper_state, 1U);
}

TEST(AckAndReportTests, WheelSpeedRoundTrip)
{
  std::vector<uint8_t> payload;
  append_u16(payload, static_cast<uint16_t>(100));
  append_u16(payload, static_cast<uint16_t>(-200));
  append_u16(payload, 300U);
  append_u16(payload, static_cast<uint16_t>(-400));

  const auto speed = robot_base::protocol::decode_wheel_speed(payload);
  ASSERT_TRUE(speed.has_value());
  EXPECT_EQ(speed->fl_mm_s, 100);
  EXPECT_EQ(speed->fr_mm_s, -200);
  EXPECT_EQ(speed->rl_mm_s, 300);
  EXPECT_EQ(speed->rr_mm_s, -400);
}

TEST(AckAndReportTests, WheelDutyRoundTrip)
{
  std::vector<uint8_t> payload;
  append_u16(payload, 400U);
  append_u16(payload, static_cast<uint16_t>(-400));
  append_u16(payload, 0U);
  append_u16(payload, 25U);

  const auto duty = robot_base::protocol::decode_wheel_duty(payload);
  ASSERT_TRUE(duty.has_value());
  EXPECT_EQ(duty->fl_permille, 400);
  EXPECT_EQ(duty->fr_permille, -400);
  EXPECT_EQ(duty->rl_permille, 0);
  EXPECT_EQ(duty->rr_permille, 25);
}

TEST(AckAndReportTests, SetParamPayload)
{
  const auto payload = robot_base::protocol::encode_set_param(0x2B, 1500);
  ASSERT_EQ(payload.size(), 5U);
  EXPECT_EQ(payload[0], 0x2BU);
  EXPECT_EQ(payload[1], static_cast<uint8_t>(1500 & 0xFF));
  EXPECT_EQ(payload[2], static_cast<uint8_t>((1500 >> 8) & 0xFF));
  EXPECT_EQ(payload[3], 0U);
  EXPECT_EQ(payload[4], 0U);
}

TEST(AckAndReportTests, ShortReportsAreRejected)
{
  using namespace robot_base::protocol;  // NOLINT
  EXPECT_FALSE(decode_status(std::vector<uint8_t>{1, 2}).has_value());
  EXPECT_FALSE(decode_odom({}).has_value());
  EXPECT_FALSE(decode_wheel_speed(std::vector<uint8_t>(7, 0)).has_value());
  EXPECT_FALSE(decode_wheel_duty(std::vector<uint8_t>(9, 0)).has_value());
  EXPECT_FALSE(decode_storage(std::vector<uint8_t>(23, 0)).has_value());
  EXPECT_FALSE(decode_log(std::vector<uint8_t>(14, 0)).has_value());
  EXPECT_FALSE(decode_crash(std::vector<uint8_t>(6, 0)).has_value());
}

TEST(StorageTests, ParamSaveVector)
{
  EXPECT_EQ(
    robot_base::protocol::build_frame(
      static_cast<uint8_t>(robot_base::protocol::Cmd::PARAM_SAVE), 4,
      robot_base::protocol::encode_param_save(0)),
    kVectorParamSave);
}

TEST(StorageTests, StorageQueryVector)
{
  EXPECT_EQ(
    robot_base::protocol::build_frame(
      static_cast<uint8_t>(robot_base::protocol::Cmd::STORAGE_QUERY), 5),
    kVectorStorageQuery);
}

TEST(StorageTests, LogExportVector)
{
  EXPECT_EQ(
    robot_base::protocol::build_frame(
      static_cast<uint8_t>(robot_base::protocol::Cmd::LOG_EXPORT), 6,
      robot_base::protocol::encode_log_export(0, 1, 0)),
    kVectorLogExport);
}

TEST(StorageTests, ParamResetRequiresConfirm)
{
  EXPECT_EQ(
    robot_base::protocol::encode_param_reset(1),
    (std::vector<uint8_t>{1, 0xA5}));
  EXPECT_EQ(
    robot_base::protocol::encode_log_clear(2),
    (std::vector<uint8_t>{2, 0xA5}));
}

TEST(StorageTests, StorageRoundTrip)
{
  std::vector<uint8_t> payload;
  payload.push_back(0x03U);
  payload.push_back(1U);
  append_u16(payload, 7U);
  append_u16(payload, 3U);
  append_u16(payload, 0U);
  append_u32(payload, 42U);
  append_u32(payload, 1000U);
  append_u32(payload, 5U);
  append_u32(payload, 99U);
  ASSERT_EQ(payload.size(), 24U);

  const auto storage = robot_base::protocol::decode_storage(payload);
  ASSERT_TRUE(storage.has_value());
  EXPECT_EQ(storage->param_seq, 7U);
  EXPECT_EQ(storage->calib_seq, 3U);
  EXPECT_EQ(storage->boot_total, 42U);
  EXPECT_TRUE(storage->clean());
  EXPECT_TRUE(storage->param_valid());
  EXPECT_FALSE(storage->save_failed());
}

TEST(StorageTests, StorageFlagsFollowBitOrder)
{
  std::vector<uint8_t> payload(24, 0U);
  payload[0] = 0x09U;  // clean + save_failed
  const auto storage = robot_base::protocol::decode_storage(payload);
  ASSERT_TRUE(storage.has_value());
  EXPECT_TRUE(storage->clean());
  EXPECT_FALSE(storage->param_valid());
  EXPECT_FALSE(storage->calib_valid());
  EXPECT_TRUE(storage->save_failed());
}

TEST(StorageTests, LogRoundTrip)
{
  std::vector<uint8_t> payload;
  payload.push_back(0U);
  payload.push_back(1U);
  payload.push_back(6U);
  payload.push_back(0x01U);
  append_u32(payload, 123U);
  append_u32(payload, 9U);
  append_u16(payload, 0U);
  payload.push_back(5U);
  const char text[] = "hello";
  payload.insert(payload.end(), text, text + 5);

  const auto log = robot_base::protocol::decode_log(payload);
  ASSERT_TRUE(log.has_value());
  EXPECT_EQ(log->text, "hello");
  EXPECT_EQ(log->seq, 9U);
  EXPECT_TRUE(log->more());
  EXPECT_FALSE(log->end());
}

TEST(StorageTests, LogEndFrame)
{
  std::vector<uint8_t> payload;
  payload.push_back(0U);
  payload.push_back(1U);
  payload.push_back(6U);
  payload.push_back(0x02U);
  append_u32(payload, 124U);
  append_u32(payload, 10U);
  append_u16(payload, 0U);
  payload.push_back(0U);

  const auto log = robot_base::protocol::decode_log(payload);
  ASSERT_TRUE(log.has_value());
  EXPECT_TRUE(log->end());
  EXPECT_FALSE(log->more());
  EXPECT_TRUE(log->text.empty());
}

TEST(StorageTests, CrashChunkRoundTrip)
{
  std::vector<uint8_t> payload;
  payload.push_back(2U);
  payload.push_back(0x01U);
  append_u16(payload, 0U);
  append_u16(payload, 1024U);
  payload.push_back(57U);
  for (int i = 0; i < 57; ++i) {
    payload.push_back(static_cast<uint8_t>(i));
  }

  const auto chunk = robot_base::protocol::decode_crash(payload);
  ASSERT_TRUE(chunk.has_value());
  EXPECT_EQ(chunk->slot, 2U);
  EXPECT_EQ(chunk->offset, 0U);
  EXPECT_EQ(chunk->total_len, 1024U);
  EXPECT_EQ(chunk->data.size(), 57U);
  EXPECT_TRUE(chunk->more());
}

TEST(DownstreamTests, ArmJointsPayload)
{
  const auto payload = robot_base::protocol::encode_set_arm_joints(
    std::array<int16_t, 6>{1, 2, 3, 4, 5, 6});
  ASSERT_EQ(payload.size(), 12U);
  EXPECT_EQ(payload[0], 1U);
  EXPECT_EQ(payload[1], 0U);
  EXPECT_EQ(payload[10], 6U);
  EXPECT_EQ(payload[11], 0U);
}

TEST(DownstreamTests, GripperValidation)
{
  EXPECT_EQ(
    robot_base::protocol::encode_set_gripper(1, 50),
    (std::vector<uint8_t>{1, 50}));
  EXPECT_THROW(
    robot_base::protocol::encode_set_gripper(1, 101),
    std::invalid_argument);
}

TEST(DownstreamTests, ClearFaultPayload)
{
  EXPECT_EQ(
    robot_base::protocol::encode_clear_fault(),
    (std::vector<uint8_t>{0xFF, 0xFF}));
  EXPECT_EQ(
    robot_base::protocol::encode_clear_fault(0x0202),
    (std::vector<uint8_t>{0x02, 0x02}));
}

TEST(DownstreamTests, CrashReadAndLogClearPayloads)
{
  EXPECT_EQ(
    robot_base::protocol::encode_crash_read(), (std::vector<uint8_t>{0xFF, 0x00}));
  EXPECT_EQ(
    robot_base::protocol::encode_crash_read(3, true),
    (std::vector<uint8_t>{3, 1}));
  EXPECT_EQ(
    robot_base::protocol::encode_log_clear(3),
    (std::vector<uint8_t>{3, 0xA5}));
}

TEST(FrameLimitsTests, MaxPayloadBuildsLongestFrame)
{
  std::vector<uint8_t> payload(robot_base::protocol::kPayloadMax, 0x5A);
  const auto frame = robot_base::protocol::build_frame(
    static_cast<uint8_t>(robot_base::protocol::Cmd::SET_VELOCITY), 0, payload);
  EXPECT_EQ(frame.size(), robot_base::protocol::kFrameMax);

  FrameParser parser;
  const auto frames = parser.feed(frame);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].payload.size(), robot_base::protocol::kPayloadMax);
}

TEST(FrameLimitsTests, OversizePayloadAndZeroCommandAreRejected)
{
  std::vector<uint8_t> payload(robot_base::protocol::kPayloadMax + 1, 0);
  EXPECT_THROW(
    robot_base::protocol::build_frame(0x01, 0, payload),
    std::invalid_argument);
  EXPECT_THROW(
    robot_base::protocol::build_frame(0x00, 0),
    std::invalid_argument);
}
