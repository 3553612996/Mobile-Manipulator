// Protocol tests use synthetic packets built from the X2 development manual.
// The two worked examples for the second-level angle correction are checked
// explicitly because the manual's formula image and text disagree.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "robot_lidar/x2_protocol.hpp"

namespace
{

using robot_lidar::x2::PacketParser;

uint16_t angle_raw(double deg)
{
  // The low bit is the fixed check bit C=1; the upper 15 bits hold deg*64.
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(std::lround(deg * 64.0)) << 1) | 1U);
}

std::vector<uint8_t> make_packet(
  uint8_t ct, double fsa_deg, double lsa_deg,
  const std::vector<uint16_t> & samples)
{
  const auto lsn = static_cast<uint8_t>(samples.size());
  std::vector<uint8_t> packet(robot_lidar::x2::kPacketFixed + 2U * samples.size(), 0U);
  packet[0] = robot_lidar::x2::kHeader0;
  packet[1] = robot_lidar::x2::kHeader1;
  packet[2] = ct;
  packet[3] = lsn;

  const uint16_t fsa = angle_raw(fsa_deg);
  const uint16_t lsa = angle_raw(lsa_deg);
  packet[4] = static_cast<uint8_t>(fsa & 0xFFU);
  packet[5] = static_cast<uint8_t>((fsa >> 8) & 0xFFU);
  packet[6] = static_cast<uint8_t>(lsa & 0xFFU);
  packet[7] = static_cast<uint8_t>((lsa >> 8) & 0xFFU);

  for (std::size_t i = 0; i < samples.size(); ++i) {
    packet[robot_lidar::x2::kPacketFixed + 2U * i] =
      static_cast<uint8_t>(samples[i] & 0xFFU);
    packet[robot_lidar::x2::kPacketFixed + 2U * i + 1U] =
      static_cast<uint8_t>((samples[i] >> 8) & 0xFFU);
  }

  const uint16_t cs = robot_lidar::x2::packet_checksum(
    packet.data(), packet.size());
  packet[8] = static_cast<uint8_t>(cs & 0xFFU);
  packet[9] = static_cast<uint8_t>((cs >> 8) & 0xFFU);
  return packet;
}

}  // namespace

TEST(ChecksumTests, Is16BitWordXor)
{
  const auto raw = make_packet(0x00, 0.0, 10.0, {4000, 4000});

  uint16_t expected = 0x55AA;  // PH word
  expected ^= static_cast<uint16_t>(raw[2] | (raw[3] << 8));  // CT | LSN<<8
  expected ^= static_cast<uint16_t>(raw[4] | (raw[5] << 8));  // FSA
  expected ^= static_cast<uint16_t>(raw[6] | (raw[7] << 8));  // LSA
  for (std::size_t i = robot_lidar::x2::kPacketFixed; i + 1U < raw.size(); i += 2U) {
    expected ^= static_cast<uint16_t>(raw[i] | (raw[i + 1U] << 8));
  }

  EXPECT_EQ(robot_lidar::x2::packet_checksum(raw.data(), raw.size()), expected);
  EXPECT_EQ(static_cast<uint16_t>(raw[8] | (raw[9] << 8)), expected);
}

TEST(DecodeTests, PointPacketFieldsAndDistances)
{
  // FSA 0 deg, LSA 10 deg, 5 samples at 1000 mm.
  const auto raw = make_packet(0x00, 0.0, 10.0, {4000, 4000, 4000, 4000, 4000});
  const auto packet = robot_lidar::x2::decode_packet(raw.data(), raw.size());
  ASSERT_TRUE(packet.has_value());
  EXPECT_FALSE(packet->is_start);
  EXPECT_EQ(packet->sample_count, 5U);
  EXPECT_NEAR(packet->start_angle_deg, 0.0, 1e-6);
  EXPECT_NEAR(packet->end_angle_deg, 10.0, 1e-6);
  ASSERT_EQ(packet->points.size(), 5U);

  EXPECT_NEAR(packet->points[0].distance_m, 1.0, 1e-9);
  EXPECT_NEAR(packet->points[4].distance_m, 1.0, 1e-9);
  EXPECT_EQ(packet->points[0].raw_distance, 4000U);
}

TEST(DecodeTests, SecondLevelCorrectionMatchesManualExamples)
{
  // Manual example: d=1000 -> -6.7622 deg; d=8000 -> -7.8374 deg.
  EXPECT_NEAR(robot_lidar::x2::angle_correction_deg(1000.0), -6.7622, 1e-4);
  EXPECT_NEAR(robot_lidar::x2::angle_correction_deg(8000.0), -7.8374, 1e-4);
  EXPECT_NEAR(robot_lidar::x2::angle_correction_deg(0.0), 0.0, 1e-12);

  const auto raw = make_packet(0x00, 0.0, 10.0, {4000, 4000});
  const auto packet = robot_lidar::x2::decode_packet(raw.data(), raw.size());
  ASSERT_TRUE(packet.has_value());
  ASSERT_EQ(packet->points.size(), 2U);
  EXPECT_NEAR(packet->points[0].angle_deg, 353.2378, 1e-3);
  EXPECT_NEAR(packet->points[1].angle_deg, 3.2378, 1e-3);

  const auto far = make_packet(0x00, 0.0, 0.0, {32000, 32000});
  const auto far_packet = robot_lidar::x2::decode_packet(far.data(), far.size());
  ASSERT_TRUE(far_packet.has_value());
  ASSERT_EQ(far_packet->points.size(), 2U);
  EXPECT_NEAR(far_packet->points[0].angle_deg, 352.1626, 1e-3);
}

TEST(DecodeTests, StartPacketCarriesScanFrequency)
{
  // CT bit0=1, CT[7:1]=60 -> 6.0 Hz.
  const auto raw = make_packet(0x79, 0.0, 0.0, {0});
  const auto packet = robot_lidar::x2::decode_packet(raw.data(), raw.size());
  ASSERT_TRUE(packet.has_value());
  EXPECT_TRUE(packet->is_start);
  EXPECT_NEAR(packet->scan_frequency_hz, 6.0, 1e-9);
  EXPECT_TRUE(packet->points.empty());
}

TEST(DecodeTests, ZeroDistanceAndSeamPacket)
{
  // A packet crossing 360 -> 0: FSA 355, LSA 5, the first-level difference
  // must be +10 degrees, not -350.
  const auto raw = make_packet(0x00, 355.0, 5.0, {0, 4000, 4000});
  const auto packet = robot_lidar::x2::decode_packet(raw.data(), raw.size());
  ASSERT_TRUE(packet.has_value());
  ASSERT_EQ(packet->points.size(), 3U);
  EXPECT_EQ(packet->points[0].raw_distance, 0U);
  EXPECT_NEAR(packet->points[0].distance_m, 0.0, 1e-12);
  // Second sample is 5 deg along the clockwise arc from 355: 360 -> 0.
  EXPECT_NEAR(packet->points[1].angle_deg, 353.2378, 1e-3);
}

TEST(DecodeTests, RejectsBadChecksumAndBadLength)
{
  auto bad = make_packet(0x00, 0.0, 10.0, {4000, 4000});
  bad[12] ^= 0x01;
  EXPECT_FALSE(robot_lidar::x2::decode_packet(bad.data(), bad.size()).has_value());

  auto raw = make_packet(0x00, 0.0, 10.0, {4000, 4000});
  EXPECT_FALSE(
    robot_lidar::x2::decode_packet(raw.data(), raw.size() - 1U).has_value());
}

TEST(DecodeTests, RejectsMissingAngleCheckBit)
{
  auto raw = make_packet(0x00, 0.0, 10.0, {4000, 4000});
  raw[4] = static_cast<uint8_t>(raw[4] & ~0x01U);  // clear FSA C bit
  const uint16_t cs = robot_lidar::x2::packet_checksum(raw.data(), raw.size());
  raw[8] = static_cast<uint8_t>(cs & 0xFFU);
  raw[9] = static_cast<uint8_t>((cs >> 8) & 0xFFU);
  EXPECT_FALSE(robot_lidar::x2::decode_packet(raw.data(), raw.size()).has_value());
}

TEST(ParserTests, RecoversAfterBadPacket)
{
  auto bad = make_packet(0x00, 0.0, 10.0, {4000, 4000});
  bad[12] ^= 0x01;
  const auto good = make_packet(0x00, 10.0, 20.0, {800, 800});

  std::vector<uint8_t> stream = bad;
  stream.insert(stream.end(), good.begin(), good.end());

  PacketParser parser;
  const auto packets = parser.feed(stream);
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_NEAR(packets[0].start_angle_deg, 10.0, 1e-6);
  EXPECT_GE(parser.checksum_errors(), 1U);
}

TEST(ParserTests, ByteAtATimeEqualsBulk)
{
  std::vector<uint8_t> stream = make_packet(0x00, 0.0, 10.0, {4000, 4000});
  const auto second = make_packet(0x00, 10.0, 20.0, {800, 800});
  stream.insert(stream.end(), second.begin(), second.end());

  PacketParser bulk_parser;
  const auto bulk = bulk_parser.feed(stream);

  PacketParser drip_parser;
  std::vector<robot_lidar::x2::Packet> drip;
  for (uint8_t byte : stream) {
    const auto frames = drip_parser.feed(&byte, 1U);
    drip.insert(drip.end(), frames.begin(), frames.end());
  }

  ASSERT_EQ(bulk.size(), 2U);
  ASSERT_EQ(drip.size(), 2U);
  for (std::size_t i = 0; i < bulk.size(); ++i) {
    EXPECT_NEAR(bulk[i].start_angle_deg, drip[i].start_angle_deg, 1e-9);
    EXPECT_EQ(bulk[i].points.size(), drip[i].points.size());
  }
}

TEST(ParserTests, HeaderInsidePayloadDoesNotSplitThePacket)
{
  // 0x55AA as a sample value puts the header bytes AA 55 inside the payload.
  const auto raw = make_packet(0x00, 0.0, 10.0, {0x55AA, 4000});
  PacketParser parser;
  const auto packets = parser.feed(raw);
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(packets[0].points[0].raw_distance, 0x55AAU);
}

TEST(ParserTests, ReusesPreviousIntervalOutsideTheSeam)
{
  PacketParser parser;
  const auto first = make_packet(0x00, 0.0, 10.0, {0, 0});
  const auto second = make_packet(0x00, 100.0, 90.0, {0, 0});

  std::vector<uint8_t> stream = first;
  stream.insert(stream.end(), second.begin(), second.end());

  const auto packets = parser.feed(stream);
  ASSERT_EQ(packets.size(), 2U);
  // LSA < FSA but this is not the 270/90 seam, so the vendor SDK reuses the
  // previous 10-degree interval instead of adding a full turn.
  ASSERT_EQ(packets[1].points.size(), 2U);
  EXPECT_NEAR(packets[1].points[0].angle_deg, 100.0, 1e-6);
  EXPECT_NEAR(packets[1].points[1].angle_deg, 110.0, 1e-6);
}

TEST(ParserTests, ZeroLsnIsDroppedAndNextPacketParses)
{
  std::vector<uint8_t> junk = {0xA5, 0x5A, 0x00, 0x00};
  const auto good = make_packet(0x00, 0.0, 10.0, {4000});
  junk.insert(junk.end(), good.begin(), good.end());

  PacketParser parser;
  const auto packets = parser.feed(junk);
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(packets[0].points.size(), 1U);
}

TEST(ParserTests, ResetDiscardsHalfPacket)
{
  const auto raw = make_packet(0x00, 0.0, 10.0, {4000, 4000, 4000});
  PacketParser parser;
  EXPECT_TRUE(parser.feed(raw.data(), 5U).empty());
  parser.reset();
  EXPECT_EQ(parser.feed(raw).size(), 1U);
}

TEST(ParserTests, RandomStreamNeverThrows)
{
  std::mt19937 rng(2026);
  std::uniform_int_distribution<int> byte_dist(0, 255);
  std::uniform_int_distribution<int> length_dist(1, 40);

  PacketParser parser;
  for (int i = 0; i < 2000; ++i) {
    std::vector<uint8_t> chunk(static_cast<std::size_t>(length_dist(rng)));
    for (auto & byte : chunk) {
      byte = static_cast<uint8_t>(byte_dist(rng));
    }
    parser.feed(chunk);
  }

  const auto good = make_packet(0x00, 0.0, 10.0, {4000});
  EXPECT_EQ(parser.feed(good).size(), 1U);
}

TEST(ParserTests, MaximumSampleCountIsAccepted)
{
  std::vector<uint16_t> samples(robot_lidar::x2::kMaxSamples, 4000);
  const auto raw = make_packet(0x00, 0.0, 359.0, samples);
  ASSERT_EQ(raw.size(), robot_lidar::x2::kMaxPacket);

  PacketParser parser;
  const auto packets = parser.feed(raw);
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(packets[0].points.size(), robot_lidar::x2::kMaxSamples);
}
