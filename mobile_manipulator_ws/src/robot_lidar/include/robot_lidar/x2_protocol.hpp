// YDLIDAR X2 serial protocol (X2 开发手册, DOC#01.13.000101; cross-checked
// against the vendor SDK, ydlidar_ros2/sdk/src/ydlidar_driver.cpp).
//
// Packet layout:
//   AA 55 | CT | LSN | FSA(2) | LSA(2) | CS(2) | S1(2) ... Sn(2)
//   - The manual text says PH=0x55AA low byte first, which is AA 55. The
//     "A5 5A" in the manual figure is a typo; the vendor SDK matches AA 55.
//   - FSA/LSA are little-endian; angle = (raw >> 1) / 64 degrees; bit0 is
//     the fixed check bit and must be 1.
//   - Si is little-endian; distance = Si / 4 mm.
//   - CS is the 16-bit XOR of PH, the CT/LSN word, FSA, LSA and every sample
//     word. It sits before the samples, not at the end of the frame.
//   - LSN is the number of samples; total packet size is 10 + 2*LSN.
//   - CT bit0 = 0: point cloud packet; bit0 = 1: lap start packet.

#ifndef ROBOT_LIDAR__X2_PROTOCOL_HPP_
#define ROBOT_LIDAR__X2_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace robot_lidar
{
namespace x2
{

constexpr uint8_t kHeader0 = 0xAA;
constexpr uint8_t kHeader1 = 0x55;
constexpr std::size_t kPacketFixed = 10;  // header..checksum
// The X2 sends roughly 40 samples per packet at 6 Hz; 128 leaves headroom and
// bounds the parser buffer at 266 bytes.
constexpr std::size_t kMaxSamples = 128;
constexpr std::size_t kMaxPacket = kPacketFixed + 2 * kMaxSamples;
constexpr double kPi = 3.14159265358979323846;

struct Point
{
  // Raw X2 angle after the second-level correction, in degrees [0, 360).
  // X2 angles increase clockwise; the ROS conversion happens in the node.
  double angle_deg{0.0};
  double distance_m{0.0};
  uint16_t raw_distance{0};
  // The manual says the interference flag lives in the LSB of Si. It is kept
  // for diagnostics but not filtered by the default driver.
  bool interference{false};
};

struct Packet
{
  uint8_t packet_type{0};
  uint8_t sample_count{0};
  // Uncorrected first-level angles; used for lap-wrap detection.
  double start_angle_deg{0.0};
  double end_angle_deg{0.0};
  bool is_start{false};
  double scan_frequency_hz{0.0};
  std::vector<Point> points;
};

// 16-bit word XOR of PH, CT/LSN, FSA, LSA and every sample word. The two
// checksum bytes at offsets 8 and 9 are excluded.
uint16_t packet_checksum(const uint8_t * data, std::size_t len);

// Carries the last valid sample-angle interval between packets. The vendor
// SDK reuses it when LSA < FSA but the packet is not at the 270/90 seam.
struct DecodeState
{
  double last_interval_deg{0.0};
};

// Decodes exactly one packet of `len` bytes. Returns nullopt on a bad length,
// bad LSN, missing FSA/LSA check bit or checksum mismatch.
std::optional<Packet> decode_packet(
  const uint8_t * data, std::size_t len, DecodeState * state = nullptr);

// Incremental parser. Any chunking is accepted; on a bad packet it drops one
// byte and rescans, so a corrupt frame cannot cascade.
class PacketParser
{
public:
  std::vector<Packet> feed(const uint8_t * data, std::size_t len);
  std::vector<Packet> feed(const std::vector<uint8_t> & data);

  // Drops a partial packet. Counters are cumulative and are not cleared.
  void reset();

  uint64_t packets_ok() const {return packets_ok_;}
  uint64_t checksum_errors() const {return checksum_errors_;}
  uint64_t dropped_bytes() const {return dropped_bytes_;}

private:
  std::vector<uint8_t> buffer_;
  DecodeState state_;
  uint64_t packets_ok_{0};
  uint64_t checksum_errors_{0};
  uint64_t dropped_bytes_{0};
};

// Second-level angle correction from the manual. Distance is in mm and the
// result is in degrees. The manual's formula image looks like a "+" in the
// denominator but the text layer and both worked examples use multiplication:
//   atan(21.8 * (155.3 - d) / (155.3 * d))
// d=1000 -> -6.7622 deg, d=8000 -> -7.8374 deg, matching the manual.
double angle_correction_deg(double distance_mm);

double normalize_deg_360(double deg);
double normalize_rad_pi(double rad);

}  // namespace x2
}  // namespace robot_lidar

#endif  // ROBOT_LIDAR__X2_PROTOCOL_HPP_
