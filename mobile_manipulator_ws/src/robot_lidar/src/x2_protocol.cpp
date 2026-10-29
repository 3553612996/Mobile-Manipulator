#include "robot_lidar/x2_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace robot_lidar
{
namespace x2
{
namespace
{

uint16_t get_u16(const uint8_t * data)
{
  return static_cast<uint16_t>(
    static_cast<uint16_t>(data[0]) |
    static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8));
}

}  // namespace

uint16_t packet_checksum(const uint8_t * data, std::size_t len)
{
  uint16_t value = 0;
  if (len >= 2U) {
    value ^= get_u16(&data[0]);  // PH (0x55AA)
  }
  if (len >= 4U) {
    value ^= get_u16(&data[2]);  // CT | (LSN << 8)
  }
  if (len >= 6U) {
    value ^= get_u16(&data[4]);  // FSA, raw
  }
  if (len >= 8U) {
    value ^= get_u16(&data[6]);  // LSA, raw
  }
  // Skip CS at 8..9; XOR each sample word that follows.
  for (std::size_t i = kPacketFixed; i + 1U < len; i += 2U) {
    value ^= get_u16(&data[i]);
  }
  return value;
}

double angle_correction_deg(double distance_mm)
{
  if (distance_mm == 0.0) {
    return 0.0;
  }
  const double argument =
    21.8 * (155.3 - distance_mm) / (155.3 * distance_mm);
  return std::atan(argument) * 180.0 / kPi;
}

double normalize_deg_360(double deg)
{
  double value = std::fmod(deg, 360.0);
  if (value < 0.0) {
    value += 360.0;
  }
  return value;
}

double normalize_rad_pi(double rad)
{
  double value = std::fmod(rad + kPi, 2.0 * kPi);
  if (value < 0.0) {
    value += 2.0 * kPi;
  }
  return value - kPi;
}

std::optional<Packet> decode_packet(
  const uint8_t * data, std::size_t len, DecodeState * state)
{
  if (len < kPacketFixed + 2U) {
    return std::nullopt;
  }
  if (data[0] != kHeader0 || data[1] != kHeader1) {
    return std::nullopt;
  }

  const uint8_t packet_type = data[2];
  const uint8_t sample_count = data[3];
  if (sample_count == 0U || sample_count > kMaxSamples) {
    return std::nullopt;
  }
  if (len != kPacketFixed + 2U * sample_count) {
    return std::nullopt;
  }

  const uint16_t received_checksum = get_u16(&data[8]);
  if (received_checksum != packet_checksum(data, len)) {
    return std::nullopt;
  }

  const uint16_t fsa_raw = get_u16(&data[4]);
  const uint16_t lsa_raw = get_u16(&data[6]);
  // The low bit is the fixed check bit C=1. Reject a frame with it cleared:
  // it is either corrupt or a false header.
  if ((fsa_raw & 0x01U) == 0U || (lsa_raw & 0x01U) == 0U) {
    return std::nullopt;
  }

  Packet packet;
  packet.packet_type = packet_type;
  packet.sample_count = sample_count;
  packet.is_start = (packet_type & 0x01U) != 0U;
  packet.start_angle_deg = static_cast<double>(fsa_raw >> 1) / 64.0;
  packet.end_angle_deg = static_cast<double>(lsa_raw >> 1) / 64.0;

  if (packet.is_start) {
    // CT[7:1] carries the scan frequency; F = value / 10 Hz.
    packet.scan_frequency_hz =
      static_cast<double>(packet_type >> 1) / 10.0;
    return packet;
  }

  // Vendor SDK logic: a normal packet gives a fresh interval; a packet that
  // crosses the 270/90 seam gets +360; anything else reuses the previous
  // interval rather than inventing a full turn.
  double interval_deg = 0.0;
  if (sample_count > 1U) {
    if (packet.end_angle_deg >= packet.start_angle_deg) {
      interval_deg = (packet.end_angle_deg - packet.start_angle_deg) /
        static_cast<double>(sample_count - 1U);
      if (state != nullptr) {
        state->last_interval_deg = interval_deg;
      }
    } else if (packet.start_angle_deg > 270.0 && packet.end_angle_deg < 90.0) {
      interval_deg = (360.0 + packet.end_angle_deg - packet.start_angle_deg) /
        static_cast<double>(sample_count - 1U);
      if (state != nullptr) {
        state->last_interval_deg = interval_deg;
      }
    } else {
      interval_deg = state != nullptr ? state->last_interval_deg : 0.0;
    }
  }

  packet.points.reserve(sample_count);
  for (std::size_t i = 0; i < sample_count; ++i) {
    const uint16_t raw_distance = get_u16(&data[kPacketFixed + 2U * i]);
    const double distance_mm = static_cast<double>(raw_distance) / 4.0;

    double angle_deg = packet.start_angle_deg +
      interval_deg * static_cast<double>(i);
    angle_deg += angle_correction_deg(distance_mm);

    Point point;
    point.angle_deg = normalize_deg_360(angle_deg);
    point.distance_m = distance_mm / 1000.0;
    point.raw_distance = raw_distance;
    point.interference = (raw_distance & 0x01U) != 0U;
    packet.points.push_back(point);
  }

  return packet;
}

std::vector<Packet> PacketParser::feed(const uint8_t * data, std::size_t len)
{
  buffer_.insert(buffer_.end(), data, data + len);

  std::vector<Packet> packets;

  while (true) {
    std::size_t header_pos = buffer_.size();
    for (std::size_t i = 0; i + 1U < buffer_.size(); ++i) {
      if (buffer_[i] == kHeader0 && buffer_[i + 1U] == kHeader1) {
        header_pos = i;
        break;
      }
    }

    if (header_pos == buffer_.size()) {
      // No header. Keep a trailing 0xA5: it may be the first half of a header
      // split across two reads.
      if (!buffer_.empty() && buffer_.back() == kHeader0) {
        dropped_bytes_ += buffer_.size() - 1U;
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        dropped_bytes_ += buffer_.size();
        buffer_.clear();
      }
      break;
    }

    if (header_pos > 0U) {
      dropped_bytes_ += header_pos;
      buffer_.erase(buffer_.begin(), buffer_.begin() + header_pos);
    }

    if (buffer_.size() < 4U) {
      break;
    }

    const uint8_t sample_count = buffer_[3];
    if (sample_count == 0U || sample_count > kMaxSamples) {
      ++checksum_errors_;
      dropped_bytes_ += 1U;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t packet_len = kPacketFixed + 2U * sample_count;
    if (buffer_.size() < packet_len) {
      break;
    }

    const auto packet = decode_packet(buffer_.data(), packet_len, &state_);
    if (packet) {
      ++packets_ok_;
      packets.push_back(*packet);
      buffer_.erase(buffer_.begin(), buffer_.begin() + packet_len);
    } else {
      ++checksum_errors_;
      dropped_bytes_ += 1U;
      buffer_.erase(buffer_.begin());
    }
  }

  return packets;
}

std::vector<Packet> PacketParser::feed(const std::vector<uint8_t> & data)
{
  return feed(data.data(), data.size());
}

void PacketParser::reset()
{
  buffer_.clear();
}

}  // namespace x2
}  // namespace robot_lidar
