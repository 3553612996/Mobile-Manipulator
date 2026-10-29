#include "robot_base/protocol.hpp"

#include <cstdio>
#include <map>
#include <stdexcept>
#include <utility>

namespace robot_base
{
namespace protocol
{
namespace
{

void put_u16(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
}

void put_i16(std::vector<uint8_t> & out, int16_t value)
{
  put_u16(out, static_cast<uint16_t>(value));
}

void put_u32(std::vector<uint8_t> & out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xFFU));
}

uint16_t get_u16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

int16_t get_i16(const uint8_t * data)
{
  return static_cast<int16_t>(get_u16(data));
}

uint32_t get_u32(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int32_t get_i32(const uint8_t * data)
{
  return static_cast<int32_t>(get_u32(data));
}

}  // namespace

// ------------------------------------------------------------------- frame

bool Frame::is_ack() const
{
  return cmd >= kAckFlag && cmd < 0xC0U;
}

std::optional<uint8_t> Frame::acked_cmd() const
{
  if (!is_ack()) {
    return std::nullopt;
  }
  return static_cast<uint8_t>(cmd & 0x7FU);
}

std::optional<uint8_t> Frame::acked_seq() const
{
  if (!is_ack() || payload.size() < 1U) {
    return std::nullopt;
  }
  return payload[0];
}

std::optional<uint8_t> Frame::result() const
{
  if (!is_ack() || payload.size() < 2U) {
    return std::nullopt;
  }
  return payload[1];
}

uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
  crc ^= static_cast<uint16_t>(static_cast<uint16_t>(byte) << 8);
  for (int bit = 0; bit < 8; ++bit) {
    if ((crc & 0x8000U) != 0U) {
      crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
    } else {
      crc = static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

uint16_t crc16_ccitt(const uint8_t * data, std::size_t len)
{
  uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0; i < len; ++i) {
    crc = crc16_update(crc, data[i]);
  }
  return crc;
}

std::vector<uint8_t> build_frame(
  uint8_t cmd, uint8_t seq, const uint8_t * payload, std::size_t payload_len)
{
  if (cmd == 0U) {
    throw std::invalid_argument("command 0x00 is not valid");
  }
  if (payload_len > kPayloadMax) {
    throw std::invalid_argument("payload exceeds 64 bytes");
  }

  std::vector<uint8_t> frame;
  frame.reserve(5U + payload_len);
  frame.push_back(kHead1);
  frame.push_back(kHead2);
  frame.push_back(static_cast<uint8_t>(payload_len + 2U));
  frame.push_back(cmd);
  frame.push_back(seq);
  for (std::size_t i = 0; i < payload_len; ++i) {
    frame.push_back(payload[i]);
  }

  // CRC covers LEN..PAYLOAD, i.e. everything after the two head bytes.
  const uint16_t crc = crc16_ccitt(frame.data() + 2U, frame.size() - 2U);
  frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
  return frame;
}

std::vector<uint8_t> build_frame(
  uint8_t cmd, uint8_t seq, const std::vector<uint8_t> & payload)
{
  return build_frame(cmd, seq, payload.data(), payload.size());
}

std::vector<Frame> FrameParser::feed(const uint8_t * data, std::size_t len)
{
  std::vector<Frame> frames;

  for (std::size_t i = 0; i < len; ++i) {
    const uint8_t byte = data[i];

    switch (state_) {
      case State::Head1:
        if (byte == kHead1) {
          state_ = State::Head2;
        }
        break;

      case State::Head2:
        // A repeated 0xAA is still a plausible head, keep waiting.
        if (byte == kHead2) {
          state_ = State::Len;
        } else if (byte == kHead1) {
          state_ = State::Head2;
        } else {
          state_ = State::Head1;
        }
        break;

      case State::Len:
        if (byte >= kLenMin && byte <= kLenMax) {
          buffer_.assign(1U, byte);
          crc_ = crc16_update(0xFFFFU, byte);
          need_ = byte;
          state_ = State::Body;
        } else {
          // Resynchronise: this byte may itself be the next 0xAA.
          state_ = (byte == kHead1) ? State::Head2 : State::Head1;
        }
        break;

      case State::Body:
        buffer_.push_back(byte);
        crc_ = crc16_update(crc_, byte);
        --need_;
        if (need_ == 0U) {
          state_ = State::CrcLo;
        }
        break;

      case State::CrcLo:
        crc_lo_ = byte;
        state_ = State::CrcHi;
        break;

      case State::CrcHi: {
        const uint16_t rx_crc = static_cast<uint16_t>(
          crc_lo_ | static_cast<uint16_t>(static_cast<uint16_t>(byte) << 8));
        if (rx_crc == crc_) {
          ++frames_ok_;
          Frame frame;
          frame.cmd = buffer_[1];
          frame.seq = buffer_[2];
          frame.payload.assign(buffer_.begin() + 3, buffer_.end());
          frames.push_back(std::move(frame));
        } else {
          ++crc_errors_;
        }
        state_ = State::Head1;
        break;
      }
    }
  }

  return frames;
}

std::vector<Frame> FrameParser::feed(const std::vector<uint8_t> & data)
{
  return feed(data.data(), data.size());
}

void FrameParser::reset()
{
  state_ = State::Head1;
  crc_ = 0xFFFFU;
  need_ = 0U;
  crc_lo_ = 0U;
  buffer_.clear();
}

// ---------------------------------------------------------------- downstream

std::vector<uint8_t> encode_set_velocity(int16_t v_mm_s, int16_t w_mrad_s)
{
  std::vector<uint8_t> payload;
  payload.reserve(4U);
  put_i16(payload, v_mm_s);
  put_i16(payload, w_mrad_s);
  return payload;
}

std::vector<uint8_t> encode_set_arm_joints(const std::array<int16_t, 6> & angles_centi_deg)
{
  std::vector<uint8_t> payload;
  payload.reserve(12U);
  for (int16_t angle : angles_centi_deg) {
    put_i16(payload, angle);
  }
  return payload;
}

std::vector<uint8_t> encode_set_gripper(uint8_t action, uint8_t force)
{
  if (force > 100U) {
    throw std::invalid_argument("gripper force must be 0..100");
  }
  return std::vector<uint8_t>{action, force};
}

std::vector<uint8_t> encode_set_enable(bool enable)
{
  return std::vector<uint8_t>{static_cast<uint8_t>(enable ? 1U : 0U)};
}

std::vector<uint8_t> encode_set_param(uint8_t param_id, int32_t value)
{
  std::vector<uint8_t> payload;
  payload.reserve(5U);
  payload.push_back(param_id);
  put_u32(payload, static_cast<uint32_t>(value));
  return payload;
}

std::vector<uint8_t> encode_clear_fault(uint16_t code)
{
  std::vector<uint8_t> payload;
  payload.reserve(2U);
  put_u16(payload, code);
  return payload;
}

std::vector<uint8_t> encode_param_save(uint8_t scope)
{
  return std::vector<uint8_t>{scope};
}

std::vector<uint8_t> encode_param_reset(uint8_t scope)
{
  return std::vector<uint8_t>{scope, 0xA5U};
}

std::vector<uint8_t> encode_log_export(uint8_t source, uint8_t action, uint32_t from_seq)
{
  std::vector<uint8_t> payload;
  payload.reserve(6U);
  payload.push_back(source);
  payload.push_back(action);
  put_u32(payload, from_seq);
  return payload;
}

std::vector<uint8_t> encode_crash_read(uint8_t slot, bool mark_read)
{
  return std::vector<uint8_t>{slot, static_cast<uint8_t>(mark_read ? 1U : 0U)};
}

std::vector<uint8_t> encode_log_clear(uint8_t source)
{
  return std::vector<uint8_t>{source, 0xA5U};
}

// ------------------------------------------------------------------ upstream

std::optional<OdomData> decode_odom(const std::vector<uint8_t> & payload)
{
  if (payload.size() != 14U) {
    return std::nullopt;
  }
  OdomData odom;
  odom.x_mm = get_i32(&payload[0]);
  odom.y_mm = get_i32(&payload[4]);
  odom.theta_centi_deg = get_i16(&payload[8]);
  odom.vl_mm_s = get_i16(&payload[10]);
  odom.vr_mm_s = get_i16(&payload[12]);
  return odom;
}

std::optional<StatusData> decode_status(const std::vector<uint8_t> & payload)
{
  if (payload.size() != 8U) {
    return std::nullopt;
  }
  StatusData status;
  status.state = payload[0];
  status.fault = get_u16(&payload[1]);
  status.battery_mv = get_u16(&payload[3]);
  status.temperature_c = static_cast<int8_t>(payload[5]);
  status.jitter_us = get_u16(&payload[6]);
  return status;
}

std::optional<EventData> decode_event(const std::vector<uint8_t> & payload)
{
  if (payload.size() != 6U) {
    return std::nullopt;
  }
  EventData event;
  event.code = get_u16(&payload[0]);
  event.timestamp_ms = get_u32(&payload[2]);
  return event;
}

std::optional<HeartbeatData> decode_heartbeat(const std::vector<uint8_t> & payload)
{
  if (payload.size() != 3U) {
    return std::nullopt;
  }
  HeartbeatData beat;
  beat.state = payload[0];
  beat.counter = get_u16(&payload[1]);
  return beat;
}

std::optional<ArmData> decode_arm(const std::vector<uint8_t> & payload)
{
  if (payload.size() != 15U) {
    return std::nullopt;
  }
  ArmData arm;
  for (std::size_t i = 0; i < arm.angles_centi_deg.size(); ++i) {
    arm.angles_centi_deg[i] = get_i16(&payload[i * 2U]);
  }
  arm.gripper_pos = get_i16(&payload[12]);
  arm.gripper_state = payload[14];
  return arm;
}

std::optional<WheelSpeedData> decode_wheel_speed(const std::vector<uint8_t> & payload)
{
  if (payload.size() != 8U) {
    return std::nullopt;
  }
  WheelSpeedData speed;
  speed.fl_mm_s = get_i16(&payload[0]);
  speed.fr_mm_s = get_i16(&payload[2]);
  speed.rl_mm_s = get_i16(&payload[4]);
  speed.rr_mm_s = get_i16(&payload[6]);
  return speed;
}

std::optional<WheelDutyData> decode_wheel_duty(const std::vector<uint8_t> & payload)
{
  if (payload.size() != 8U) {
    return std::nullopt;
  }
  WheelDutyData duty;
  duty.fl_permille = get_i16(&payload[0]);
  duty.fr_permille = get_i16(&payload[2]);
  duty.rl_permille = get_i16(&payload[4]);
  duty.rr_permille = get_i16(&payload[6]);
  return duty;
}

std::optional<StorageData> decode_storage(const std::vector<uint8_t> & payload)
{
  if (payload.size() != 24U) {
    return std::nullopt;
  }
  StorageData storage;
  storage.flags = payload[0];
  storage.log_level = payload[1];
  storage.param_seq = get_u16(&payload[2]);
  storage.calib_seq = get_u16(&payload[4]);
  storage.boot_total = get_u32(&payload[8]);
  storage.log_used_bytes = get_u32(&payload[12]);
  storage.log_erase_count = get_u32(&payload[16]);
  storage.last_save_s = get_u32(&payload[20]);
  return storage;
}

std::optional<LogData> decode_log(const std::vector<uint8_t> & payload)
{
  if (payload.size() < 15U) {
    return std::nullopt;
  }
  LogData log;
  log.source = payload[0];
  log.level = payload[1];
  log.tag = payload[2];
  log.flags = payload[3];
  log.timestamp_s = get_u32(&payload[4]);
  log.seq = get_u32(&payload[8]);
  log.code = get_u16(&payload[12]);
  const uint8_t text_len = payload[14];
  if (payload.size() < 15U + text_len) {
    return std::nullopt;
  }
  log.text.assign(
    reinterpret_cast<const char *>(&payload[15]),
    static_cast<std::size_t>(text_len));
  return log;
}

std::optional<CrashChunk> decode_crash(const std::vector<uint8_t> & payload)
{
  if (payload.size() < 7U) {
    return std::nullopt;
  }
  CrashChunk chunk;
  chunk.slot = payload[0];
  chunk.flags = payload[1];
  chunk.offset = get_u16(&payload[2]);
  chunk.total_len = get_u16(&payload[4]);
  const uint8_t length = payload[6];
  if (payload.size() < 7U + length) {
    return std::nullopt;
  }
  chunk.data.assign(payload.begin() + 7, payload.begin() + 7 + length);
  return chunk;
}

// -------------------------------------------------------------------- names

std::string state_name(uint8_t state)
{
  switch (static_cast<State>(state)) {
    case State::INIT:
      return "INIT";
    case State::IDLE:
      return "IDLE";
    case State::MANUAL:
      return "MANUAL";
    case State::AUTO:
      return "AUTO";
    case State::ESTOP:
      return "ESTOP";
    case State::FAULT:
      return "FAULT";
  }
  return "0x" + std::to_string(state);
}

std::string result_name(uint8_t result)
{
  switch (static_cast<Result>(result)) {
    case Result::OK:
      return "成功";
    case Result::OUT_OF_RANGE:
      return "参数越界";
    case Result::BAD_STATE:
      return "当前状态不允许";
    case Result::UNKNOWN_CMD:
      return "未知命令";
    case Result::FAILED:
      return "执行失败";
    case Result::BUSY:
      return "忙";
    case Result::STORAGE_UNAVAILABLE:
      return "存储不可用";
  }
  char text[8];
  std::snprintf(text, sizeof(text), "0x%02X", result);
  return text;
}

std::string fault_name(uint16_t code)
{
  static const std::map<uint16_t, const char *> names = {
    {0x0101, "电机过流"},
    {0x0102, "编码器异常"},
    {0x0201, "CRC 错误率超限"},
    {0x0202, "通信超时"},
    {0x0301, "关节超限位"},
    {0x0302, "关节堵转"},
    {0x0303, "轨迹执行异常"},
    {0x0401, "电池低压"},
    {0x0402, "温度过高"},
    {0x0501, "急停触发"},
    {0x0601, "参数区无效"},
    {0x0602, "标定区无效"},
    {0x0603, "存储写入失败"},
    {0x0604, "分区表/镜像无效"},
  };
  const auto found = names.find(code);
  if (found != names.end()) {
    return found->second;
  }
  char text[8];
  std::snprintf(text, sizeof(text), "0x%04X", code);
  return text;
}

}  // namespace protocol
}  // namespace robot_base
