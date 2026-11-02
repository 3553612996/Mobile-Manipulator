// Wire format for the RK3566 <-> STM32 link (docs/architecture/protocol.md v1).
//
// This is the only place in the ROS 2 workspace that knows the frame layout,
// CRC parameters and payload byte order. It is deliberately free of rclcpp so
// the codec can be unit tested on a PC without a serial port, and can later be
// reused by a standalone transport node.

#ifndef ROBOT_BASE__PROTOCOL_HPP_
#define ROBOT_BASE__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot_base
{
namespace protocol
{

constexpr uint8_t kHead1 = 0xAA;
constexpr uint8_t kHead2 = 0x55;
// ESTOP has no payload, so LEN = CMD + SEQ = 2 is the shortest legal frame.
constexpr std::size_t kLenMin = 2;
constexpr std::size_t kLenMax = 66;
constexpr std::size_t kPayloadMax = kLenMax - 2;
constexpr std::size_t kFrameMax = 5 + kLenMax;
constexpr uint8_t kAckFlag = 0x80;
constexpr uint16_t kClearAllFaults = 0xFFFF;

enum class Cmd : uint8_t
{
  SET_VELOCITY = 0x01,
  SET_ARM_JOINTS = 0x02,
  SET_GRIPPER = 0x03,
  SET_ENABLE = 0x04,
  SET_PARAM = 0x05,
  ESTOP = 0x06,
  CLEAR_FAULT = 0x07,
  PARAM_SAVE = 0x08,
  PARAM_RESET = 0x09,
  STORAGE_QUERY = 0x0A,
  LOG_EXPORT = 0x0B,
  CRASH_READ = 0x0C,
  LOG_CLEAR = 0x0D,
};

enum class Report : uint8_t
{
  ODOM = 0xC1,
  ARM = 0xC2,
  STATUS = 0xC3,
  EVENT = 0xC4,
  HEARTBEAT = 0xC5,
  WHEEL_SPEED = 0xC6,
  WHEEL_DUTY = 0xC7,
  STORAGE = 0xC8,
  LOG = 0xC9,
  CRASH = 0xCA,
};

enum class State : uint8_t
{
  INIT = 0,
  IDLE = 1,
  MANUAL = 2,
  AUTO = 3,
  ESTOP = 4,
  FAULT = 5,
};

enum class Result : uint8_t
{
  OK = 0x00,
  OUT_OF_RANGE = 0x01,
  BAD_STATE = 0x02,
  UNKNOWN_CMD = 0x03,
  FAILED = 0x04,
  BUSY = 0x05,
  STORAGE_UNAVAILABLE = 0x06,
};

struct Frame
{
  uint8_t cmd{0};
  uint8_t seq{0};
  std::vector<uint8_t> payload;

  bool is_ack() const;
  std::optional<uint8_t> acked_cmd() const;
  std::optional<uint8_t> acked_seq() const;
  std::optional<uint8_t> result() const;
};

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection.
uint16_t crc16_update(uint16_t crc, uint8_t byte);
uint16_t crc16_ccitt(const uint8_t * data, std::size_t len);

// Raises std::invalid_argument on a zero command or a payload over 64 bytes.
std::vector<uint8_t> build_frame(
  uint8_t cmd, uint8_t seq, const uint8_t * payload = nullptr,
  std::size_t payload_len = 0);
std::vector<uint8_t> build_frame(
  uint8_t cmd, uint8_t seq, const std::vector<uint8_t> & payload);

// Incremental parser (protocol.md section 9). feed() accepts any chunking,
// including one byte at a time. An invalid LEN or CRC returns to the
// head-search state, so a corrupt frame cannot cascade.
class FrameParser
{
public:
  std::vector<Frame> feed(const uint8_t * data, std::size_t len);
  std::vector<Frame> feed(const std::vector<uint8_t> & data);

  // Drops a partial frame. Counters are deliberately kept: they describe the
  // link, not the current frame.
  void reset();

  uint64_t frames_ok() const {return frames_ok_;}
  uint64_t crc_errors() const {return crc_errors_;}

private:
  enum class State
  {
    Head1,
    Head2,
    Len,
    Body,
    CrcLo,
    CrcHi,
  };

  State state_{State::Head1};
  uint16_t crc_{0xFFFF};
  uint16_t need_{0};
  uint8_t crc_lo_{0};
  std::vector<uint8_t> buffer_;
  uint64_t frames_ok_{0};
  uint64_t crc_errors_{0};
};

// ---------------------------------------------------------------- downstream

std::vector<uint8_t> encode_set_velocity(int16_t v_mm_s, int16_t w_mrad_s);
std::vector<uint8_t> encode_set_arm_joints(const std::array<int16_t, 6> & angles_centi_deg);
std::vector<uint8_t> encode_set_gripper(uint8_t action, uint8_t force);
std::vector<uint8_t> encode_set_enable(bool enable);
std::vector<uint8_t> encode_set_param(uint8_t param_id, int32_t value);
std::vector<uint8_t> encode_clear_fault(uint16_t code = kClearAllFaults);
std::vector<uint8_t> encode_param_save(uint8_t scope = 0);
std::vector<uint8_t> encode_param_reset(uint8_t scope = 0);
std::vector<uint8_t> encode_log_export(uint8_t source, uint8_t action, uint32_t from_seq);
std::vector<uint8_t> encode_crash_read(uint8_t slot = 0xFF, bool mark_read = false);
std::vector<uint8_t> encode_log_clear(uint8_t source);

// ------------------------------------------------------------------ upstream

struct OdomData
{
  int32_t x_mm{0};
  int32_t y_mm{0};
  // Range is +-327.67 deg; the firmware wraps heading to (-180, 180].
  int16_t theta_centi_deg{0};
  int16_t vl_mm_s{0};
  int16_t vr_mm_s{0};
};

struct StatusData
{
  uint8_t state{0};
  uint16_t fault{0};
  uint16_t battery_mv{0};
  int8_t temperature_c{0};
  uint16_t jitter_us{0};
};

struct EventData
{
  uint16_t code{0};
  uint32_t timestamp_ms{0};
};

struct HeartbeatData
{
  uint8_t state{0};
  uint16_t counter{0};
};

struct ArmData
{
  std::array<int16_t, 6> angles_centi_deg{};
  int16_t gripper_pos{0};
  uint8_t gripper_state{0};
};

struct WheelSpeedData
{
  int16_t fl_mm_s{0};
  int16_t fr_mm_s{0};
  int16_t rl_mm_s{0};
  int16_t rr_mm_s{0};
};

struct WheelDutyData
{
  int16_t fl_permille{0};
  int16_t fr_permille{0};
  int16_t rl_permille{0};
  int16_t rr_permille{0};
};

struct StorageData
{
  uint8_t flags{0};
  uint8_t log_level{0};
  uint16_t param_seq{0};
  uint16_t calib_seq{0};
  uint32_t boot_total{0};
  uint32_t log_used_bytes{0};
  uint32_t log_erase_count{0};
  uint32_t last_save_s{0};

  bool clean() const {return (flags & 0x01U) != 0U;}
  bool param_valid() const {return (flags & 0x02U) != 0U;}
  bool calib_valid() const {return (flags & 0x04U) != 0U;}
  bool save_failed() const {return (flags & 0x08U) != 0U;}
  bool image_a_valid() const {return (flags & 0x10U) != 0U;}
  bool image_b_valid() const {return (flags & 0x20U) != 0U;}
  bool crash_unread() const {return (flags & 0x40U) != 0U;}
  bool log_wrapped() const {return (flags & 0x80U) != 0U;}
};

struct LogData
{
  uint8_t source{0};
  uint8_t level{0};
  uint8_t tag{0};
  uint8_t flags{0};
  uint32_t timestamp_s{0};
  uint32_t seq{0};
  uint16_t code{0};
  std::string text;

  bool more() const {return (flags & 0x01U) != 0U;}
  bool end() const {return (flags & 0x02U) != 0U;}
};

struct CrashChunk
{
  uint8_t slot{0};
  uint8_t flags{0};
  uint16_t offset{0};
  uint16_t total_len{0};
  std::vector<uint8_t> data;

  bool more() const {return (flags & 0x01U) != 0U;}
};

std::optional<OdomData> decode_odom(const std::vector<uint8_t> & payload);
std::optional<StatusData> decode_status(const std::vector<uint8_t> & payload);
std::optional<EventData> decode_event(const std::vector<uint8_t> & payload);
std::optional<HeartbeatData> decode_heartbeat(const std::vector<uint8_t> & payload);
std::optional<ArmData> decode_arm(const std::vector<uint8_t> & payload);
std::optional<WheelSpeedData> decode_wheel_speed(const std::vector<uint8_t> & payload);
std::optional<WheelDutyData> decode_wheel_duty(const std::vector<uint8_t> & payload);
std::optional<StorageData> decode_storage(const std::vector<uint8_t> & payload);
std::optional<LogData> decode_log(const std::vector<uint8_t> & payload);
std::optional<CrashChunk> decode_crash(const std::vector<uint8_t> & payload);

// Human-readable names for logs and diagnostics. Unknown values are rendered
// as 0xNN so a protocol mismatch is visible instead of silently "unknown".
std::string state_name(uint8_t state);
std::string fault_name(uint16_t code);
std::string result_name(uint8_t result);

}  // namespace protocol
}  // namespace robot_base

#endif  // ROBOT_BASE__PROTOCOL_HPP_
