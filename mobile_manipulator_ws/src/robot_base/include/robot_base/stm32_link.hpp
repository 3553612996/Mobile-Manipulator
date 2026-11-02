// Owns the UART and the frame codec. robot_base is the only ROS package that
// opens the STM32 port; when robot_arm joins it must go through this class
// (or through a transport node promoting it), never through a second open().
//
// Transmission policy lives here only at the level of "one writer, one SEQ
// counter, ACK matching by SEQ". Scheduling (20 ms velocity stream, 500 ms
// link watchdog) belongs to the node.

#ifndef ROBOT_BASE__STM32_LINK_HPP_
#define ROBOT_BASE__STM32_LINK_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "robot_base/protocol.hpp"
#include "robot_base/serial_io.hpp"

namespace robot_base
{

class Stm32Link
{
public:
  using FrameCallback = std::function<void(const protocol::Frame &)>;

  struct AckResult
  {
    bool received{false};
    uint8_t result{0};
  };

  Stm32Link(std::unique_ptr<SerialIo> io, FrameCallback on_frame);
  ~Stm32Link();

  Stm32Link(const Stm32Link &) = delete;
  Stm32Link & operator=(const Stm32Link &) = delete;

  bool open(const std::string & port, int baud, std::string * error = nullptr);
  void close();
  bool is_open() const {return open_.load();}

  void set_link_timeout(std::chrono::milliseconds timeout);
  // False when no frame has arrived within the link timeout. The node uses
  // this to stop sending; the MCU stops itself after 200 ms without velocity.
  bool link_alive() const;
  bool has_frame() const {return has_frame_.load();}
  int64_t last_frame_age_ms() const;

  // Fire-and-forget. Refused when the link is considered lost.
  bool send(
    uint8_t cmd, const std::vector<uint8_t> & payload,
    std::string * error = nullptr);

  // Sends, waits for the ACK, and retransmits the *same* frame (same SEQ) up
  // to `retries` times. The firmware answers a repeated SEQ from its cached
  // result instead of executing the command twice.
  AckResult send_with_ack(
    uint8_t cmd, const std::vector<uint8_t> & payload,
    std::chrono::milliseconds timeout, int retries,
    std::string * error = nullptr);

  uint64_t rx_frames() const {return rx_frames_.load();}
  uint64_t crc_errors() const {return crc_errors_.load();}
  uint64_t tx_frames() const {return tx_frames_.load();}
  uint64_t ack_timeouts() const {return ack_timeouts_.load();}

private:
  struct AckWaiter
  {
    bool done{false};
    uint8_t result{0};
  };

  void rx_loop();
  void dispatch_ack(const protocol::Frame & frame);
  bool write_locked(const std::vector<uint8_t> & frame, std::string * error);

  std::unique_ptr<SerialIo> io_;
  FrameCallback on_frame_;

  mutable std::mutex lifecycle_mutex_;
  std::thread rx_thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> open_{false};
  protocol::FrameParser parser_;
  uint64_t last_crc_seen_{0};

  std::atomic<int64_t> last_frame_ns_{0};
  std::atomic<bool> has_frame_{false};
  std::atomic<int64_t> link_timeout_ns_{500000000};

  std::mutex tx_mutex_;
  uint8_t seq_{0};
  std::atomic<uint64_t> tx_frames_{0};

  std::atomic<uint64_t> rx_frames_{0};
  std::atomic<uint64_t> crc_errors_{0};
  std::atomic<uint64_t> ack_timeouts_{0};

  std::mutex ack_mutex_;
  std::condition_variable ack_cv_;
  std::map<uint16_t, std::shared_ptr<AckWaiter>> waiters_;
};

}  // namespace robot_base

#endif  // ROBOT_BASE__STM32_LINK_HPP_
