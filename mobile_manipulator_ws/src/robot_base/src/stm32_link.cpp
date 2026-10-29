#include "robot_base/stm32_link.hpp"

#include <algorithm>
#include <utility>

namespace robot_base
{
namespace
{

// protocol.md section 9: a gap longer than this abandons a half frame.
constexpr auto kByteGapReset = std::chrono::milliseconds(50);

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

uint16_t ack_key(uint8_t cmd, uint8_t seq)
{
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(cmd) << 8) | static_cast<uint16_t>(seq));
}

}  // namespace

Stm32Link::Stm32Link(std::unique_ptr<SerialIo> io, FrameCallback on_frame)
: io_(std::move(io)), on_frame_(std::move(on_frame))
{
}

Stm32Link::~Stm32Link()
{
  close();
}

bool Stm32Link::open(const std::string & port, int baud, std::string * error)
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

  if (rx_thread_.joinable()) {
    stop_.store(true);
    rx_thread_.join();
  }
  io_->close();

  stop_.store(false);
  std::string local_error;
  std::string & err = error != nullptr ? *error : local_error;
  if (!io_->open(port, baud, err)) {
    open_.store(false);
    return false;
  }

  parser_ = protocol::FrameParser{};
  last_crc_seen_ = 0;
  // The age is measured from the open instant until the first frame arrives,
  // which keeps diagnostics meaningful without pretending the link is alive.
  last_frame_ns_.store(steady_now_ns());
  has_frame_.store(false);
  open_.store(true);
  rx_thread_ = std::thread(&Stm32Link::rx_loop, this);
  return true;
}

void Stm32Link::close()
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  stop_.store(true);
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  io_->close();
  open_.store(false);
}

void Stm32Link::set_link_timeout(std::chrono::milliseconds timeout)
{
  link_timeout_ns_.store(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count());
}

bool Stm32Link::link_alive() const
{
  if (!open_.load() || !has_frame_.load()) {
    return false;
  }
  return last_frame_age_ms() <=
         (link_timeout_ns_.load() / 1000000LL);
}

int64_t Stm32Link::last_frame_age_ms() const
{
  return (steady_now_ns() - last_frame_ns_.load()) / 1000000LL;
}

bool Stm32Link::write_locked(
  const std::vector<uint8_t> & frame, std::string * error)
{
  std::string local_error;
  std::string & err = error != nullptr ? *error : local_error;
  if (!io_->write(frame.data(), frame.size(), err)) {
    return false;
  }
  tx_frames_.fetch_add(1);
  return true;
}

bool Stm32Link::send(
  uint8_t cmd, const std::vector<uint8_t> & payload, std::string * error)
{
  if (!open_.load()) {
    if (error != nullptr) {
      *error = "serial port is not open";
    }
    return false;
  }
  if (!link_alive()) {
    if (error != nullptr) {
      *error = "serial link lost (no frame within " +
        std::to_string(link_timeout_ns_.load() / 1000000LL) + " ms)";
    }
    return false;
  }

  std::lock_guard<std::mutex> tx_lock(tx_mutex_);
  const uint8_t seq = seq_++;
  const auto raw = protocol::build_frame(cmd, seq, payload);
  return write_locked(raw, error);
}

Stm32Link::AckResult Stm32Link::send_with_ack(
  uint8_t cmd, const std::vector<uint8_t> & payload,
  std::chrono::milliseconds timeout, int retries, std::string * error)
{
  AckResult result;

  if (!open_.load()) {
    if (error != nullptr) {
      *error = "serial port is not open";
    }
    return result;
  }
  if (!link_alive()) {
    if (error != nullptr) {
      *error = "serial link lost";
    }
    return result;
  }

  uint8_t seq = 0;
  std::vector<uint8_t> raw;
  {
    std::lock_guard<std::mutex> tx_lock(tx_mutex_);
    seq = seq_++;
    raw = protocol::build_frame(cmd, seq, payload);
  }

  auto waiter = std::make_shared<AckWaiter>();
  const uint16_t key = ack_key(cmd, seq);
  {
    std::lock_guard<std::mutex> ack_lock(ack_mutex_);
    waiters_[key] = waiter;
  }

  bool wrote_any = false;
  for (int attempt = 0; attempt <= retries; ++attempt) {
    if (!link_alive()) {
      if (error != nullptr) {
        *error = "serial link lost while waiting for ACK";
      }
      break;
    }

    {
      std::lock_guard<std::mutex> tx_lock(tx_mutex_);
      if (!write_locked(raw, error)) {
        break;
      }
    }
    wrote_any = true;

    std::unique_lock<std::mutex> ack_lock(ack_mutex_);
    if (ack_cv_.wait_for(ack_lock, timeout, [&waiter]() {return waiter->done;})) {
      result.received = true;
      result.result = waiter->result;
      waiters_.erase(key);
      return result;
    }
  }

  {
    std::lock_guard<std::mutex> ack_lock(ack_mutex_);
    waiters_.erase(key);
  }
  ack_timeouts_.fetch_add(1);
  if (wrote_any && error != nullptr && error->empty()) {
    *error = "no ACK after " + std::to_string(retries + 1) + " attempts";
  } else if (!wrote_any && error != nullptr && error->empty()) {
    *error = "command was not sent";
  }
  return result;
}

void Stm32Link::rx_loop()
{
  auto last_byte = std::chrono::steady_clock::now();
  uint8_t buffer[256];

  while (!stop_.load()) {
    const int count = io_->read(buffer, sizeof(buffer), 20);
    if (count < 0) {
      open_.store(false);
      break;
    }
    if (count == 0) {
      continue;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_byte > kByteGapReset) {
      parser_.reset();
    }
    last_byte = now;

    auto frames = parser_.feed(buffer, static_cast<std::size_t>(count));

    const uint64_t crc_total = parser_.crc_errors();
    if (crc_total > last_crc_seen_) {
      crc_errors_.fetch_add(crc_total - last_crc_seen_);
      last_crc_seen_ = crc_total;
    }

    for (const auto & frame : frames) {
      last_frame_ns_.store(steady_now_ns());
      has_frame_.store(true);
      rx_frames_.fetch_add(1);
      dispatch_ack(frame);
      if (on_frame_) {
        on_frame_(frame);
      }
    }
  }
}

void Stm32Link::dispatch_ack(const protocol::Frame & frame)
{
  if (!frame.is_ack()) {
    return;
  }

  const auto acked_cmd = frame.acked_cmd();
  const auto acked_seq = frame.acked_seq();
  const auto acked_result = frame.result();
  if (!acked_cmd || !acked_seq || !acked_result) {
    return;
  }

  const uint16_t key = ack_key(*acked_cmd, *acked_seq);
  std::lock_guard<std::mutex> ack_lock(ack_mutex_);
  const auto found = waiters_.find(key);
  if (found != waiters_.end()) {
    found->second->done = true;
    found->second->result = *acked_result;
    ack_cv_.notify_all();
  }
}

}  // namespace robot_base
