// Stm32Link owns the parts of the protocol that are easy to get wrong:
// retransmitting with the same SEQ, giving up after the retry budget, and
// refusing to send once the MCU has gone quiet for the link timeout.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "robot_base/protocol.hpp"
#include "robot_base/serial_io.hpp"
#include "robot_base/stm32_link.hpp"

namespace
{

using namespace std::chrono_literals;

class FakeSerialIo : public robot_base::SerialIo
{
public:
  using WriteHook = std::function<void(const std::vector<uint8_t> &)>;

  bool open(const std::string & port, int baud, std::string & error) override
  {
    (void)port;
    (void)baud;
    (void)error;
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = true;
    return true;
  }

  void close() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = false;
    cv_.notify_all();
  }

  bool is_open() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return open_;
  }

  int read(uint8_t * buffer, std::size_t max_len, int timeout_ms) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(
      lock, std::chrono::milliseconds(timeout_ms),
      [this]() {return !rx_.empty() || !open_;});
    if (!open_) {
      return -1;
    }
    if (rx_.empty()) {
      return 0;
    }

    const std::size_t count = std::min(max_len, rx_.size());
    for (std::size_t i = 0; i < count; ++i) {
      buffer[i] = rx_.front();
      rx_.pop_front();
    }
    return static_cast<int>(count);
  }

  bool write(const uint8_t * data, std::size_t len, std::string & error) override
  {
    (void)error;
    const std::vector<uint8_t> frame(data, data + len);
    WriteHook hook;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      writes_.push_back(frame);
      hook = write_hook_;
    }
    if (hook) {
      hook(frame);
    }
    return true;
  }

  void push_rx(const std::vector<uint8_t> & data)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rx_.insert(rx_.end(), data.begin(), data.end());
    cv_.notify_all();
  }

  void set_write_hook(WriteHook hook)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    write_hook_ = std::move(hook);
  }

  std::vector<std::vector<uint8_t>> writes() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return writes_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool open_{false};
  std::deque<uint8_t> rx_;
  std::vector<std::vector<uint8_t>> writes_;
  WriteHook write_hook_;
};

std::vector<uint8_t> heartbeat_frame()
{
  return robot_base::protocol::build_frame(
    static_cast<uint8_t>(robot_base::protocol::Report::HEARTBEAT), 0,
    std::vector<uint8_t>{1, 0, 0});
}

bool wait_for_frames(
  const robot_base::Stm32Link & link, uint64_t expected, int timeout_ms = 500)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (link.rx_frames() >= expected) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return link.rx_frames() >= expected;
}

}  // namespace

TEST(Stm32LinkTests, AckCompletesTheRequest)
{
  auto fake = std::make_unique<FakeSerialIo>();
  FakeSerialIo * fake_ptr = fake.get();
  fake_ptr->set_write_hook(
    [fake_ptr](const std::vector<uint8_t> & written) {
      robot_base::protocol::FrameParser parser;
      const auto frames = parser.feed(written);
      if (frames.size() != 1) {
        return;
      }
      fake_ptr->push_rx(
        robot_base::protocol::build_frame(
          static_cast<uint8_t>(
            frames[0].cmd | robot_base::protocol::kAckFlag),
          0,
          std::vector<uint8_t>{
            frames[0].seq,
            static_cast<uint8_t>(robot_base::protocol::Result::OK)}));
    });

  std::atomic<int> ack_frames{0};
  robot_base::Stm32Link link(
    std::move(fake),
    [&ack_frames](const robot_base::protocol::Frame & frame) {
      if (frame.is_ack()) {
        ack_frames.fetch_add(1);
      }
    });
  link.set_link_timeout(5s);
  ASSERT_TRUE(link.open("/dev/fake", 115200));
  fake_ptr->push_rx(heartbeat_frame());
  ASSERT_TRUE(wait_for_frames(link, 1U));

  const auto result = link.send_with_ack(0x04, std::vector<uint8_t>{1}, 100ms, 3);
  EXPECT_TRUE(result.received);
  EXPECT_EQ(result.result, static_cast<uint8_t>(robot_base::protocol::Result::OK));
  EXPECT_EQ(fake_ptr->writes().size(), 1U);
  EXPECT_EQ(link.ack_timeouts(), 0U);
}

TEST(Stm32LinkTests, MissingAckRetransmitsSameSeqThenGivesUp)
{
  auto fake = std::make_unique<FakeSerialIo>();
  FakeSerialIo * fake_ptr = fake.get();

  robot_base::Stm32Link link(std::move(fake), nullptr);
  link.set_link_timeout(5s);
  ASSERT_TRUE(link.open("/dev/fake", 115200));
  fake_ptr->push_rx(heartbeat_frame());
  ASSERT_TRUE(wait_for_frames(link, 1U));

  std::string error;
  const auto result = link.send_with_ack(
    0x04, std::vector<uint8_t>{1}, 20ms, 3, &error);

  EXPECT_FALSE(result.received);
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(link.ack_timeouts(), 1U);

  const auto writes = fake_ptr->writes();
  ASSERT_EQ(writes.size(), 4U);  // initial attempt + 3 retries
  for (const auto & write : writes) {
    EXPECT_EQ(write, writes.front());
  }
  // Same SEQ on every retry is what lets the firmware de-duplicate.
  robot_base::protocol::FrameParser parser;
  const auto frames = parser.feed(writes.front());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].cmd, 0x04U);
}

TEST(Stm32LinkTests, SilentLinkIsDeadAndRefusesToSend)
{
  auto fake = std::make_unique<FakeSerialIo>();
  FakeSerialIo * fake_ptr = fake.get();

  robot_base::Stm32Link link(std::move(fake), nullptr);
  link.set_link_timeout(50ms);
  ASSERT_TRUE(link.open("/dev/fake", 115200));

  std::this_thread::sleep_for(80ms);
  EXPECT_FALSE(link.link_alive());

  std::string error;
  EXPECT_FALSE(link.send(0x01, std::vector<uint8_t>{0, 0, 0, 0}, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(fake_ptr->writes().size(), 0U);

  // One report is enough to bring the link back.
  fake_ptr->push_rx(heartbeat_frame());
  ASSERT_TRUE(wait_for_frames(link, 1U));
  EXPECT_TRUE(link.link_alive());
  EXPECT_TRUE(link.send(0x01, std::vector<uint8_t>{0, 0, 0, 0}, &error));
  EXPECT_EQ(fake_ptr->writes().size(), 1U);
}

TEST(Stm32LinkTests, CrcErrorsAreCountedAndDoNotStopParsing)
{
  auto fake = std::make_unique<FakeSerialIo>();
  FakeSerialIo * fake_ptr = fake.get();

  std::atomic<int> frames{0};
  robot_base::Stm32Link link(
    std::move(fake),
    [&frames](const robot_base::protocol::Frame &) {frames.fetch_add(1);});
  link.set_link_timeout(5s);
  ASSERT_TRUE(link.open("/dev/fake", 115200));

  auto bad = robot_base::protocol::build_frame(
    static_cast<uint8_t>(robot_base::protocol::Cmd::ESTOP), 3);
  bad.back() ^= 0xFFU;
  fake_ptr->push_rx(bad);
  fake_ptr->push_rx(heartbeat_frame());

  ASSERT_TRUE(wait_for_frames(link, 1U));
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(frames.load(), 1);
  EXPECT_EQ(link.crc_errors(), 1U);
}

TEST(Stm32LinkTests, HalfFrameIsDroppedAfterTheFiftyMillisecondGap)
{
  auto fake = std::make_unique<FakeSerialIo>();
  FakeSerialIo * fake_ptr = fake.get();

  std::atomic<int> frames{0};
  robot_base::Stm32Link link(
    std::move(fake),
    [&frames](const robot_base::protocol::Frame &) {frames.fetch_add(1);});
  link.set_link_timeout(5s);
  ASSERT_TRUE(link.open("/dev/fake", 115200));

  const auto full = heartbeat_frame();
  const std::vector<uint8_t> half(full.begin(), full.begin() + 5);
  fake_ptr->push_rx(half);
  std::this_thread::sleep_for(120ms);  // > 50 ms without a byte, read + parse

  const std::vector<uint8_t> rest(full.begin() + 5, full.end());
  fake_ptr->push_rx(rest);
  std::this_thread::sleep_for(80ms);
  EXPECT_EQ(frames.load(), 0);

  fake_ptr->push_rx(full);
  ASSERT_TRUE(wait_for_frames(link, 1U));
  EXPECT_EQ(frames.load(), 1);
}
