// Keyboard teleop that publishes complete STM32 SET_VELOCITY frames.
//
// The host subscribes to /stm32/cmd_raw (std_msgs/msg/UInt8MultiArray) and
// writes each message to the chassis UART. Frames are sent at 50 Hz, so the
// MCU's 200 ms command watchdog stays satisfied while the node is running.
//
// Keys:
//   w / s : forward / backward, 150 mm/s (parameter)
//   a / d : turn left / right, 500 mrad/s (parameter)
//   space or x : stop
//   q : quit (sends one zero frame first)
//
// The chassis must already be enabled (SET_ENABLE 1) by the host. This node
// only sends SET_VELOCITY 0x01; it does not send enable/clear-fault.

#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "robot_base/protocol.hpp"

namespace robot_raw_bridge
{
namespace
{
using namespace std::chrono_literals;
}  // namespace

class CmdRawPublisher : public rclcpp::Node
{
public:
  CmdRawPublisher()
  : Node("cmd_raw_publisher")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/stm32/cmd_raw");
    linear_speed_mm_s_ = declare_parameter<int>("linear_speed_mm_s", 150);
    angular_speed_mrad_s_ = declare_parameter<int>("angular_speed_mrad_s", 500);

    if (linear_speed_mm_s_ <= 0 || angular_speed_mrad_s_ <= 0) {
      throw std::invalid_argument("speeds must be > 0");
    }

    publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
      output_topic_, rclcpp::QoS(10).reliable());

    stdin_is_tty_ = isatty(STDIN_FILENO) != 0;
    if (stdin_is_tty_) {
      tcgetattr(STDIN_FILENO, &saved_termios_);
      termios raw = saved_termios_;
      raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    } else {
      RCLCPP_WARN(
        get_logger(), "stdin is not a TTY; keys are disabled, publishing zero");
    }

    RCLCPP_INFO(
      get_logger(), "cmd_raw_publisher -> %s  linear=%d mm/s  angular=%d mrad/s",
      output_topic_.c_str(), linear_speed_mm_s_, angular_speed_mrad_s_);
    RCLCPP_INFO(
      get_logger(),
      "keys: w/s=forward/back, a/d=turn left/right, x/space=stop, q=quit");

    timer_ = create_wall_timer(20ms, std::bind(&CmdRawPublisher::on_timer, this));
  }

  ~CmdRawPublisher() override
  {
    if (stdin_is_tty_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &saved_termios_);
    }
  }

private:
  void on_timer()
  {
    if (stdin_is_tty_) {
      char buffer[64];
      while (true) {
        const ssize_t count = read(STDIN_FILENO, buffer, sizeof(buffer));
        if (count <= 0) {
          break;
        }
        for (ssize_t i = 0; i < count; ++i) {
          handle_key(buffer[i]);
        }
      }
    }

    if (quit_) {
      v_mm_s_ = 0;
      w_mrad_s_ = 0;
      send_frame();
      RCLCPP_INFO(get_logger(), "quit");
      rclcpp::shutdown();
      return;
    }

    send_frame();
  }

  void handle_key(char key)
  {
    const char lower = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    switch (lower) {
      case 'w':
        v_mm_s_ = linear_speed_mm_s_;
        w_mrad_s_ = 0;
        break;
      case 's':
        v_mm_s_ = -linear_speed_mm_s_;
        w_mrad_s_ = 0;
        break;
      case 'a':
        v_mm_s_ = 0;
        w_mrad_s_ = angular_speed_mrad_s_;
        break;
      case 'd':
        v_mm_s_ = 0;
        w_mrad_s_ = -angular_speed_mrad_s_;
        break;
      case ' ':
      case 'x':
        v_mm_s_ = 0;
        w_mrad_s_ = 0;
        break;
      case 'q':
        quit_ = true;
        return;
      default:
        return;
    }

    RCLCPP_INFO(
      get_logger(), "cmd v=%d mm/s w=%d mrad/s", v_mm_s_, w_mrad_s_);
  }

  void send_frame()
  {
    const auto payload = robot_base::protocol::encode_set_velocity(
      static_cast<int16_t>(std::clamp(v_mm_s_, -32768, 32767)),
      static_cast<int16_t>(std::clamp(w_mrad_s_, -32768, 32767)));
    const auto frame = robot_base::protocol::build_frame(
      static_cast<uint8_t>(robot_base::protocol::Cmd::SET_VELOCITY),
      seq_, payload);
    seq_ = static_cast<uint8_t>(seq_ + 1U);

    std_msgs::msg::UInt8MultiArray message;
    message.data = frame;
    publisher_->publish(message);
  }

  std::string output_topic_;
  int linear_speed_mm_s_{150};
  int angular_speed_mrad_s_{500};
  int v_mm_s_{0};
  int w_mrad_s_{0};
  uint8_t seq_{0};
  bool quit_{false};
  bool stdin_is_tty_{false};
  termios saved_termios_{};

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_raw_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_raw_bridge::CmdRawPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("cmd_raw_publisher"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
