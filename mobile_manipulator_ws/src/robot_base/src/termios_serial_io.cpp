#include "robot_base/serial_io.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <string>

namespace robot_base
{
namespace
{

bool baud_to_speed(int baud, speed_t & speed)
{
  switch (baud) {
    case 9600:
      speed = B9600;
      return true;
    case 19200:
      speed = B19200;
      return true;
    case 38400:
      speed = B38400;
      return true;
    case 57600:
      speed = B57600;
      return true;
    case 115200:
      speed = B115200;
      return true;
    case 230400:
      speed = B230400;
      return true;
    case 460800:
      speed = B460800;
      return true;
    case 921600:
      speed = B921600;
      return true;
    default:
      return false;
  }
}

class TermiosSerialIo final : public SerialIo
{
public:
  ~TermiosSerialIo() override {close();}

  bool open(const std::string & port, int baud, std::string & error) override
  {
    if (fd_ >= 0) {
      close();
    }

    speed_t speed = B115200;
    if (!baud_to_speed(baud, speed)) {
      error = "unsupported baud rate " + std::to_string(baud);
      return false;
    }

    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      error = port + ": " + std::strerror(errno);
      return false;
    }

    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
      error = port + ": tcgetattr: " + std::strerror(errno);
      ::close(fd);
      return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~(PARENB | CSTOPB);
#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
      error = port + ": cfsetispeed/cfsetospeed: " + std::strerror(errno);
      ::close(fd);
      return false;
    }

    (void)tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
      error = port + ": tcsetattr: " + std::strerror(errno);
      ::close(fd);
      return false;
    }

    fd_ = fd;
    return true;
  }

  void close() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const override {return fd_ >= 0;}

  int read(uint8_t * buffer, std::size_t max_len, int timeout_ms) override
  {
    if (fd_ < 0) {
      return -1;
    }

    pollfd p{};
    p.fd = fd_;
    p.events = POLLIN;
    const int ready = ::poll(&p, 1, timeout_ms);
    if (ready < 0) {
      return errno == EINTR ? 0 : -1;
    }
    if (ready == 0) {
      return 0;
    }
    // A USB serial adapter that is unplugged can leave a valid fd behind:
    // poll reports HUP/ERR and read() then returns 0 forever. Treat that as a
    // link failure so Stm32Link can close the fd and reopen the device.
    if ((p.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0U) {
      return -1;
    }
    if ((p.revents & POLLIN) == 0U) {
      return 0;
    }

    ssize_t n;
    do {
      n = ::read(fd_, buffer, max_len);
    } while (n < 0 && errno == EINTR);

    if (n < 0) {
      return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
    }
    return static_cast<int>(n);
  }

  bool write(const uint8_t * data, std::size_t len, std::string & error) override
  {
    if (fd_ < 0) {
      error = "serial port is not open";
      return false;
    }

    std::size_t written = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);

    while (written < len) {
      ssize_t n;
      do {
        n = ::write(fd_, data + written, len - written);
      } while (n < 0 && errno == EINTR);

      if (n > 0) {
        written += static_cast<std::size_t>(n);
        continue;
      }

      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) {
          error = "serial write timeout";
          return false;
        }

        pollfd p{};
        p.fd = fd_;
        p.events = POLLOUT;
        const int ready = ::poll(&p, 1, static_cast<int>(remaining));
        if (ready < 0 && errno == EINTR) {
          continue;
        }
        if (ready <= 0) {
          error = "serial write timeout";
          return false;
        }
        continue;
      }

      error = std::string("serial write failed: ") + std::strerror(errno);
      return false;
    }

    return true;
  }

private:
  int fd_{-1};
};

}  // namespace

std::unique_ptr<SerialIo> make_termios_serial_io()
{
  return std::make_unique<TermiosSerialIo>();
}

}  // namespace robot_base
