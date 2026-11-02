#include "robot_lidar/x2_serial.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace robot_lidar
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

}  // namespace

X2Serial::~X2Serial()
{
  close();
}

bool X2Serial::open(const std::string & port, int baud, std::string & error)
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

void X2Serial::close()
{
  if (fd_ < 0) {
    return;
  }
  // Best effort: drop DTR so the motor stops when the node exits.
  std::string ignored;
  (void)set_dtr(false, ignored);
  ::close(fd_);
  fd_ = -1;
}

int X2Serial::read(uint8_t * buffer, std::size_t max_len)
{
  if (fd_ < 0) {
    return -1;
  }

  ssize_t count;
  do {
    count = ::read(fd_, buffer, max_len);
  } while (count < 0 && errno == EINTR);

  if (count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    return -1;
  }
  return static_cast<int>(count);
}

bool X2Serial::set_dtr(bool on, std::string & error)
{
  if (fd_ < 0) {
    error = "serial port is not open";
    return false;
  }

  int status = 0;
  if (ioctl(fd_, TIOCMGET, &status) < 0) {
    error = std::string("TIOCMGET: ") + std::strerror(errno);
    return false;
  }

  if (on) {
    status |= TIOCM_DTR;
  } else {
    status &= ~TIOCM_DTR;
  }

  if (ioctl(fd_, TIOCMSET, &status) < 0) {
    error = std::string("TIOCMSET: ") + std::strerror(errno);
    return false;
  }
  return true;
}

}  // namespace robot_lidar
