// Minimal non-blocking serial port for the X2.
//
// The USB adapter supplied with the X2 uses DTR to switch the motor on: the
// lidar powers up idle and does not spin until DTR is asserted. On a direct
// UART (no DTR line) the ioctl fails; that is reported but treated as
// non-fatal by the node.

#ifndef ROBOT_LIDAR__X2_SERIAL_HPP_
#define ROBOT_LIDAR__X2_SERIAL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace robot_lidar
{

class X2Serial
{
public:
  X2Serial() = default;
  ~X2Serial();

  X2Serial(const X2Serial &) = delete;
  X2Serial & operator=(const X2Serial &) = delete;

  bool open(const std::string & port, int baud, std::string & error);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // Non-blocking. Returns bytes read, 0 when nothing is buffered, -1 on error.
  int read(uint8_t * buffer, std::size_t max_len);

  bool set_dtr(bool on, std::string & error);

private:
  int fd_{-1};
};

}  // namespace robot_lidar

#endif  // ROBOT_LIDAR__X2_SERIAL_HPP_
