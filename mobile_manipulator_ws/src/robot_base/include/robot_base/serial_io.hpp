// Thin seam between Stm32Link and the operating system. Tests substitute a
// fake implementation; production uses termios_serial_io.cpp.

#ifndef ROBOT_BASE__SERIAL_IO_HPP_
#define ROBOT_BASE__SERIAL_IO_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace robot_base
{

class SerialIo
{
public:
  virtual ~SerialIo() = default;

  virtual bool open(const std::string & port, int baud, std::string & error) = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;

  // Reads up to max_len bytes. Returns bytes read, 0 on timeout, -1 on error.
  virtual int read(uint8_t * buffer, std::size_t max_len, int timeout_ms) = 0;

  virtual bool write(const uint8_t * data, std::size_t len, std::string & error) = 0;
};

std::unique_ptr<SerialIo> make_termios_serial_io();

}  // namespace robot_base

#endif  // ROBOT_BASE__SERIAL_IO_HPP_
