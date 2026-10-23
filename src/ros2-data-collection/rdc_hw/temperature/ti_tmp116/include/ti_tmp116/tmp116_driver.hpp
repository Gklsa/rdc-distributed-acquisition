#pragma once

#include <string>

namespace ti_tmp116
{

// TMP116 temperature sensor driver (I2C interface)
// Hardware : I2C bus, default address 0x48
// Datasheet: https://www.ti.com/lit/ds/symlink/tmp116.pdf
class TMP116Driver
{
public:
  explicit TMP116Driver(int bus_id = 1, int address = 0x48);
  ~TMP116Driver();

  TMP116Driver(const TMP116Driver &) = delete;
  TMP116Driver & operator=(const TMP116Driver &) = delete;

  double read_temperature();

private:
  void open_bus();
  void configure();

  int bus_id_;
  int address_;
  int fd_;
  bool configured_;
};

}  // namespace ti_tmp116
