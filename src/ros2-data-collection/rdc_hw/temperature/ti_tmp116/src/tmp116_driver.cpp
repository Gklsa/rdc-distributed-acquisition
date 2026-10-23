#include "ti_tmp116/tmp116_driver.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <stdexcept>
#include <system_error>
#include <thread>

namespace ti_tmp116
{

namespace
{
constexpr uint8_t TEMP_RESULT = 0x00;
constexpr uint8_t CONFIGURATION = 0x01;
constexpr double TEMP_RESOLUTION = 0.0078125;  // degC per LSB
}  // namespace

TMP116Driver::TMP116Driver(int bus_id, int address)
: bus_id_(bus_id), address_(address), fd_(-1), configured_(false)
{
}

TMP116Driver::~TMP116Driver()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void TMP116Driver::open_bus()
{
  if (fd_ >= 0) {
    return;
  }
  const std::string path = "/dev/i2c-" + std::to_string(bus_id_);
  fd_ = ::open(path.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "TMP116: failed to open " + path);
  }
  if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
    throw std::system_error(errno, std::generic_category(), "TMP116: failed to select I2C address");
  }
}

void TMP116Driver::configure()
{
  // Continuous conversion, default averaging/conversion-cycle (matches the Python driver: 0x02, 0x00)
  uint8_t buf[3] = {CONFIGURATION, 0x02, 0x00};
  if (::write(fd_, buf, sizeof(buf)) != static_cast<ssize_t>(sizeof(buf))) {
    throw std::system_error(errno, std::generic_category(), "TMP116: failed to write configuration");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  configured_ = true;
}

double TMP116Driver::read_temperature()
{
  open_bus();
  if (!configured_) {
    configure();
  }

  uint8_t reg = TEMP_RESULT;
  if (::write(fd_, &reg, 1) != 1) {
    throw std::system_error(errno, std::generic_category(), "TMP116: failed to select temperature register");
  }

  uint8_t data[2] = {0, 0};
  if (::read(fd_, data, 2) != 2) {
    throw std::system_error(errno, std::generic_category(), "TMP116: failed to read temperature register");
  }

  int32_t raw = (static_cast<int32_t>(data[0]) << 8) | data[1];
  if (raw == 0x8000) {
    throw std::runtime_error("TMP116: data invalid flag set");
  }
  if (raw > 32767) {
    raw -= 65536;
  }
  return raw * TEMP_RESOLUTION;
}

}  // namespace ti_tmp116
