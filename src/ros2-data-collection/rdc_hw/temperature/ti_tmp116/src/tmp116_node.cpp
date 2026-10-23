#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "ti_tmp116/tmp116_driver.hpp"

class TMP116Node : public rclcpp::Node
{
public:
  TMP116Node()
  : Node("tmp116_node")
  {
    declare_parameter("i2c_bus", 1);
    declare_parameter("i2c_address", 0x48);
    declare_parameter("publish_rate_hz", 1.0);

    const int bus = static_cast<int>(get_parameter("i2c_bus").as_int());
    const int address = static_cast<int>(get_parameter("i2c_address").as_int());
    const double rate_hz = get_parameter("publish_rate_hz").as_double();

    driver_ = std::make_unique<ti_tmp116::TMP116Driver>(bus, address);
    pub_ = create_publisher<sensor_msgs::msg::Temperature>("/temperature", 10);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TMP116Node::publish_temperature, this));

    RCLCPP_INFO(
      get_logger(), "TMP116 node started (bus=%d, addr=0x%02X, %.2fHz)",
      bus, address, rate_hz);
  }

private:
  void publish_temperature()
  {
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = now();
    try {
      msg.temperature = driver_->read_temperature();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "TMP116 read failed: %s", e.what());
      return;
    }
    msg.variance = 0.0;
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Temp: %.3f degC", msg.temperature);
  }

  std::unique_ptr<ti_tmp116::TMP116Driver> driver_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TMP116Node>());
  rclcpp::shutdown();
  return 0;
}
