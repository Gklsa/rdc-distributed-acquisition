#ifndef MOCKED_SENSORS_CPP__MOCK_TEMPERATURE_PUBLISHER_HPP_
#define MOCKED_SENSORS_CPP__MOCK_TEMPERATURE_PUBLISHER_HPP_

#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/string.hpp"

class MockTemperaturePublisher : public rclcpp::Node
{
public:
  MockTemperaturePublisher();

private:
  void publish_temp();
  void publish_runtime();

  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr runtime_pub_;
  rclcpp::TimerBase::SharedPtr temp_timer_;
  rclcpp::TimerBase::SharedPtr runtime_timer_;

  std::mt19937 rng_;
  std::normal_distribution<double> noise_;

  uint64_t msg_count_;
  rclcpp::Time start_time_;
};

#endif  // MOCKED_SENSORS_CPP__MOCK_TEMPERATURE_PUBLISHER_HPP_
