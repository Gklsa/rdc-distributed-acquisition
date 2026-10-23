#ifndef MOCKED_SENSORS_CPP__MOCK_CAMERA_PUBLISHER_HPP_
#define MOCKED_SENSORS_CPP__MOCK_CAMERA_PUBLISHER_HPP_

#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class MockCameraPublisher : public rclcpp::Node
{
public:
  MockCameraPublisher();

private:
  void publish_image();
  void publish_runtime();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr runtime_pub_;
  rclcpp::TimerBase::SharedPtr camera_timer_;
  rclcpp::TimerBase::SharedPtr runtime_timer_;

  std::mt19937 rng_;
  std::uniform_int_distribution<int> pixel_noise_;

  uint64_t msg_count_;
  rclcpp::Time start_time_;
};

#endif  // MOCKED_SENSORS_CPP__MOCK_CAMERA_PUBLISHER_HPP_
