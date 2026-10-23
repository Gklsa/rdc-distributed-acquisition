#ifndef MOCKED_SENSORS_CPP__MOCK_IMU_PUBLISHER_HPP_
#define MOCKED_SENSORS_CPP__MOCK_IMU_PUBLISHER_HPP_

#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

class MockImuPublisher : public rclcpp::Node
{
public:
  MockImuPublisher();

private:
  void publish_imu();
  void publish_runtime();

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr runtime_pub_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr runtime_timer_;

  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  std::normal_distribution<double> accel_noise_;

  uint64_t msg_count_;
  rclcpp::Time start_time_;
};

#endif  // MOCKED_SENSORS_CPP__MOCK_IMU_PUBLISHER_HPP_
