#include "mocked_sensors_cpp/mock_imu_publisher.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

MockImuPublisher::MockImuPublisher()
: Node("mock_imu_publisher_cpp"),
  rng_(std::random_device{}()),
  noise_(0.0, 0.01),
  accel_noise_(0.0, 0.05),
  msg_count_(0)
{
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  runtime_pub_ = create_publisher<std_msgs::msg::String>("/runtime/status", 10);

  // 10 Hz sensor publish
  imu_timer_ = create_wall_timer(100ms, std::bind(&MockImuPublisher::publish_imu, this));

  // 5 s runtime status report
  runtime_timer_ = create_wall_timer(5s, std::bind(&MockImuPublisher::publish_runtime, this));

  start_time_ = now();
  RCLCPP_INFO(get_logger(), "C++ mock IMU publisher started — 10 Hz on /imu/data");
}

void MockImuPublisher::publish_imu()
{
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = now();
  msg.header.frame_id = "imu_link";

  // orientation: near-identity quaternion with small noise
  msg.orientation.x = noise_(rng_);
  msg.orientation.y = noise_(rng_);
  msg.orientation.z = noise_(rng_);
  msg.orientation.w = 1.0;
  msg.orientation_covariance[0] = 0.01;
  msg.orientation_covariance[4] = 0.01;
  msg.orientation_covariance[8] = 0.01;

  // angular velocity (rad/s)
  msg.angular_velocity.x = noise_(rng_);
  msg.angular_velocity.y = noise_(rng_);
  msg.angular_velocity.z = noise_(rng_);
  msg.angular_velocity_covariance[0] = 0.001;
  msg.angular_velocity_covariance[4] = 0.001;
  msg.angular_velocity_covariance[8] = 0.001;

  // linear acceleration (m/s^2) — Z axis ~9.81 gravity
  msg.linear_acceleration.x = accel_noise_(rng_);
  msg.linear_acceleration.y = accel_noise_(rng_);
  msg.linear_acceleration.z = 9.81 + accel_noise_(rng_);
  msg.linear_acceleration_covariance[0] = 0.01;
  msg.linear_acceleration_covariance[4] = 0.01;
  msg.linear_acceleration_covariance[8] = 0.01;

  imu_pub_->publish(msg);
  ++msg_count_;
}

void MockImuPublisher::publish_runtime()
{
  auto elapsed = (now() - start_time_).seconds();
  double obs_hz = (elapsed > 0.0) ? (static_cast<double>(msg_count_) / elapsed) : 0.0;

  // Build JSON manually — no external library needed
  std::ostringstream json;
  json << "{"
       << "\"schema_version\":\"1.0\","
       << "\"device_id\":\"cpp_mock_imu\","
       << "\"node_name\":\"mock_imu_publisher_cpp\","
       << "\"experiment_id\":null,"
       << "\"status\":\"running\","
       << "\"recording_active\":false,"
       << "\"runtime_seconds\":" << std::fixed << std::setprecision(1) << elapsed << ","
       << "\"topic_stats\":{"
       <<   "\"/imu/data\":{"
       <<     "\"expected_hz\":10.0,"
       <<     "\"observed_hz\":" << std::fixed << std::setprecision(2) << obs_hz << ","
       <<     "\"message_count\":" << msg_count_ << ","
       <<     "\"state\":\"" << (obs_hz >= 8.0 ? "ok" : "warning") << "\""
       <<   "}"
       << "}"
       << "}";

  auto out = std_msgs::msg::String();
  out.data = json.str();
  runtime_pub_->publish(out);

  RCLCPP_INFO(get_logger(), "runtime: count=%lu  hz=%.2f", msg_count_, obs_hz);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
