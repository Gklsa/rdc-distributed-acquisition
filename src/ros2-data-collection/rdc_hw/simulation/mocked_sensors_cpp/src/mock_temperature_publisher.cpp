#include "mocked_sensors_cpp/mock_temperature_publisher.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

MockTemperaturePublisher::MockTemperaturePublisher()
: Node("mock_temperature_publisher_cpp"),
  rng_(std::random_device{}()),
  noise_(0.0, 0.1),
  msg_count_(0)
{
  temp_pub_    = create_publisher<sensor_msgs::msg::Temperature>("/temperature", 10);
  runtime_pub_ = create_publisher<std_msgs::msg::String>("/runtime/status", 10);

  // 1 Hz sensor publish
  temp_timer_    = create_wall_timer(1s,  std::bind(&MockTemperaturePublisher::publish_temp, this));
  runtime_timer_ = create_wall_timer(5s,  std::bind(&MockTemperaturePublisher::publish_runtime, this));

  start_time_ = now();
  RCLCPP_INFO(get_logger(), "C++ mock temperature publisher started — 1 Hz on /temperature");
}

void MockTemperaturePublisher::publish_temp()
{
  auto msg = sensor_msgs::msg::Temperature();
  msg.header.stamp    = now();
  msg.header.frame_id = "temperature_sensor";
  msg.temperature     = 25.0 + noise_(rng_);   // ~25 °C with small noise
  msg.variance        = 0.01;
  temp_pub_->publish(msg);
  ++msg_count_;
}

void MockTemperaturePublisher::publish_runtime()
{
  auto elapsed = (now() - start_time_).seconds();
  double obs_hz = (elapsed > 0.0) ? (static_cast<double>(msg_count_) / elapsed) : 0.0;

  std::ostringstream json;
  json << "{"
       << "\"schema_version\":\"1.0\","
       << "\"device_id\":\"cpp_mock_temperature\","
       << "\"node_name\":\"mock_temperature_publisher_cpp\","
       << "\"experiment_id\":null,"
       << "\"status\":\"running\","
       << "\"recording_active\":false,"
       << "\"runtime_seconds\":" << std::fixed << std::setprecision(1) << elapsed << ","
       << "\"topic_stats\":{"
       <<   "\"/temperature\":{"
       <<     "\"expected_hz\":1.0,"
       <<     "\"observed_hz\":" << std::fixed << std::setprecision(2) << obs_hz << ","
       <<     "\"message_count\":" << msg_count_ << ","
       <<     "\"state\":\"" << (obs_hz >= 0.8 ? "ok" : "warning") << "\""
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
  rclcpp::spin(std::make_shared<MockTemperaturePublisher>());
  rclcpp::shutdown();
  return 0;
}
