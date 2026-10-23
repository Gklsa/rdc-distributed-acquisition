#include "mocked_sensors_cpp/mock_camera_publisher.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

static constexpr int IMAGE_WIDTH  = 320;
static constexpr int IMAGE_HEIGHT = 240;

MockCameraPublisher::MockCameraPublisher()
: Node("mock_camera_publisher_cpp"),
  rng_(std::random_device{}()),
  pixel_noise_(0, 255),
  msg_count_(0)
{
  camera_pub_  = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
  runtime_pub_ = create_publisher<std_msgs::msg::String>("/runtime/status", 10);

  // 10 Hz sensor publish
  camera_timer_  = create_wall_timer(100ms, std::bind(&MockCameraPublisher::publish_image, this));
  runtime_timer_ = create_wall_timer(5s,    std::bind(&MockCameraPublisher::publish_runtime, this));

  start_time_ = now();
  RCLCPP_INFO(get_logger(),
    "C++ mock camera publisher started — 10 Hz on /camera/image_raw (%dx%d RGB8)",
    IMAGE_WIDTH, IMAGE_HEIGHT);
}

void MockCameraPublisher::publish_image()
{
  auto msg = sensor_msgs::msg::Image();
  msg.header.stamp    = now();
  msg.header.frame_id = "camera_link";
  msg.width           = IMAGE_WIDTH;
  msg.height          = IMAGE_HEIGHT;
  msg.encoding        = "rgb8";
  msg.is_bigendian    = false;
  msg.step            = IMAGE_WIDTH * 3;   // 3 bytes per pixel (RGB)

  // fill with random noise to simulate an image
  msg.data.resize(msg.step * IMAGE_HEIGHT);
  for (auto & byte : msg.data) {
    byte = static_cast<uint8_t>(pixel_noise_(rng_));
  }

  camera_pub_->publish(msg);
  ++msg_count_;
}

void MockCameraPublisher::publish_runtime()
{
  auto elapsed = (now() - start_time_).seconds();
  double obs_hz = (elapsed > 0.0) ? (static_cast<double>(msg_count_) / elapsed) : 0.0;

  std::ostringstream json;
  json << "{"
       << "\"schema_version\":\"1.0\","
       << "\"device_id\":\"cpp_mock_camera\","
       << "\"node_name\":\"mock_camera_publisher_cpp\","
       << "\"experiment_id\":null,"
       << "\"status\":\"running\","
       << "\"recording_active\":false,"
       << "\"runtime_seconds\":" << std::fixed << std::setprecision(1) << elapsed << ","
       << "\"topic_stats\":{"
       <<   "\"/camera/image_raw\":{"
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
  rclcpp::spin(std::make_shared<MockCameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
