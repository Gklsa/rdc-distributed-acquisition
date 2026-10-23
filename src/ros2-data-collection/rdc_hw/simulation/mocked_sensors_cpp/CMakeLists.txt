cmake_minimum_required(VERSION 3.8)
project(mocked_sensors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(mock_imu_publisher src/mock_imu_publisher.cpp)
ament_target_dependencies(mock_imu_publisher rclcpp sensor_msgs std_msgs)

add_executable(mock_temperature_publisher src/mock_temperature_publisher.cpp)
ament_target_dependencies(mock_temperature_publisher rclcpp sensor_msgs std_msgs)

add_executable(mock_camera_publisher src/mock_camera_publisher.cpp)
ament_target_dependencies(mock_camera_publisher rclcpp sensor_msgs std_msgs)

install(TARGETS
  mock_imu_publisher
  mock_temperature_publisher
  mock_camera_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch_local/
  DESTINATION share/${PROJECT_NAME}/launch_local)

ament_package()
