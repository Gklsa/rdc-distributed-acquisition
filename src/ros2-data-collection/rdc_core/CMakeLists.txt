cmake_minimum_required(VERSION 3.8)
project(rdc_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(nlohmann_json REQUIRED)

# -- C++ : experiment_manager_cpp (rosbag2_cpp::Writer) ------------------
add_executable(experiment_manager_cpp src/experiment_manager_cpp_node.cpp)
target_link_libraries(experiment_manager_cpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  nlohmann_json::nlohmann_json)

install(TARGETS experiment_manager_cpp
  DESTINATION lib/${PROJECT_NAME})

# -- Python : pc_master, runtime_status, experiment_manager (subprocess) -
ament_python_install_package(rdc_core_py)

install(PROGRAMS
  scripts/pc_master
  scripts/runtime_status
  scripts/experiment_manager
  DESTINATION lib/${PROJECT_NAME})

ament_package()
