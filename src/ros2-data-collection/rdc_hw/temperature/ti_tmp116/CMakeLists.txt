cmake_minimum_required(VERSION 3.8)
project(ti_tmp116)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(tmp116_driver src/tmp116_driver.cpp)
target_include_directories(tmp116_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(tmp116_node src/tmp116_node.cpp)
target_link_libraries(tmp116_node tmp116_driver)
ament_target_dependencies(tmp116_node rclcpp sensor_msgs)

install(TARGETS tmp116_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS tmp116_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
