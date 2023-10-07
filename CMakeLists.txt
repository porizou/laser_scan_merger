cmake_minimum_required(VERSION 3.5)
project(laser_scan_merger)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(laser_scan_merger src/laser_scan_merger.cpp)
ament_target_dependencies(laser_scan_merger
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

install(TARGETS
  laser_scan_merger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
