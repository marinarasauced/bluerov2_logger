cmake_minimum_required(VERSION 3.8)
project(bluerov2_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bluerov2_bag_recorder src/bluerov2_bag_recorder.cpp)

ament_target_dependencies(bluerov2_bag_recorder rclcpp rosbag2_cpp mavros_msgs sensor_msgs std_msgs)

install(TARGETS
  bluerov2_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
