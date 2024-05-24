cmake_minimum_required(VERSION 3.5)
project(optitrack_ros2_interface)

# Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Specify additional locations of header files
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(optitrack_transform_publisher_ros2 src/optitrack_transform_publisher_ros2.cpp)
ament_target_dependencies(optitrack_transform_publisher_ros2 rclcpp std_msgs sensor_msgs geometry_msgs Eigen3)

# Install the executable
install(TARGETS optitrack_transform_publisher_ros2
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
