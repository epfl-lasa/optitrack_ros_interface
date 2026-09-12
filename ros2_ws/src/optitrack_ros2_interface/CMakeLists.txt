cmake_minimum_required(VERSION 3.8)
project(optitrack_ros2_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(optitrack_transform_publisher_ros2
  src/optitrack_transform_publisher_ros2.cpp)
target_include_directories(optitrack_transform_publisher_ros2 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(optitrack_transform_publisher_ros2 Eigen3::Eigen)
ament_target_dependencies(optitrack_transform_publisher_ros2 rclcpp geometry_msgs)

install(TARGETS optitrack_transform_publisher_ros2
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# Fallback config so `ros2 launch` works natively, outside the container, when
# OPTITRACK_CONFIG is unset. In the container, compose bind-mounts the repo
# ./config over /opt/optitrack/config and OPTITRACK_CONFIG points there.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "optitrack_cfg.yaml")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_transform test/test_pose_transform.cpp)
  target_include_directories(test_pose_transform PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_pose_transform Eigen3::Eigen)
endif()

ament_package()
