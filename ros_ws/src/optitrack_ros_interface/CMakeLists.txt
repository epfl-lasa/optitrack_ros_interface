cmake_minimum_required(VERSION 3.0.2)
project(optitrack_ros_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
find_package(Eigen3 REQUIRED)

# NOTE: no LIBRARIES entry. The previous file declared LIBRARIES ${PROJECT_NAME}
# while never calling add_library, which put a non-existent
# liboptitrack_ros_interface.so on every downstream package's link line.
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(optitrack_transform_publisher src/optitrack_transform_publisher.cpp)
target_link_libraries(optitrack_transform_publisher ${catkin_LIBRARIES})

# The previous file had no install() rules at all.
install(TARGETS optitrack_transform_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
