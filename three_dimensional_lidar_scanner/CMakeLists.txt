cmake_minimum_required(VERSION 3.0.2)
project(three_dimensional_lidar_scanner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  sensor_msgs
  tf2
  laser_geometry
  pcl_ros
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  AverageScans.srv
  Servo.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime tf2 laser_geometry pcl_ros
)

add_library(ScanAverager src/ScanAverager.cpp)
target_include_directories(ScanAverager PUBLIC include)

add_library(VectorAveraging src/VectorAveraging.cpp)
target_include_directories(VectorAveraging PUBLIC include)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/ScanAveragerTests.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test VectorAveraging)
endif()

add_executable(average_scan_server src/ScanAveragerServer.cpp)
target_link_libraries(average_scan_server ScanAverager VectorAveraging ${catkin_LIBRARIES})
add_dependencies(average_scan_server three_dimensional_lidar_scanner_gencpp)

add_executable(three_dimensional_scanner src/Scanner.cpp)
target_link_libraries(three_dimensional_scanner ${catkin_LIBRARIES})
add_dependencies(three_dimensional_scanner three_dimensional_lidar_scanner_gencpp)