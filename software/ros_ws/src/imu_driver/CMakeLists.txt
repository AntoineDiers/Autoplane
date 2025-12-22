cmake_minimum_required(VERSION 3.8)
project(imu_driver)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Declare executable
file(GLOB_RECURSE SOURCES src/*.cpp)
add_executable(imu_driver ${SOURCES})
ament_target_dependencies(imu_driver rclcpp sensor_msgs tf2_geometry_msgs)

install(TARGETS imu_driver DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
