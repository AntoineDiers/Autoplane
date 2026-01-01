cmake_minimum_required(VERSION 3.8)
project(autopilot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB_RECURSE SOURCES src/*.cpp)
add_executable(autopilot ${SOURCES})
ament_target_dependencies(autopilot rclcpp std_msgs)

install(TARGETS autopilot DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
