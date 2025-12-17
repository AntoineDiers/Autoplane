cmake_minimum_required(VERSION 3.8)
project(simulator)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  gazebo
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
