cmake_minimum_required(VERSION 3.8)
project(gps_driver)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoplane_lib REQUIRED)


file(GLOB_RECURSE SOURCES src/*.cpp)
add_executable(gps_driver ${SOURCES})
ament_target_dependencies(gps_driver rclcpp std_msgs autoplane_lib)

install(TARGETS gps_driver DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
