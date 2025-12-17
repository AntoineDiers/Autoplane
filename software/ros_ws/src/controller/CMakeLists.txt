cmake_minimum_required(VERSION 3.8)
project(controller)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


# Declare executable
add_executable(controller src/main.cpp)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)
target_include_directories(controller PUBLIC ${Eigen3_INCLUDE_DIRS})
ament_target_dependencies(controller rclcpp geometry_msgs tf2_geometry_msgs sensor_msgs eigen3_cmake_module)

install(TARGETS controller DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
