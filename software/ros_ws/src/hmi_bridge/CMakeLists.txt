cmake_minimum_required(VERSION 3.8)
project(hmi_bridge)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


# Declare executable
file(GLOB_RECURSE SOURCES src/*.cpp)
add_executable(hmi_bridge ${SOURCES})
target_include_directories(hmi_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(hmi_bridge PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../../../deps/ 
  ${CMAKE_CURRENT_LIST_DIR}/../../../deps/json/include)
ament_target_dependencies(hmi_bridge rclcpp)

install(TARGETS hmi_bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
