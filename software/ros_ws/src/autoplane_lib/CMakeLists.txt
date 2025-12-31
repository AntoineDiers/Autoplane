cmake_minimum_required(VERSION 3.8)
project(autoplane_lib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

file(GLOB_RECURSE SOURCES src/*.cpp)
add_library(autoplane_lib ${SOURCES})
ament_target_dependencies(autoplane_lib rclcpp)
ament_export_targets(autoplane_lib HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/autoplane_lib
  DESTINATION include
)
 
install(
  TARGETS autoplane_lib
  EXPORT autoplane_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
