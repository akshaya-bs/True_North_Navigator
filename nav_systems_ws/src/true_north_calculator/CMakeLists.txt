cmake_minimum_required(VERSION 3.8)
project(true_north_calculator)

# Compiler flags (only needed if compiling C++ code)
if(CMAKE_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)       # Only needed for C++ nodes
find_package(rclpy REQUIRED)        # Only needed for Python nodes
find_package(true_north_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)     
find_package(geometry_msgs REQUIRED) 



ament_export_dependencies(
  rclpy
  true_north_interfaces
  sensor_msgs
  std_srvs          
  geometry_msgs      
)


# Installation
install(
  DIRECTORY launch
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

ament_package()

