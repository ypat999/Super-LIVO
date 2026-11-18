cmake_minimum_required(VERSION 3.8)
project(latency_probe_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(latency_probe src/latency_probe.cpp)
ament_target_dependencies(latency_probe rclcpp)

install(TARGETS latency_probe DESTINATION lib/${PROJECT_NAME})

ament_package()
