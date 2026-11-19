cmake_minimum_required(VERSION 3.16)
project(super_livo_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# 纯头文件库：没有编译单元，只导出 include 目录。
# calibration_source.hpp 依赖 OpenCV FileStorage，导出依赖供下游 find_package 使用。
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(Eigen3 OpenCV)
ament_package()
