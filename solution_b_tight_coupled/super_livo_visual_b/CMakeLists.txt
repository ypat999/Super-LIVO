cmake_minimum_required(VERSION 3.16)
project(super_livo_visual_b)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/frontend/patch_tracker.cpp
  src/frontend/feature_lifecycle.cpp
  src/feature_map/feature_track.cpp
  src/feature_map/keyframe_manager.cpp
  src/constraint/lidar_visual_constraint.cpp
  src/constraint/msckf_constraint.cpp
  src/constraint/photometric_constraint.cpp
  src/estimator/unified_estimator.cpp
  src/estimator/map_commit_scheduler.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp Eigen3 OpenCV)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
