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
# 节点层新增依赖：图像/点云/里程计/IMU 消息、cv_bridge、pcl、彩色点云包
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pointcloud_colorizer REQUIRED)
# 落盘彩色地图 PCD 需要 PCL io 组件（libpcl_io）
find_package(PCL REQUIRED COMPONENTS common io)

# ---- 方案 B 算法库 ---------------------------------------------------------
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

# ---- 方案 B ROS2 节点 ------------------------------------------------------
add_executable(visual_b_node src/node/visual_b_node.cpp)
target_include_directories(visual_b_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(visual_b_node ${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(visual_b_node rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge pcl_conversions pointcloud_colorizer)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME} visual_b_node
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  # 可执行文件必须装到 lib/<pkg>，ros2 run / ros2 launch 才能按包名找到它
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp Eigen3 OpenCV sensor_msgs nav_msgs geometry_msgs cv_bridge pcl_conversions pointcloud_colorizer)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
