cmake_minimum_required(VERSION 3.16)
project(super_livo_visual_a)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
# 节点层新增依赖：图像/点云/里程计消息、cv_bridge、pcl、彩色点云包
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pointcloud_colorizer REQUIRED)
# A/B 共用算法：退化签名映射、各向异性先验、重投影方差模型（头文件库）
find_package(super_livo_common REQUIRED)
# 落盘彩色地图 PCD 需要 PCL io 组件（libpcl_io）
find_package(PCL REQUIRED COMPONENTS common io)
# 公共代码库 Super-LIO：本包只"引用"其资源（库级链接，不做话题级回灌）
find_package(super_lio REQUIRED)

# ---- 方案 A 算法库 ---------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/frontend/klt_frontend.cpp
  src/constraint/depth_association.cpp
  src/constraint/reprojection_constraint.cpp
  src/constraint/degeneracy_gate.cpp
  src/update/visual_update.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${super_livo_common_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

# ---- 方案 A ROS2 节点 ------------------------------------------------------
add_executable(visual_a_node src/node/visual_a_node_main.cpp src/node/visual_a_node.cpp)
target_include_directories(visual_a_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
  ${super_livo_common_INCLUDE_DIRS}
)
target_link_libraries(visual_a_node ${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(visual_a_node rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge pcl_conversions pointcloud_colorizer)

# ---- 方案 A 独立可执行入口 -------------------------------------------------
# 同一进程内运行 Super-LIO 主干（super_lio::lio）与方案 A 视觉支路。
add_executable(visual_a_app src/app/visual_a_app.cpp src/node/visual_a_node.cpp)
target_include_directories(visual_a_app PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
  ${super_livo_common_INCLUDE_DIRS}
)
target_link_libraries(visual_a_app ${PROJECT_NAME} ${PCL_LIBRARIES} super_lio::lio)
ament_target_dependencies(visual_a_app rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge pcl_conversions pointcloud_colorizer)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME} visual_a_node visual_a_app
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  # 可执行文件必须装到 lib/<pkg>，ros2 run / ros2 launch 才能按包名找到它
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp Eigen3 OpenCV sensor_msgs nav_msgs geometry_msgs cv_bridge pcl_conversions pointcloud_colorizer super_livo_common)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
