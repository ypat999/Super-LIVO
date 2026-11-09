cmake_minimum_required(VERSION 3.5)
project(calib_ros2)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Ceres REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 启用 MOC/UIC 相关特性（如 Qt 需求）
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(cv_bridge_INCLUDE_DIRS "/opt/ros/humble/include/cv_bridge/")

# 包含头文件目录
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${rosbag2_cpp_INCLUDE_DIRS}
  ${rosbag2_storage_INCLUDE_DIRS}
  ${rosbag2_transport_INCLUDE_DIRS}
  include
)

add_executable(bag_to_pcd src/bag_to_pcd.cpp)
add_executable(lidar_camera_calib src/lidar_camera_calib.cpp)
add_executable(lidar_camera_multi_calib src/lidar_camera_multi_calib.cpp)

# 连接库文件
target_link_libraries(bag_to_pcd 
  ${PCL_LIBRARIES} 
  rosbag2_cpp::rosbag2_cpp
)

target_link_libraries(lidar_camera_calib 
  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  Ceres::ceres
)

target_link_libraries(lidar_camera_multi_calib 
  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  cv_bridge::cv_bridge
)

ament_target_dependencies(bag_to_pcd
  rclcpp
  sensor_msgs
  pcl_conversions
  cv_bridge
  livox_ros_driver2
)

ament_target_dependencies(lidar_camera_calib
  rclcpp
  sensor_msgs
  pcl_conversions
  cv_bridge
  livox_ros_driver2
)

ament_target_dependencies(lidar_camera_multi_calib
  rclcpp
  sensor_msgs
  pcl_conversions
  cv_bridge
  livox_ros_driver2
)


# 让 ROS 2 发现可执行文件
install(TARGETS
  bag_to_pcd
  lidar_camera_calib
  lidar_camera_multi_calib
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
