cmake_minimum_required(VERSION 3.10...3.22)
project(camera_lidar_calibration)

# 외부 라이브러리(PCL 등)에서 발생하는 CMake 개발자 경고 숨기기
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

set(CMAKE_BUILD_TYPE "Release")
# ROS 2 Humble uses C++17 by default
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
)

add_executable(cameraCalib src/cameraCalib.cpp)
ament_target_dependencies(cameraCalib rclcpp cv_bridge)
target_link_libraries(cameraCalib ${OpenCV_LIBRARIES})

add_executable(pcdTransfer src/pcdTransfer.cpp src/common.h)
ament_target_dependencies(pcdTransfer rclcpp pcl_conversions pcl_ros rosbag2_cpp livox_ros_driver2)
target_link_libraries(pcdTransfer ${PCL_LIBRARIES})

add_executable(cornerPhoto src/corner_photo.cpp src/common.h)
ament_target_dependencies(cornerPhoto rclcpp cv_bridge image_transport)
target_link_libraries(cornerPhoto ${OpenCV_LIBRARIES})

add_executable(getExt1 src/cam_lid_external1.cpp src/common.h)
ament_target_dependencies(getExt1 rclcpp cv_bridge pcl_conversions)
target_link_libraries(getExt1 ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(getExt2 src/cam_lid_external2.cpp src/common.h)
ament_target_dependencies(getExt2 rclcpp cv_bridge pcl_conversions)
target_link_libraries(getExt2 ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(projectCloud src/projectCloud.cpp src/common.h)
ament_target_dependencies(projectCloud rclcpp cv_bridge rosbag2_cpp livox_ros_driver2 image_transport)
target_link_libraries(projectCloud ${OpenCV_LIBRARIES})

add_executable(colorLidar src/color_lidar_display.cpp src/common.h)
ament_target_dependencies(colorLidar rclcpp cv_bridge rosbag2_cpp livox_ros_driver2 nav_msgs pcl_conversions)
target_link_libraries(colorLidar ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

install(TARGETS
  cameraCalib
  pcdTransfer
  cornerPhoto
  getExt1
  getExt2
  projectCloud
  colorLidar
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
