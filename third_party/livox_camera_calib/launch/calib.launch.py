import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    calib_ros2_config_dir = get_package_share_directory('calib_ros2')
    calib_config = os.path.join(calib_ros2_config_dir, 'config', 'calib.yaml')
    rviz_config = os.path.join(calib_ros2_config_dir, 'rviz_cfg', 'calib.rviz')

    return LaunchDescription([
        # 启动 LiDAR-Camera 校准节点
        Node(
            package='calib_ros2',
            executable='lidar_camera_calib',
            name='lidar_camera_calib',
            output='screen',
            parameters=[calib_config]  # 直接传递参数文件
        ),
        # 启动 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output="screen",
            arguments=['-d', rviz_config]
        )
    ])