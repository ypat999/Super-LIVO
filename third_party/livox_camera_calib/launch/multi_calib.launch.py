import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_name = 'camera1'
    # 获取功能包的路径
    package_share_directory = get_package_share_directory('calib_ros2')
    
    # 配置文件路径
    multi_calib_config_path = f"{package_share_directory}/config/multi_calib_{camera_name}.yaml"
    rviz_config_path = f"{package_share_directory}/rviz_cfg/calib.rviz"
    
    return LaunchDescription([
        
        # lidar_camera_multi_calib 节点
        Node(
            package='calib_ros2',
            executable='lidar_camera_multi_calib',
            name='lidar_camera_multi_calib',
            output='screen',
            parameters=[multi_calib_config_path]  # 直接传递参数文件
        ),
        
        # RViz 节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_path],
            output='screen',
        )
    ])