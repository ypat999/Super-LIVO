from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_lidar_calibration',
            executable='cornerPhoto',
            output='screen',
            parameters=[{
                'input_photo_path': 'data/photo/0.png',
                'ouput_path': 'data/corner_photo.txt',
                'intrinsic_path': 'data/parameters/intrinsic.txt'
            }]
        )
    ])