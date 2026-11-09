from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_lidar_calibration',
            executable='getExt1',
            output='screen',
            parameters=[{
                'input_lidar_path': 'data/corner_lidar.txt',
                'input_photo_path': 'data/corner_photo.txt',
                'intrinsic_path': 'data/parameters/intrinsic.txt',
                'extrinsic_path': 'data/parameters/extrinsic.txt',
                'error_threshold': 12,
                # 외참수 행렬 초기값 (Livox와 카메라의 상대 위치에 맞춰 세팅하세요)
                'init_value': [0.0, -1.0, 0.0, 0.0,
                               0.0, 0.0, -1.0, 0.0,
                               1.0, 0.0, 0.0, 0.0]
            }]
        )
    ])