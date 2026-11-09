from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_lidar_calibration',
            executable='cameraCalib',
            output='screen',
            parameters=[{
                'camera_in_path': 'data/camera/in.txt',
                'camera_folder_path': 'data/camera/photos/',
                'result_path': 'data/camera/result.txt',
                'row_number': 6,
                'col_number': 9,
                'width': 12,
                'height': 12
            }]
        )
    ])