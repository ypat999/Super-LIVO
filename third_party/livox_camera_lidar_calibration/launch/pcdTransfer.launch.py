from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_lidar_calibration',
            executable='pcdTransfer',
            output='screen',
            parameters=[{
                # 라이다 bag 폴더가 있는 기본 경로 (끝에 / 필수)
                'input_bag_path': 'data/lidar/',
                # pcd 파일이 저장될 기본 경로 (끝에 / 필수)
                'output_pcd_path': 'data/pcdFiles/',
                # 변환할 최대 점의 개수 (충분히 크게 설정)
                'threshold_lidar': 80000,
                # 캘리브레이션 위치 개수 (예: 0.bag 하나면 1, 0~2.bag 이면 3)
                'data_num': 1
            }]
        )
    ])