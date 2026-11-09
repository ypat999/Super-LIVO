import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_name = 'camera1'
    return launch.LaunchDescription([
        DeclareLaunchArgument("bags_dir", default_value="/data/DeltaElect/Bag/20250310_bag"),
        DeclareLaunchArgument("pcds_dir", default_value=f"/data/DeltaElect/lib/calib_ros2_ws/src/calib_ros2/result/pcd_{camera_name}"),
        DeclareLaunchArgument("images_dir", default_value=f"/data/DeltaElect/lib/calib_ros2_ws/src/calib_ros2/result/img_{camera_name}"),
        DeclareLaunchArgument("lidar_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("image_topic", default_value=f"/hikcamera/{camera_name}/compressed"),
        DeclareLaunchArgument("is_custom_msg", default_value="true"),

        launch_ros.actions.Node(
            package="calib_ros2",
            executable="bag_to_pcd",
            name="bag_to_pcd",
            output="screen",
            parameters=[{
                "bags_dir": LaunchConfiguration("bags_dir"),
                "pcds_dir": LaunchConfiguration("pcds_dir"),
                "images_dir": LaunchConfiguration("images_dir"),
                "lidar_topic": LaunchConfiguration("lidar_topic"),
                "image_topic": LaunchConfiguration("image_topic"),
                "is_custom_msg": LaunchConfiguration("is_custom_msg"),
            }]
        )
    ])