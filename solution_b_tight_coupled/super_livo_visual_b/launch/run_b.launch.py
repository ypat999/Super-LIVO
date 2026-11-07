# ============================================================================
# 方案 B 启动脚本（ROS2 launch）
# ----------------------------------------------------------------------------
# 只启动 super_livo_visual_b 视觉节点并加载参数。运行前必须先让 Super-LIO
# 处理同一份 bag 得到 T_wl、lio/cloud_world（否则本节点拿不到里程计/地图）。
# 运行顺序同方案 A（见 solution_a_lightweight/super_livo_visual_a/launch/run_a.launch.py）。
# ============================================================================
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('super_livo_visual_b'), 'config', 'visual_b_params.yaml']
        ),
        description='方案 B 视觉节点参数文件路径',
    )

    visual_b_node = Node(
        package='super_livo_visual_b',
        executable='visual_b_node',
        name='super_livo_visual_b',
        output='screen',
        parameters=[params_file],
        # TODO(flash)：若需要，可在此追加 remappings 覆盖 topic
    )

    return LaunchDescription([
        declare_params,
        visual_b_node,
    ])
