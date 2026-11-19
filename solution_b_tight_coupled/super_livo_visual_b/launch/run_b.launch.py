# ============================================================================
# 方案 B 启动脚本（ROS2 launch）
# ----------------------------------------------------------------------------
# 启动单体可执行文件 visual_b_app：同一进程内运行 Super-LIO 主干与方案 B
# 视觉支路（进程内钩子直推，无 DDS 回环）。
# ============================================================================
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    lio_params_file = LaunchConfiguration('lio_params_file')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('super_livo_visual_b'), 'config', 'visual_b_params.yaml']
        ),
        description='方案 B 视觉支路参数文件路径',
    )
    declare_lio_params = DeclareLaunchArgument(
        'lio_params_file',
        default_value='config/retail_lio.yaml',
        description='Super-LIO 主干参数文件路径（相对路径按启动目录解析）',
    )

    visual_b_app = Node(
        package='super_livo_visual_b',
        executable='visual_b_app',
        name='super_livo_visual_b',
        output='screen',
        parameters=[lio_params_file, params_file],
        # TODO(flash)：若需要，可在此追加 remappings 覆盖 topic
    )

    return LaunchDescription([
        declare_params,
        declare_lio_params,
        visual_b_app,
    ])
