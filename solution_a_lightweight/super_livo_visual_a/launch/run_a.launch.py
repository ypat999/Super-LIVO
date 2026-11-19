# ============================================================================
# 方案 A 启动脚本（ROS2 launch，单体模式）
# ----------------------------------------------------------------------------
# 启动 visual_a_app：同一进程内运行 Super-LIO 主干 + 方案 A 视觉支路，
# 不再需要单独起 super_lio_node（旧多 node 代码已删除）。
#
# 典型运行顺序（终端 1 回放 bag，终端 2 起本 launch）：
#   ros2 bag play <bag> --rate 1.0
#   ros2 launch super_livo_visual_a run_a.launch.py params_file:=<数据集yaml>
#
# params_file 需要同时包含 LIO 参数（如 livox_360.yaml）与视觉参数
# （如 visual_a_room.yaml），顶层键用 /** 通配。
# ============================================================================
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 参数文件路径：安装后位于 share/super_livo_visual_a/config/visual_a_params.yaml
    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('super_livo_visual_a'), 'config', 'visual_a_params.yaml']
        ),
        description='方案 A 视觉节点参数文件路径',
    )

    # 方案 A 单体（Super-LIO 主干 + 视觉支路同进程）
    visual_a_app = Node(
        package='super_livo_visual_a',
        executable='visual_a_app',
        output='screen',
        parameters=[params_file],
        # TODO(flash)：若需要，可在此追加 remappings 覆盖 topic（如 image_topic 不同）
    )

    return LaunchDescription([
        declare_params,
        visual_a_app,
    ])
