# ============================================================================
# 方案 A 启动脚本（ROS2 launch）
# ----------------------------------------------------------------------------
# 只启动 super_livo_visual_a 视觉节点并加载参数。运行前必须先让 Super-LIO
# 处理同一份 bag 得到 T_wl 与 lio/cloud_world（否则本节点拿不到里程计与地图）。
#
# 典型运行顺序（终端 1 起 Super-LIO，终端 2 回放 bag，终端 3 起本节点）：
#   ros2 launch super_lio <对应 launch>.py        # 起 Super-LIO，参数用 Retail_Street 配置
#   ros2 bag play /home/ywj/slam_data/Retail_Street --rate 1.0   # 回放 bag
#   ros2 launch super_livo_visual_a run_a.launch.py              # 起本节点
#
# 注：Super-LIO 的 launch 文件名与 config 需按实际安装位置核对；bag 回放若
# 用 livox 时间戳需注意 use_local_timestamp:=false 等参数（见项目 memory）。
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

    # 视觉辅助节点
    visual_a_node = Node(
        package='super_livo_visual_a',
        executable='visual_a_node',
        name='super_livo_visual_a',
        output='screen',
        parameters=[params_file],
        # TODO(flash)：若需要，可在此追加 remappings 覆盖 topic（如 image_topic 不同）
    )

    return LaunchDescription([
        declare_params,
        visual_a_node,
    ])
