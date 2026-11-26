#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""solution_d：FAST-LIVO2 对照臂启动文件。

改造自 third_party/FAST-LIVO2-ROS2/launch/mapping_mid360.launch.py：
  1. 相机是 raw bgr8，去掉 image_transport/republish 中转；
  2. 参数文件默认指向 solution_d_fastlivo2/config 下的本机标定；
  3. img_time_offset 可由 launch 参数覆盖（启动脚本会先现场标定时钟偏移）；
  4. fastlivo_mapping 延迟启动，确保 parameter_blackboard 的参数服务先就绪
     （camera_loader 用 SyncParametersClient 只等 100ms，抢跑会直接抛异常）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
SOLUTION_DIR = os.path.dirname(HERE)
DEFAULT_PARAMS = os.path.join(SOLUTION_DIR, "config", "fastlivo2_mid360_hik.yaml")
DEFAULT_CAMERA = os.path.join(SOLUTION_DIR, "config", "camera_hik_pinhole.yaml")

# vikit_common 是 plain CMake 装到 .deps/vikit，不在 ws/install 的 setup.bash 里，
# 不注入 LD_LIBRARY_PATH 会在启动时报 libvikit_common.so: cannot open shared object file
VIKIT_LIB = os.path.join(SOLUTION_DIR, ".deps", "vikit", "lib")
_APP_ENV = dict(os.environ)
_APP_ENV["LD_LIBRARY_PATH"] = VIKIT_LIB + ":" + _APP_ENV.get("LD_LIBRARY_PATH", "")


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    camera_file = LaunchConfiguration("camera_params_file")
    img_time_offset = LaunchConfiguration("img_time_offset")
    use_rviz = LaunchConfiguration("use_rviz")
    use_respawn = LaunchConfiguration("use_respawn")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=DEFAULT_PARAMS,
                              description="FAST-LIVO2 主参数文件"),
        DeclareLaunchArgument("camera_params_file", default_value=DEFAULT_CAMERA,
                              description="vikit camera_loader 读取的相机参数文件"),
        DeclareLaunchArgument("img_time_offset",
                              default_value=os.environ.get("IMG_TIME_OFFSET", "0.0"),
                              description="雷达钟 - 相机钟（秒），脚本会先现场标定"),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument("use_respawn", default_value="False"),

        # 相机参数走独立节点，供 vikit 的 getRemoteParam 跨节点读取
        Node(
            package="demo_nodes_cpp",
            executable="parameter_blackboard",
            name="parameter_blackboard",
            parameters=[camera_file],
            output="screen",
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="fast_livo",
                    executable="fastlivo_mapping",
                    # 不设 name：LIVMapper 内部已建名为 laserMapping 的节点，
                    # 设 name 会把同进程内所有节点都重映射成同名，触发 rosout 冲突告警。
                    parameters=[
                        params_file,
                        {"time_offset": {"img_time_offset": ParameterValue(
                            img_time_offset, value_type=float)}},
                    ],
                    output="screen",
                    respawn=use_respawn,
                    additional_env=_APP_ENV,
                ),
            ],
        ),

        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", os.path.join(
                get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")],
            output="screen",
        ),
    ])
