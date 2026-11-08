# 全局配置参数包
# 此文件作为标准Python模块，可以被其他模块导入使用
import os
import platform

# 获取当前主机名
current_machine = platform.node()
print(f"当前运行主机: {current_machine}")

# 实时读取当前用户家目录，替换硬编码的 /home/cat
HOME = os.path.expanduser('~')

# ========== LIO-SAM 相关配置 ==========
# 雷达倾斜配置开关
USE_TILT_CONFIG = True  # True: 使用倾斜配置文件, False: 使用默认配置文件

# 建图模式开关 - 支持从环境变量读取，如果未设置则使用默认值
MANUAL_BUILD_MAP = os.environ.get('MANUAL_BUILD_MAP', 'False').lower() == 'true'  # True: 建图模式（打开octomap server，不运行nav2和web）, False: 导航模式

# 建图工具选择
BUILD_TOOL = os.environ.get('BUILD_TOOL', 'octomap_server').lower()  # 建图模式工具选择

# SLAM算法选择
SLAM_ALGORITHM = os.environ.get('SLAM_ALGORITHM', 'fast_lio').lower()  # SLAM算法选择: fast_lio, point_lio, lio_sam, super_lio

# 自动建图模式开关 - 支持从环境变量读取，如果未设置则使用默认值
AUTO_BUILD_MAP = os.environ.get('AUTO_BUILD_MAP', 'False').lower() == 'true'  # True: 自动建图模式（延迟启动explore_lite）, False: 正常模式

# 仅记录模式开关
RECORD_ONLY = False  # True: 仅记录数据，不建图（不运行nav2和web）, False: 导航模式

# ========== Nav2 相关配置 ==========
# 导航模式配置
NAVIGATION_MODE = os.environ.get('NAVIGATION_MODE', 'standalone').lower()  # standalone: 独立模式, integrated: 集成模式

# ========== 主机特定的配置字典 ==========
config_by_machine = {
    'RK3588': {
        # RK3588主机配置 - LIO-SAM
        'ONLINE_LIDAR': True,
        'LIO_SAM_BASE_CODE_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/LIO-SAM_MID360_ROS2_DOG/',
        'DEFAULT_BAG_PATH': '/home/cat/slam_data/livox_record_tilt_test_crop/',
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/cat/slam_data/reliability_override.yaml',
        'LIO_SAM_DEFAULT_LOAM_SAVE_DIR': '/home/cat/slam_data/loam/',
        
        # RK3588主机配置 - Nav2
        'NAV2_BASE_CODE_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/',
        'NAV2_DEFAULT_MAP_FILE': "/home/cat/slam_data/grid_map/map.yaml",
        'NAV2_DEFAULT_WEB_SCRIPT_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/web/run_web.sh',
        'NAV2_DEFAULT_BT_XML_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/config/navigate_to_pose_w_replanning_and_recovery.xml',
        'NAV2_DEFAULT_PARAMS_FILE': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/config/nav2_params.yaml',
        
        # RK3588主机配置 - FAST-LIO
        'FAST_LIO_BASE_CODE_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/FAST_LIO_ROS2_edit/',
        'FAST_LIO_LIDAR_TYPE': 1,  # 其他主机lidar_type为1
        'FAST_LIO_MAP_FILE_PATH': '/home/cat/slam_data/pcd/test.pcd',  # 添加的地图文件路径
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/cat/slam_data/reliability_override.yaml',
        'DEFAULT_USE_SIM_TIME': False,
        
        # RK3588主机配置 - Livox MID360
        'LIVOX_MID360_CONFIG': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/livox_ros_driver2/config/MID360_config_tilt.json',
        'LIVOX_MID360_CONFIG_NO_TILT': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/livox_ros_driver2/config/MID360_config_drone.json',
        
        # RK3588主机配置 - EGO Planner（真实飞行）
        'EGO_PLANNER_NAMESPACE': '/',
        'EGO_PLANNER_ODOM_TOPIC': '/lio/odom',
        'EGO_PLANNER_CLOUD_TOPIC': '/lio/cloud_world',
        'EGO_PLANNER_POSE_TOPIC': '/mid360/pose',
    },
    'jqr001': {
        # jqr001主机配置 - LIO-SAM
        'ONLINE_LIDAR': False,
        'LIO_SAM_BASE_CODE_PATH': '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/',
        'DEFAULT_BAG_PATH': '/home/ywj/projects/dataset/robot/livox_record_tilt_test2/',
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/ywj/projects/dataset/reliability_override.yaml',
        'LIO_SAM_DEFAULT_LOAM_SAVE_DIR': '/home/ywj/projects/LOAM/',
        
        # jqr001主机配置 - Nav2
        'NAV2_BASE_CODE_PATH': '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/',
        'NAV2_DEFAULT_MAP_FILE': '/home/ywj/projects/map_grid/map.yaml',
        'NAV2_DEFAULT_WEB_SCRIPT_PATH': '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh',
        'NAV2_DEFAULT_BT_XML_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/navigate_to_pose_w_replanning_and_recovery.xml',
        'NAV2_DEFAULT_PARAMS_FILE': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml',
        
        # jqr001主机配置 - FAST-LIO
        'FAST_LIO_BASE_CODE_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/FAST_LIO_ROS2_edit/',
        'FAST_LIO_LIDAR_TYPE': 1,  # 其他主机lidar_type为1
        'FAST_LIO_MAP_FILE_PATH': '/home/ywj/projects/pcd/test.pcd',  # 添加的地图文件路径
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/ywj/projects/dataset/reliability_override.yaml',
        'DEFAULT_USE_SIM_TIME': True,
        
        # jqr001主机配置 - Livox MID360
        'LIVOX_MID360_CONFIG': '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/livox_ros_driver2/config/MID360_config_drone.json',
        'LIVOX_MID360_CONFIG_NO_TILT': '/home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/livox_ros_driver2/config/MID360_config_drone.json',
        
        # jqr001主机配置 - EGO Planner（真实飞行）
        'EGO_PLANNER_NAMESPACE': '/',
        'EGO_PLANNER_ODOM_TOPIC': '/lio/odom',
        'EGO_PLANNER_CLOUD_TOPIC': '/lio/cloud_world',
        'EGO_PLANNER_POSE_TOPIC': '/mid360/pose',
    },
    'DESKTOP-4LS1SSN': {
        # DESKTOP-4LS1SSN主机配置 - LIO-SAM
        'ONLINE_LIDAR': False,
        'LIO_SAM_BASE_CODE_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/',
        'DEFAULT_BAG_PATH': '/mnt/d/projects/robot/livox_record_tilt_test2/',
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/scripts/reliability_override.yaml',
        'LIO_SAM_DEFAULT_LOAM_SAVE_DIR': '/mnt/d/projects/LOAM/',
        
        # DESKTOP-4LS1SSN主机配置 - Nav2
        'NAV2_BASE_CODE_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/',
        'NAV2_DEFAULT_MAP_FILE': "/home/ywj/slam_data/map.yaml",
        'NAV2_DEFAULT_WEB_SCRIPT_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh',
        'NAV2_DEFAULT_BT_XML_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/navigate_to_pose_w_replanning_and_recovery.xml',
        'NAV2_DEFAULT_PARAMS_FILE': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params_sim.yaml',
        
        # DESKTOP-4LS1SSN主机配置 - FAST-LIO
        'FAST_LIO_BASE_CODE_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/FAST_LIO_ROS2_edit/',
        'FAST_LIO_LIDAR_TYPE': 1,  # DESKTOP-4LS1SSN主机lidar_type为5
        'FAST_LIO_MAP_FILE_PATH': '/home/ywj/pcd/test.pcd',  # 添加的地图文件路径
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/scripts/reliability_override.yaml',
        'DEFAULT_USE_SIM_TIME': True,
        
        # DESKTOP-4LS1SSN主机配置 - Livox MID360
        'LIVOX_MID360_CONFIG': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/livox_ros_driver2/config/MID360_config_gazebo.json',
        'LIVOX_MID360_CONFIG_NO_TILT': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/livox_ros_driver2/config/MID360_config_gazebo.json',
        
        # DESKTOP-4LS1SSN主机配置 - EGO Planner（仿真飞行）
        'EGO_PLANNER_NAMESPACE': '/x500_depth_0/',
        'EGO_PLANNER_ODOM_TOPIC': '/x500_depth_0/livox_down_frame/mid360_down_lidar/odometry',
        'EGO_PLANNER_CLOUD_TOPIC': '/livox_down/lidar',
        'EGO_PLANNER_POSE_TOPIC': '/x500_depth_0/StereoOV7251/pose',
    },
    'DESKTOP-ypat': {
        # DESKTOP-ypat主机配置 - LIO-SAM
        'ONLINE_LIDAR': False,
        'LIO_SAM_BASE_CODE_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/',
        'DEFAULT_BAG_PATH': '/mnt/d/work/robot/livox_record_tilt_test2/',
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/scripts/reliability_override.yaml',
        'LIO_SAM_DEFAULT_LOAM_SAVE_DIR': '/mnt/d/work/LOAM/',
        
        # DESKTOP-ypat主机配置 - Nav2
        'NAV2_BASE_CODE_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/',
        'NAV2_DEFAULT_MAP_FILE': "/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/map_sample/map.yaml",
        'NAV2_DEFAULT_WEB_SCRIPT_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh',
        'NAV2_DEFAULT_BT_XML_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/navigate_to_pose_w_replanning_and_recovery.xml',
        'NAV2_DEFAULT_PARAMS_FILE': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params_sim.yaml',
        
        # DESKTOP-ypat主机配置 - FAST-LIO
        'FAST_LIO_BASE_CODE_PATH': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/FAST_LIO_ROS2_edit/',
        'FAST_LIO_LIDAR_TYPE': 1,  # 其他主机lidar_type为1
        'FAST_LIO_MAP_FILE_PATH': '/home/ywj/pcd/test.pcd',  # 添加的地图文件路径
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/scripts/reliability_override.yaml',
        'DEFAULT_USE_SIM_TIME': True,
        
        # DESKTOP-ypat主机配置 - Livox MID360
        'LIVOX_MID360_CONFIG': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/livox_ros_driver2/config/MID360_config_gazebo.json',
        'LIVOX_MID360_CONFIG_NO_TILT': '/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/livox_ros_driver2/config/MID360_config_gazebo.json',
        
        # DESKTOP-ypat主机配置 - EGO Planner（仿真飞行）
        'EGO_PLANNER_NAMESPACE': '/x500_depth_0/',
        'EGO_PLANNER_ODOM_TOPIC': '/x500_depth_0/livox_down_frame/mid360_down_lidar/odometry',
        'EGO_PLANNER_CLOUD_TOPIC': '/livox_down/lidar',
        'EGO_PLANNER_POSE_TOPIC': '/x500_depth_0/StereoOV7251/pose',
    },
    'lubancat': {
        # lubancat主机配置 - LIO-SAM
        'ONLINE_LIDAR': True,
        'LIO_SAM_BASE_CODE_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/LIO-SAM_MID360_ROS2_DOG/',
        'DEFAULT_BAG_PATH': '/home/cat/slam_data/livox_record_new/',
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/cat/slam_data/reliability_override.yaml',
        'LIO_SAM_DEFAULT_LOAM_SAVE_DIR': '/home/cat/slam_data/loam/',

        # lubancat主机配置 - Nav2
        'NAV2_BASE_CODE_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/',
        'NAV2_DEFAULT_MAP_FILE': "/home/cat/slam_data/grid_map/map.yaml",
        'NAV2_DEFAULT_WEB_SCRIPT_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/web/run_web.sh',
        'NAV2_DEFAULT_BT_XML_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/config/navigate_to_pose_w_replanning_and_recovery.xml',
        'NAV2_DEFAULT_PARAMS_FILE': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/config/nav2_params.yaml',
        
        # lubancat主机配置 - FAST-LIO
        'FAST_LIO_BASE_CODE_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/FAST_LIO_ROS2_edit/',
        'FAST_LIO_LIDAR_TYPE': 1,  # 其他主机lidar_type为1
        'FAST_LIO_MAP_FILE_PATH': '/home/cat/slam_data/pcd/test.pcd',  # 添加的地图文件路径
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/cat/slam_data/reliability_override.yaml',
        'DEFAULT_USE_SIM_TIME': False,
        
        # lubancat主机配置 - Livox MID360
        'LIVOX_MID360_CONFIG': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/livox_ros_driver2/config/MID360_config_drone.json',
        'LIVOX_MID360_CONFIG_NO_TILT': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/livox_ros_driver2/config/MID360_config_drone.json',
        
        # lubancat主机配置 - EGO Planner（真实飞行）
        'EGO_PLANNER_NAMESPACE': '/',
        'EGO_PLANNER_ODOM_TOPIC': '/lio/odom',
        'EGO_PLANNER_CLOUD_TOPIC': '/lio/cloud_world',
        'EGO_PLANNER_POSE_TOPIC': '/mid360/pose',
    },
    'ywj-B250-D3A': {
        # ywj-B250-D3A主机配置 - LIO-SAM
        'ONLINE_LIDAR': False,
        'LIO_SAM_BASE_CODE_PATH': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/',
        'DEFAULT_BAG_PATH': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/LIO-SAM_MID360_ROS2_DOG/',
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/scripts/reliability_override.yaml',
        'LIO_SAM_DEFAULT_LOAM_SAVE_DIR': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/LOAM/',
        
        # ywj-B250-D3A主机配置 - Nav2
        'NAV2_BASE_CODE_PATH': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/',
        'NAV2_DEFAULT_MAP_FILE': "/home/ywj/slam_data/grid_map/map.yaml",
        'NAV2_DEFAULT_WEB_SCRIPT_PATH': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/web/run_web.sh',
        'NAV2_DEFAULT_BT_XML_PATH': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/navigate_to_pose_w_replanning_and_recovery.xml',
        'NAV2_DEFAULT_PARAMS_FILE': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml',
        
        # ywj-B250-D3A主机配置 - FAST-LIO
        'FAST_LIO_BASE_CODE_PATH': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/FAST_LIO_ROS2_edit/',
        'FAST_LIO_LIDAR_TYPE': 1,  # 其他主机lidar_type为1
        'FAST_LIO_MAP_FILE_PATH': '/home/ywj/slam_data/pcd/test.pcd',  # 添加的地图文件路径
        'DEFAULT_RELIABILITY_OVERRIDE': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/scripts/reliability_override.yaml',
        'DEFAULT_USE_SIM_TIME': True,
        
        # ywj-B250-D3A主机配置 - Livox MID360
        'LIVOX_MID360_CONFIG': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/livox_ros_driver2/config/MID360_config_gazebo.json',
        'LIVOX_MID360_CONFIG_NO_TILT': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/livox_ros_driver2/config/MID360_config_gazebo.json',
        
        # ywj-B250-D3A主机配置 - Super-LIO
        'SUPER_LIO_BASE_CODE_PATH': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/Super-LIO/',
        'SUPER_LIO_SAVE_MAP': True,  # 台式机离线开发，不保存地图
        'SUPER_LIO_SAVE_MAP_DIR': '/home/ywj/slam_data/pcd',
        
        #SC-PGO
        'SC_PGO_SAVE_DIRECTORY': '/home/ywj/save_data/',
        
        # ywj-B250-D3A主机配置 - EGO Planner（仿真飞行）
        'EGO_PLANNER_NAMESPACE': '/x500_depth_0/',
        'EGO_PLANNER_ODOM_TOPIC': '/x500_depth_0/livox_down_frame/mid360_down_lidar/odometry',
        'EGO_PLANNER_CLOUD_TOPIC': '/livox_down/lidar',
        'EGO_PLANNER_POSE_TOPIC': '/x500_depth_0/StereoOV7251/pose',
    }
}

# 默认配置（当主机名不在配置字典中时使用）
default_config = {
    # LIO-SAM 默认配置
    'ONLINE_LIDAR': True,
    'LIO_SAM_BASE_CODE_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/LIO-SAM_MID360_ROS2_DOG/',
    'DEFAULT_BAG_PATH': '/home/cat/slam_data/livox_record_new/',
    'DEFAULT_RELIABILITY_OVERRIDE': '/home/cat/slam_data/reliability_override.yaml',
    'LIO_SAM_DEFAULT_LOAM_SAVE_DIR': '/home/cat/slam_data/loam/',
    
    # Nav2 默认配置
    'NAV2_BASE_CODE_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/',
    'NAV2_DEFAULT_MAP_FILE': "/home/cat/slam_data/grid_map/map.yaml",
    'NAV2_DEFAULT_WEB_SCRIPT_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/web/run_web.sh',
    'NAV2_DEFAULT_BT_XML_PATH': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml',
    'NAV2_DEFAULT_PARAMS_FILE': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/nav2_dog_slam/config/nav2_params.yaml',
    
    # FAST-LIO 默认配置
    'FAST_LIO_BASE_CODE_PATH': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/FAST_LIO_ROS2_edit/',
    'FAST_LIO_LIDAR_TYPE': 1,  # 默认lidar_type为1
    'FAST_LIO_MAP_FILE_PATH': '/home/cat/slam_data/pcd/test.pcd',  # 默认地图文件路径
    'DEFAULT_RELIABILITY_OVERRIDE': '/home/cat/slam_data/reliability_override.yaml',
    'DEFAULT_USE_SIM_TIME': True,
    
    # Livox MID360 默认配置
    'LIVOX_MID360_CONFIG': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/livox_ros_driver2/config/MID360_config_drone.json',
    'LIVOX_MID360_CONFIG_NO_TILT': '/home/cat/git/xtd2_ws/XTDrone2_ego_planner/lio/livox_ros_driver2/config/MID360_config_drone.json',
    
    # EGO Planner 默认配置（真实飞行默认）
    'EGO_PLANNER_NAMESPACE': '/',
    'EGO_PLANNER_ODOM_TOPIC': '/lio/odom',
    'EGO_PLANNER_CLOUD_TOPIC': '/lio/cloud_world',
    'EGO_PLANNER_POSE_TOPIC': '/mid360/pose',
}

# 根据当前主机名选择配置
def _normalize_cat_paths(cfg):
    """将配置中硬编码的 /home/cat/ 路径替换为当前用户家目录"""
    return {
        k: (v.replace('/home/cat/', f'{HOME}/', 1) if isinstance(v, str) and v.startswith('/home/cat/') else v)
        for k, v in cfg.items()
    }

selected_config = _normalize_cat_paths(config_by_machine.get(current_machine, default_config))

# ========== 导出LIO-SAM配置参数 ==========
ONLINE_LIDAR = selected_config['ONLINE_LIDAR']
LIO_SAM_BASE_CODE_PATH = selected_config['LIO_SAM_BASE_CODE_PATH']
DEFAULT_BAG_PATH = selected_config['DEFAULT_BAG_PATH']
DEFAULT_RELIABILITY_OVERRIDE = selected_config['DEFAULT_RELIABILITY_OVERRIDE']
LIO_SAM_DEFAULT_LOAM_SAVE_DIR = selected_config['LIO_SAM_DEFAULT_LOAM_SAVE_DIR']

DEFAULT_USE_SIM_TIME = selected_config['DEFAULT_USE_SIM_TIME']
DEFAULT_USE_SIM_TIME_STRING = 'true'
if not DEFAULT_USE_SIM_TIME:
    DEFAULT_USE_SIM_TIME_STRING = 'false'


# 导出 Livox MID360 配置参数
LIVOX_MID360_CONFIG = selected_config['LIVOX_MID360_CONFIG']
LIVOX_MID360_CONFIG_NO_TILT = selected_config['LIVOX_MID360_CONFIG_NO_TILT']



# 导出SLAM算法选择参数
SLAM_ALGORITHM = SLAM_ALGORITHM

# ========== 导出Nav2配置参数 ==========
NAV2_BASE_CODE_PATH = selected_config['NAV2_BASE_CODE_PATH']
NAV2_DEFAULT_MAP_FILE = selected_config['NAV2_DEFAULT_MAP_FILE']
NAV2_DEFAULT_WEB_SCRIPT_PATH = selected_config['NAV2_DEFAULT_WEB_SCRIPT_PATH']
NAV2_DEFAULT_BT_XML_PATH = selected_config['NAV2_DEFAULT_BT_XML_PATH']
NAV2_DEFAULT_PARAMS_FILE = selected_config['NAV2_DEFAULT_PARAMS_FILE']

# ========== 导出FAST-LIO配置参数 ==========
FAST_LIO_BASE_CODE_PATH = selected_config['FAST_LIO_BASE_CODE_PATH']
FAST_LIO_LIDAR_TYPE = selected_config['FAST_LIO_LIDAR_TYPE']
FAST_LIO_MAP_FILE_PATH = selected_config['FAST_LIO_MAP_FILE_PATH']
DEFAULT_RELIABILITY_OVERRIDE = selected_config['DEFAULT_RELIABILITY_OVERRIDE']

# ========== 坐标系名称配置 ==========
MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_LINK_FRAME = 'base_link'
LIVOX_FRAME = 'livox_frame'

# ========== 话题名称配置 ==========
ODOM_TOPIC = '/odometry/global'
SCAN_TOPIC = '/scan'
CMD_VEL_TOPIC = '/cmd_vel'
LIDAR_TOPIC = '/livox/lidar'
IMU_TOPIC = '/livox/imu'

# ========== EGO Planner 配置参数 ==========
EGO_PLANNER_NAMESPACE = selected_config['EGO_PLANNER_NAMESPACE']
EGO_PLANNER_ODOM_TOPIC = selected_config['EGO_PLANNER_ODOM_TOPIC']
EGO_PLANNER_CLOUD_TOPIC = selected_config['EGO_PLANNER_CLOUD_TOPIC']
EGO_PLANNER_POSE_TOPIC = selected_config['EGO_PLANNER_POSE_TOPIC']

# ========== 服务名称配置 ==========
INITIAL_POSE_SERVICE = '/initialpose'
GOAL_POSE_SERVICE = '/goal_pose'

# ========== Nav2参数文件自动更新 ==========
def update_nav2_params():
    """自动更新nav2_params.yaml中的配置"""
    nav2_params_path = os.path.join(NAV2_BASE_CODE_PATH, 'config/nav2_params.yaml')
    if os.path.exists(nav2_params_path):
        try:
            with open(nav2_params_path, 'r') as file:
                lines = file.readlines()
            with open(nav2_params_path, 'w') as file:
                for line in lines:
                    if '      use_sim_time' in line:
                        file.write(f'      use_sim_time: {DEFAULT_USE_SIM_TIME_STRING}\n')
                    elif '    use_sim_time:' in line:
                        file.write(f'    use_sim_time: {DEFAULT_USE_SIM_TIME_STRING}\n')
                    elif 'yaml_filename:' in line:
                        file.write(f'    yaml_filename: {NAV2_DEFAULT_MAP_FILE}\n')
                    elif 'default_nav_to_pose_bt_xml:' in line:
                        file.write(f'    default_nav_to_pose_bt_xml: "{NAV2_DEFAULT_BT_XML_PATH}"\n')
                    else:
                        file.write(line)
            print(f"Nav2参数文件已更新: {nav2_params_path}")
        except Exception as e:
            print(f"更新Nav2参数文件时出错: {e}")

# ========== FAST-LIO参数文件自动更新 ==========
def update_lio_params():
    """自动更新FAST-LIO的所有配置文件"""
    import glob
    # 获取所有yaml配置文件
    config_dir = os.path.join(FAST_LIO_BASE_CODE_PATH, 'config')
    if os.path.exists(config_dir):
        yaml_files = glob.glob(os.path.join(config_dir, "mid360.yaml"))
        for yaml_file in yaml_files:
            try:
                with open(yaml_file, 'r') as file:
                    lines = file.readlines()
                with open(yaml_file, 'w') as file:
                    for line in lines:
                        if 'lidar_type:' in line:
                            file.write(f'            lidar_type: {FAST_LIO_LIDAR_TYPE}\n')
                        elif 'map_file_path:' in line:
                            # 更新所有的map_file_path
                            file.write(f'        map_file_path: "{FAST_LIO_MAP_FILE_PATH}"\n')
                        else:
                            file.write(line)
                print(f"FAST-LIO参数文件已更新: {yaml_file}")
            except Exception as e:
                print(f"更新FAST-LIO参数文件 {yaml_file} 时出错: {e}")


    config_dir = os.path.join(FAST_LIO_BASE_CODE_PATH, '../point_lio_ros2/config')
    if os.path.exists(config_dir):
        yaml_files = glob.glob(os.path.join(config_dir, "mid360.yaml"))
        for yaml_file in yaml_files:
            try:
                with open(yaml_file, 'r') as file:
                    lines = file.readlines()
                with open(yaml_file, 'w') as file:
                    for line in lines:
                        if 'lidar_type:' in line:
                            file.write(f'            lidar_type: {FAST_LIO_LIDAR_TYPE}\n')
                        else:
                            file.write(line)
                print(f"POINT-LIO参数文件已更新: {yaml_file}")
            except Exception as e:
                print(f"更新POINT-LIO参数文件 {yaml_file} 时出错: {e}")


# 导入时自动更新Nav2参数
update_nav2_params()

# 导入时自动更新FAST-LIO参数
update_lio_params()