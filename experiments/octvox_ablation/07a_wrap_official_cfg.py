#!/usr/bin/env python3
"""把官方 FAST-LIVO2 数据集配置（无 ros__parameters 包装）转成 ROS2 参数文件。

Retail_Street/avia.yaml 与 camera_pinhole.yaml 来自 HKU 仓库原样拷贝；
ROS2 `--params-file` 要求 `/**/ros__parameters` 包装，故此处只做包装与类型规整，
不改任何数值。

相机内参并入同一文件：本机 WSL 下 parameter_blackboard 的参数服务不可达
（`SyncParametersClient::wait_for_service` 超时，上游已加 dummy 兜底），
而 vikit 的 `getRemoteParam` 会**先读本节点自己的参数**，因此把 cam_* 直接
写进 fastlivo 自身参数即可，无需修改任何源码。
"""
import sys
import yaml

src, dst = sys.argv[1], sys.argv[2]
cam_path = sys.argv[3] if len(sys.argv) > 3 else None

with open(src) as f:
    cfg = yaml.safe_load(f)

# 显式补上 livox 消息类型（数据集用 livox_ros_driver2 CustomMsg）
cfg.setdefault("common", {}).setdefault("livox_msg_type", "livox_ros_driver2")

# 官方配置里这些量声明为 double_array；YAML 里写成 [1, 0, ...] 会被解析为
# integer_array，ROS2 拒绝类型变更，故强制转成 float。
for k in ("extrinsic_T", "extrinsic_R", "Rcl", "Pcl"):
    if k in cfg.get("extrin_calib", {}):
        cfg["extrin_calib"][k] = [float(v) for v in cfg["extrin_calib"][k]]

# 源码中声明为 double 的标量（写成整数会被 ROS2 判为类型不符）
DOUBLES = [
    ("time_offset", "imu_time_offset"), ("time_offset", "img_time_offset"),
    ("time_offset", "exposure_time_init"), ("time_offset", "lidar_time_offset"),
    ("preprocess", "blind"), ("preprocess", "filter_size_surf"),
    ("imu", "acc_cov"), ("imu", "gyr_cov"), ("imu", "b_acc_cov"), ("imu", "b_gyr_cov"),
    ("lio", "dept_err"), ("lio", "beam_err"), ("lio", "min_eigen_value"),
    ("lio", "voxel_size"), ("lio", "sigma_num"),
    ("local_map", "sliding_thresh"),
    ("pcd_save", "filter_size_pcd"), ("publish", "blind_rgb_points"),
    ("vio", "inv_expo_cov"), ("debug", "plot_time"),
]
for sec, key in DOUBLES:
    if key in cfg.get(sec, {}):
        cfg[sec][key] = float(cfg[sec][key])

# 相机内参并入顶层（vikit 的 getRemoteParam 优先读本节点参数）
if cam_path:
    with open(cam_path) as f:
        cam = yaml.safe_load(f)
    ros = (cam.get("/**") or cam.get("**") or {}).get("ros__parameters", cam)
    CAM_DOUBLES = {"scale", "cam_fx", "cam_fy", "cam_cx", "cam_cy",
                   "cam_d0", "cam_d1", "cam_d2", "cam_d3"}
    for k, v in ros.items():
        cfg[k] = float(v) if k in CAM_DOUBLES else v

with open(dst, "w") as f:
    yaml.safe_dump({"/**": {"ros__parameters": cfg}}, f,
                   sort_keys=False, default_flow_style=False, width=200)
print("wrote", dst)
