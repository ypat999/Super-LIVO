#!/usr/bin/env bash
set -e
SRC=/tmp/visa_ref/config/livox_360_visa_room_off.yaml
python3 - <<'PY'
import re
src = "/tmp/visa_ref/config/livox_360_visa_room_off.yaml"
dst = "/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b2.yaml"
lines = open(src, encoding="utf-8").read().split("\n")
i = next(k for k, l in enumerate(lines) if re.match(r"^\s*lio\.visual", l))
nxt = [k for k in range(i, len(lines)) if re.match(r"^\s*lio\.(?!visual)", lines[k])]
j = nxt[0] if nxt else len(lines)
print("visual section lines:", i + 1, "..", j)
print("first kept line after:", repr(lines[j]) if j < len(lines) else "<EOF>")

block = """    # ===== B2：光流两帧几何 -> 相机相对旋转 -> 注入 ESKF 旋转 =====
    # 只做「相邻图像帧 KLT + Essential/recoverPose -> 相对旋转」，不做逐点
    # LiDAR 深度关联/重投影（那条路在 off/A 线已验证无效并删除）。
    # enable=false 时订阅、前端、注入全部不存在，行为与无视觉版本逐帧一致。
    lio.visual.enable: true
    lio.visual.image_topic: "/camera/color/image_raw"

    # 图像 header 时刻 + img_time_offset = 归到 lidar 帧头的采集时刻。
    # 实测：lidar 帧头比最近一张图像 header 晚 69.84 ms（5311 帧统计，p95-p5=0.16ms）
    lio.visual.img_time_offset: 0.06984
    # 图像对与 lidar 帧窗口的对齐松弛量(s)，同时用于冻结参考节点的时间匹配。
    # 该 bag lidar 帧内点时间跨度 160 ms（>帧周期 100 ms，帧间重叠），故放宽到 ±50 ms。
    lio.visual.frame_slack: 0.05

    lio.visual.cam_fx: 611.5093994140625
    lio.visual.cam_fy: 611.7654418945312
    lio.visual.cam_cx: 639.8602905273438
    lio.visual.cam_cy: 397.16888427734375
    lio.visual.cam_width: 1280
    lio.visual.cam_height: 800
    lio.visual.cam_dist: [-0.030312219634652138, 0.032308001071214676,
                          -0.00007440096669597551, -0.0003438158892095089]

    # T_cam_lidar（cam<-lidar），行主序 3x3 + 平移。Super-LIO 的 body 帧即 IMU 帧，
    # 这里直接用 T_cam_lidar 近似 T_cam_imu，引入的误差就是 lidar-imu 外参（厘米级），
    # 对 yaw 漂移量级无影响。只用其旋转部分 Rcl 做相机系<->机体系的相对旋转映射。
    lio.visual.Rcl: [-0.0194742202781, -0.999337825, 0.0307354236874,
                     0.00249929990846, -0.0307898155693, -0.999522756498,
                     0.999807235543, -0.0193881092914, 0.00309725258526]
    lio.visual.Pcl: [0.0296499257858, -0.0687642637427, -0.0600391734927]

    # KLT 前端
    lio.visual.klt.max_features: 150
    lio.visual.klt.min_distance: 10.0
    lio.visual.klt.fb_error: 1.0
    lio.visual.klt.pyramid_levels: 3
    lio.visual.klt.use_fast: true
    lio.visual.klt.fast_threshold: 20
    lio.visual.klt.lk_window: 21

    # 两帧几何（Essential 矩阵 RANSAC）
    lio.visual.essential.threshold: 1.0
    lio.visual.essential.prob: 0.999

    # 内点数低于此值则本帧对不产生测量
    lio.visual.gate.min_inliers: 30

    # 相对旋转测量噪声标准差(rad)。0.02 rad ≈ 1.15°
    lio.visual.sigma_rot: 0.02""".split("\n")

new = lines[:i] + block + [""] + lines[j:]
open(dst, "w", encoding="utf-8", newline="\n").write("\n".join(new))


def strip_visual(ls):
    k = next(m for m, l in enumerate(ls) if re.match(r"^\s*lio\.visual", l))
    nx = [m for m in range(k, len(ls)) if re.match(r"^\s*lio\.(?!visual)", ls[m])]
    n = nx[0] if nx else len(ls)
    return ls[:k] + ls[n:]


print("outside-visual identical to off baseline:",
      strip_visual(lines) == strip_visual(new))
print("off lines:", len(lines), " b2 lines:", len(new))
PY
echo "=== b2 文件 visual 段 ==="
grep -n 'lio\.visual' /home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b2.yaml
