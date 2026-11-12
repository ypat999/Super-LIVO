#!/usr/bin/env bash
set -e

# 1) CMakeLists 纳入 src/visual 与 include/visual
bash /tmp/cm_v.sh

D=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio

# 2) 新文件统一 CRLF（与仓库现有文件一致）
for f in "$D/include/visual/visual_rotation.hpp" \
         "$D/src/visual/visual_rotation.cpp" \
         "$D/src/visual/visual_eskf_inject.cpp"; do
  sed -i 's/$/\r/' "$f"
  echo "crlf: $f"
done

# 3) 由备份 off 基线逐行复制，只替换 lio.visual 段
python3 - <<'PY'
import re
src = "/tmp/visa_ref/config/livox_360_visa_room_off.yaml"
dst = "/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b2.yaml"
text = open(src, encoding="utf-8").read()
lines = text.split("\n")
start = next(i for i, l in enumerate(lines) if l.strip() == "visual:")
end = next(i for i in range(start + 1, len(lines))
           if re.match(r"^  \w.*:\s*$", lines[i]) or lines[i].strip() == "")
block = """  visual:
    # ===== B2：光流两帧几何 -> 相机相对旋转 -> 注入 ESKF 旋转 =====
    # 只做「相邻图像帧 KLT + Essential/recoverPose -> 相对旋转」，
    # 不做逐点 LiDAR 深度关联/重投影（那条路已验证无效）。
    # enable=false 时订阅、前端、注入全部不存在，行为与无视觉版本逐帧一致。
    enable: true
    image_topic: "/camera/color/image_raw"

    # 图像 header 时刻 + img_time_offset = 归到 lidar 帧头的采集时刻。
    # 实测：lidar 帧头比最近一张图像 header 晚 69.84 ms（5311 帧统计，p95-p5=0.16ms）
    img_time_offset: 0.06984

    # 图像对与 lidar 帧窗口的对齐松弛量(s)，同时用于冻结参考节点的时间匹配
    frame_slack: 0.05

    # 针孔内参
    cam_fx: 611.5093994140625
    cam_fy: 611.7654418945312
    cam_cx: 639.8602905273438
    cam_cy: 397.16888427734375
    cam_width: 1280
    cam_height: 800
    # k1,k2,p1,p2（plumb_bob）
    cam_dist: [-0.030312219634652138, 0.032308001071214676,
               -0.00007440096669597551, -0.0003438158892095089]

    # T_cam_lidar 行主序 3x3 + 平移（body 帧=IMU 帧，用 T_cam_lidar 近似 T_cam_imu）
    Rcl: [-0.0194742202781, -0.999337825, 0.0307354236874,
           0.00249929990846, -0.0307898155693, -0.999522756498,
           0.999807235543, -0.0193881092914, 0.00309725258526]
    Pcl: [0.0296499257858, -0.0687642637427, -0.0600391734927]

    klt:
      max_features: 150
      min_distance: 10.0
      fb_error: 1.0
      pyramid_levels: 3
      use_fast: true
      fast_threshold: 20
      lk_window: 21

    essential:
      threshold: 1.0          # 像素重投影阈值
      prob: 0.999

    gate:
      min_inliers: 30         # 内点数低于此值则本帧对不产生测量

    # 相对旋转测量噪声标准差(rad)。0.02 rad ≈ 1.15°
    sigma_rot: 0.02"""
new = lines[:start] + block.split("\n") + lines[end:]
open(dst, "w", encoding="utf-8", newline="\n").write("\n".join(new))
print("wrote", dst)
PY

echo "=== 非 visual 段与 off 基线的 diff（应为空） ==="
diff <(sed -n '/^  visual:/,/^  relocation:/p' /tmp/visa_ref/config/livox_360_visa_room_off.yaml | grep -v '^  visual:\|^  relocation:') \
     <(sed -n '/^  visual:/,/^  relocation:/p' "$D/config/livox_360_visa_room_b2.yaml" | grep -v '^  visual:\|^  relocation:') \
  | head -5 || true
python3 - <<'PY'
import re
a = open("/tmp/visa_ref/config/livox_360_visa_room_off.yaml", encoding="utf-8").read().split("\n")
b = open("/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b2.yaml", encoding="utf-8").read().split("\n")
def cut(ls):
    s = next(i for i,l in enumerate(ls) if l.strip()=="visual:")
    e = next(i for i in range(s+1,len(ls)) if re.match(r"^  \w.*:\s*$", ls[i]) or ls[i].strip()=="")
    return ls[:s] + ls[e:]
print("outside-visual identical:", cut(a)==cut(b))
print("lines off/b2:", len(a), len(b))
PY
