#!/bin/bash
echo "== 上色帧数（日志行计数） =="
grep -ac '点上色' /tmp/sf_vis.log /tmp/e1_vis.log
echo "== 容量上限告警 =="
grep -a '上限' /tmp/sf_vis.log /tmp/e1_vis.log | head -4
echo "== 两份 PCD 点数 =="
python3 - <<'PY'
import sys
sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
from blackframe_voxel_replay import read_pcd_xyzrgb
for p in ["/home/ywj/slam_data/singleframe/sl_map/super_livo_visual_a_colored.pcd",
          "/home/ywj/slam_data/e1_append/e1_map/super_livo_visual_a_colored.pcd"]:
    P, r = read_pcd_xyzrgb(p)
    print(p.split("/")[-2], f"{len(P):,}")
PY
echo "== 两个 bag 的 colored_cloud 帧数 =="
python3 - <<'PY'
import sys
sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
from blackframe_voxel_replay import load_bag
for b in ["/home/ywj/slam_data/singleframe/sl_frames", "/home/ywj/slam_data/e1_append/e1_frames"]:
    f, o = load_bag(b)
    print(b.split("/")[-1], f"colored {len(f)} 帧, odom {len(o)} 条, 时长 "
          f"{f[-1][0]-f[0][0]:.1f}s" if f else "空")
PY
