#!/usr/bin/env bash
# 诊断第二环：两个 bag 同时回放（不启动节点）时，大消息（body 云 1MB / 图像 1MB）
# 与网卡实际吞吐的关系。用于验证"总带宽不够 -> best_effort 大消息整帧丢弃"。
set -o pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash

OUT=/tmp/bw_probe
rm -rf "$OUT"; mkdir -p "$OUT"
FL2BAG=/home/ywj/slam_data/official_vio_20261114/bag
RAWBAG=/home/ywj/slam_data/Retail_Street
NIC=enP4353p0s0

rx() { awk -v n=":" '$1=="'"$NIC"':" {print $2, $10}' /proc/net/dev; }

R0=$(rx); T0=$(date +%s.%N)
timeout 20 ros2 bag play "$FL2BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_fl2.log" 2>&1 &
timeout 20 ros2 bag play "$RAWBAG" --delay 2.61 --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_raw.log" 2>&1 &
timeout 20 ros2 topic hz /cloud_registered_body > "$OUT/hz_body.log" 2>&1 &
timeout 20 ros2 topic hz /left_camera/image > "$OUT/hz_img.log" 2>&1 &
timeout 20 ros2 topic hz /cloud_registered > "$OUT/hz_cr.log" 2>&1 &
wait
R1=$(rx); T1=$(date +%s.%N)
echo "--- hz body(1MB) ---";      tail -2 "$OUT/hz_body.log"
echo "--- hz image(1MB) ---";     tail -2 "$OUT/hz_img.log"
echo "--- hz cloud_registered ---"; tail -2 "$OUT/hz_cr.log"
echo "--- NIC rx ---"
echo "before: $R0"
echo "after : $R1"
python3 - "$R0" "$R1" "$T0" "$T1" <<'EOF'
import sys
b = sys.argv[1].split(); a = sys.argv[2].split()
tb_ = float(sys.argv[4]) - float(sys.argv[3])
print("rx_bytes %.1f MB, %.2f MB/s, dropped_pkts %s" % (
    (int(a[0]) - int(b[0])) / 1e6,
    (int(a[0]) - int(b[0])) / 1e6 / tb_,
    int(a[1]) - int(b[1])))
EOF
echo DONE_BW_PROBE
