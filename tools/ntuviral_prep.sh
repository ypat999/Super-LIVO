#!/usr/bin/env bash
# NTU VIRAL 单序列取数 + ROS1→ROS2 转换 + 接口核对。
# 默认不下载任何东西：先 --list 看清单，再 --seq rtp_01 拉单个序列。
#
#   bash tools/ntuviral_prep.sh --list
#   bash tools/ntuviral_prep.sh --seq rtp_01              # 下载+解压+转换+核对
#   bash tools/ntuviral_prep.sh --seq rtp_01 --convert-only
#   PROXY=http://127.0.0.1:10811 bash tools/ntuviral_prep.sh --seq rtp_01
set -euo pipefail

ROOT=${ROOT:-/home/ywj/slam_data/ntu_viral}
RAW=$ROOT/raw
OUT=$ROOT/bags2
mkdir -p "$RAW" "$OUT"

# Dataverse datafile id（来自 ntu-aris.github.io/ntu_viral_dataset 下载表）
SEQ_IDS=(
  "eee_01 68133 398.7 8.7"
  "eee_02 68131 321.1 7.0"
  "eee_03 68132 181.4 4.3"
  "nya_01 68144 396.3 8.6"
  "nya_02 68138 428.7 9.4"
  "nya_03 68142 411.2 9.0"
  "sbs_01 68139 354.2 7.8"
  "sbs_02 68140 373.3 8.2"
  "sbs_03 68143 389.3 8.5"
  "rtp_01 98194 482 5.0"
  "rtp_02 98191 439 5.2"
  "rtp_03 98193 355 4.0"
  "tnp_01 98195 583 8.1"
  "tnp_02 98196 457 6.2"
  "tnp_03 98189 407 5.5"
  "spms_01 98192 446 5.5"
  "spms_02 98190 398 4.0"
  "spms_03 98188 386 5.0"
)

lookup() {  # 名称 -> "datafile 时长 大小"
  local want=$1
  for e in "${SEQ_IDS[@]}"; do
    set -- $e
    if [ "$1" = "$want" ]; then echo "$2 $3 $4"; return 0; fi
  done
  return 1
}

if [ "${1:-}" = "--list" ] || [ -z "${1:-}" ]; then
  echo "序列      datafile  时长(s)  大小(GB)   场景"
  echo "------    --------  ------  --------   ----------------------------"
  for e in "${SEQ_IDS[@]}"; do
    set -- $e
    printf "%-8s  %8s  %6s  %8s\n" "$1" "$2" "$3" "$4"
  done
  echo
  echo "推荐：rtp_01(482s/5.0GB) 或 tnp_01(583s/8.1GB) —— 时长明显大于现有 Retail 135s / bag2 160s。"
  exit 0
fi

SEQ=""
CONVERT_ONLY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --seq) SEQ=$2; shift 2 ;;
    --convert-only) CONVERT_ONLY=1; shift ;;
    *) shift ;;
  esac
done
[ -n "$SEQ" ] || { echo "需要 --seq <名称>（--list 看清单）"; exit 1; }

read -r DID DUR SIZE <<<"$(lookup "$SEQ")"
[ -n "$DID" ] || { echo "未知序列 $SEQ"; exit 1; }
echo "=== $SEQ: datafile=$DID 时长=${DUR}s 约 ${SIZE}GB ==="

URL="https://researchdata.ntu.edu.sg/api/access/datafile/$DID"
CURL=(curl -sSL --retry 6 --retry-delay 15 --speed-limit 20000 --speed-time 120)
if [ -n "${PROXY:-}" ]; then CURL+=(-x "$PROXY"); fi

ZIP=$RAW/$SEQ.zip
if [ $CONVERT_ONLY -eq 0 ]; then
  if [ -s "$ZIP" ]; then
    echo "已存在 $ZIP ($(du -h "$ZIP" | cut -f1))，跳过下载"
  else
    echo "下载中（断点续传 -C -）..."
    "${CURL[@]}" -C - -o "$ZIP" "$URL"
  fi
fi

# 无论是否 --convert-only，解压目录缺失时都要先解压，否则下面 find 不到 .bag
if [ -z "$(find "$RAW/$SEQ" -maxdepth 2 -name '*.bag' 2>/dev/null | head -1)" ]; then
  [ -s "$ZIP" ] || { echo "缺少 $ZIP，无法解压"; exit 1; }
  echo "解压..."
  unzip -n -q "$ZIP" -d "$RAW/$SEQ" || { echo "解压失败"; exit 1; }
fi

BAG=$(find "$RAW/$SEQ" -maxdepth 2 -name "*.bag" | head -1)
[ -n "$BAG" ] || { echo "没找到 .bag，解压目录内容："; ls -R "$RAW/$SEQ" | head -20; exit 1; }
echo "源 bag: $BAG ($(du -h "$BAG" | cut -f1))"

echo "=== ROS1 侧话题清单（转换前核对）==="
python3 - "$BAG" <<'PY'
import sys
from rosbags.rosbag1 import Reader
with Reader(sys.argv[1]) as r:
    print('时长 %.1f s  消息 %d' % ((r.duration or 0) * 1e-9, r.message_count))
    for c in sorted(r.connections, key=lambda x: -x.msgcount):
        print('  %-34s %-34s %8d' % (c.topic, c.msgtype, c.msgcount))
PY

echo "=== 转换 ROS1 -> ROS2（剔除 UWB/温度/磁场，保留雷达+IMU+相机+真值）==="
DST=$OUT/$SEQ
rm -rf "$DST"
export PATH="$HOME/.local/bin:$PATH"
rosbags-convert --src "$BAG" --dst "$DST" --dst-storage sqlite3 \
  --exclude-topic /uwb_endorange_info /uwb_exorange_info /node_pos_sc \
                  /imu/temperature /imu/magnetic_field

# rosbags 写的是 offered_qos_profiles: []，Humble 的 yaml-cpp 会报 bad conversion
# 让 ros2 bag play/info 直接失败；改成空字符串即可。
sed -i 's|offered_qos_profiles: \[\]|offered_qos_profiles: ""|g' "$DST/metadata.yaml"

echo "=== 转换后核对：话题 + 相机内参（CameraInfo）==="
python3 - "$DST" <<'PY'
import sys
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

ts = get_typestore(Stores.ROS2_HUMBLE)
with Reader(sys.argv[1]) as r:
    print('时长 %.1f s  消息 %d' % ((r.duration or 0) * 1e-9, r.message_count))
    for c in sorted(r.connections, key=lambda x: -x.msgcount):
        print('  %-34s %-40s %8d' % (c.topic, c.msgtype, c.msgcount))
    for topic in ('/left/camera_info', '/right/camera_info',
                  '/left/image_raw/camera_info', '/camera_info'):
        if topic not in {c.topic for c in r.connections}:
            continue
        for conn, _t, raw in r.messages(topics=[topic]):
            m = r.deserialize(raw, conn.msgtype)
            K = np.array(m.k).reshape(3, 3)
            D = np.array(m.d)
            print('\n%s  分辨率 %dx%d' % (topic, m.width, m.height))
            print('  fx=%.3f fy=%.3f cx=%.3f cy=%.3f' % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
            print('  D=%s' % np.round(D, 6).tolist())
            print('  投影矩阵 P=%s' % np.round(np.array(m.p).reshape(3, 4)[0], 3).tolist())
            print('  帧id=%s' % m.header.frame_id)
            break
PY

cat <<EOF

下一步（数据到位后）：
1) Super-LIO：已有现成配置 third_party/Super-LIO/src/super_lio/config/NTU.yaml
   （lidar_topic=/os1_cloud_node1/points, imu_topic=/imu/imu, lidar_type=7）
   需另开 output.dense/map 供 A/B 着色与视觉约束使用。
2) A/B 参数：把上面打印的 fx/fy/cx/cy/D 填入 image_topic=/left/image_raw 的配置；
   雷达-相机外参需标定或取官方 viral_eval 包（本数据集页面未直接给出数值）。
3) 评测：python3 tools/odom_vs_gt.py --gt gt/$SEQ/ground_truth.csv \\
        --bag $DST --topic /lio/odom --label lio/odom
EOF
