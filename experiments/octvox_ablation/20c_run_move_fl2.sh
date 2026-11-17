#!/usr/bin/env bash
# 官方 FAST-LIVO2（ROS2 移植）在 move_ros2 上的对照跑批，产物归档到 slam_data/<area>/results/<tag>/。
# 用法: bash 20c_run_move_fl2.sh [tag]      默认 tag=move_hf_fl2，倍速 RATE=0.1
# 主参数文件就是 $REPO/config/move_fastcalib.yaml（FL2 参数 schema，Rcl/Pcl 与 A/B 三臂逐字一致），
# 相机内参走 $FL2_WS/cfg/camera_pinhole_s3.yaml（parameter_blackboard）。
# 实测（2026-11-17，RATE=0.1）：raw_points.pcd 4.12 M 点、736 位姿、span 76.6 s、
# path 10.089 m、闭环 0.020 m（11-16 轮为 0.344 m），maxstep 0.462 m —— 唯一一次 >0.3 m
# 跳变落在 bag 自身的图像空洞窗口，本轮没有 not synced 告警。
set -o pipefail
REPO="${REPO:-$HOME/git/Super-LIVO}"
FL2_WS="${FL2_WS:-$HOME/fastlivo2_ws}"
TAG="${1:-move_hf_fl2}"
CFG="${CFG:-$REPO/config/move_fastcalib.yaml}"
CAM="${CAM:-$FL2_WS/cfg/camera_pinhole_s3.yaml}"
BAG="${BAG:-/mnt/dataset/slam_data/calib/move_ros2}"
DATA_ROOT="${DATA_ROOT:-/mnt/dataset/slam_data}"
LOGDIR=$FL2_WS/third_party/FAST-LIVO2-ROS2/Log
[ -d "$LOGDIR" ] || LOGDIR=$FL2_WS/src/FAST-LIVO2/Log
PCD=$LOGDIR/pcd; RES=$LOGDIR/result

# 产出物归档到源数据所在分区：slam_data/<area>/results/<tag>/
if [[ "$BAG" == "$DATA_ROOT"/* ]]; then AREA=$(basename "$(dirname "$BAG")"); else AREA=calib; fi
OUT=$DATA_ROOT/$AREA/results/$TAG

export LD_LIBRARY_PATH="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -vE 'trae|^\s*$' | paste -sd:)"
source /opt/ros/humble/setup.bash
source "$FL2_WS/install/setup.bash"
export ROS_DOMAIN_ID=12
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

mkdir -p "$PCD" "$RES" "$OUT"; rm -f "$PCD"/*.pcd "$RES"/*.txt "$LOGDIR"/imu.txt "$LOGDIR"/mat_*.txt "/tmp/${TAG}_node.log" "/tmp/${TAG}_bag.log"
find "$OUT" -maxdepth 1 -type f -delete
pkill -f "fastlivo_mappin[g]" 2>/dev/null || true
pkill -f "paramet[a]_blackboard" 2>/dev/null || true
sleep 2

ros2 launch fast_livo mapping_mid360_lvi.launch.py \
  mid360_params_file:="$CFG" camera_params_file:="$CAM" use_rviz:=False \
  > "/tmp/${TAG}_node.log" 2>&1 &
LPID=$!

for i in $(seq 1 40); do grep -aq "intrinsic:" "/tmp/${TAG}_node.log" && break; sleep 1; done
sleep 3
if grep -aq "Camera model not correctly specified" "/tmp/${TAG}_node.log"; then
  echo "FATAL: camera model load failed"; tail -30 "/tmp/${TAG}_node.log"; exit 1
fi

echo "=== [$TAG] play bag (rate=${RATE:-0.1}) ==="
ros2 bag play "$BAG" --rate "${RATE:-0.1}" --disable-keyboard-controls < /dev/null > "/tmp/${TAG}_bag.log" 2>&1
echo "=== [$TAG] bag finished, drain 25s ==="
sleep 25

echo "=== [$TAG] stop node (triggers savePCD) ==="
kill -INT "$LPID" 2>/dev/null || true
pkill -INT -f "fastlivo_mappin[g]" 2>/dev/null || true
sleep 25
pkill -f "fastlivo_mappin[g]" 2>/dev/null || true
pkill -f "paramet[a]_blackboard" 2>/dev/null || true
sleep 2

echo "=== [$TAG] export artifacts to slam_data ==="
[ -f "$PCD/all_raw_points.pcd" ] && mv -f "$PCD/all_raw_points.pcd" "$OUT/raw_points.pcd" || true
[ -f "$PCD/all_downsampled_points.pcd" ] && mv -f "$PCD/all_downsampled_points.pcd" "$OUT/downsampled_points.pcd" || true
shopt -s nullglob
for f in "$PCD"/*.txt "$LOGDIR"/imu.txt "$LOGDIR"/mat_out.txt "$LOGDIR"/mat_pre.txt "$RES"/*.txt; do
  mv -f "$f" "$OUT/$(basename "$f")"
done
shopt -u nullglob
echo "ARTIFACTS: $OUT"
ls -la "$OUT"
