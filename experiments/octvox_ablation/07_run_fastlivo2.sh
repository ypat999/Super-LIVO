#!/usr/bin/env bash
# 以官方 FAST-LIVO2（a09599a）在同一 bag 上取参考轨迹。
# 只记录位姿，不保存点云；重复跑两遍用于估计其自身重复性。
#
# 说明：本机 WSL 下 parameter_blackboard 的参数服务不可达，因此相机内参
# 直接并入 fastlivo 自身参数文件（07a_wrap_official_cfg.py），不改源码。
set -o pipefail

OUT=/home/ywj/slam_data/fastlivo2_ref_20261110
CFG=${CFG_OVERRIDE:-$OUT/avia_wrapped.yaml}
CAM=/home/ywj/slam_data/Retail_Street/camera_pinhole.yaml
BAG=/home/ywj/slam_data/Retail_Street
QOS=/tmp/rec_ref_qos.yaml
mkdir -p "$OUT"

if [ -z "${CFG_OVERRIDE:-}" ]; then
  python3 /home/ywj/git/Super-LIVO/experiments/octvox_ablation/07a_wrap_official_cfg.py \
      /home/ywj/slam_data/Retail_Street/avia.yaml "$CFG" "$CAM"
fi

cat > "$QOS" <<'YAML'
/aft_mapped_to_init:
  reliability: reliable
  history: keep_all
  depth: 2000
YAML

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/fastlivo2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

run_one() {
  tag=$1
  dir="$OUT/$tag"
  rm -rf "$dir"
  mkdir -p "$dir"
  pkill -f fastlivo_mapping >/dev/null 2>&1
  sleep 2

  /usr/bin/time -v -o "$dir/time.txt" ros2 run fast_livo fastlivo_mapping \
      --ros-args --params-file "$CFG" > "$dir/node.log" 2>&1 &
  sleep 8

  if ! pgrep -f fastlivo_mapping >/dev/null; then
    echo "[$tag] node 启动失败"; tail -30 "$dir/node.log"; return 1
  fi

  ros2 bag record -o "$dir/bag" --qos-profile-overrides-path "$QOS" \
      /aft_mapped_to_init > "$dir/rec.log" 2>&1 &
  recpid=$!
  sleep 3

  ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls \
      > "$dir/play.log" 2>&1
  sleep 3
  kill -INT "$recpid" 2>/dev/null
  sleep 3
  pkill -f fastlivo_mapping >/dev/null 2>&1
  sleep 2
  echo "[$tag] done: $(ls "$dir" | tr '\n' ' ')"
}

for tag in ${TAGS:-ref1 ref2}; do
  run_one "$tag"
done
echo "ALL DONE"
