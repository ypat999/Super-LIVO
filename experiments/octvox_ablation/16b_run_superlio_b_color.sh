#!/usr/bin/env bash
# 与 16_run_superlio_a_color.sh 同口径的 B 方案版本：同一份 Super-LIO 位姿/点云，
# 只把染色节点换成方案 B（紧耦合）。
# 跑两遍：
#   b_lio   color.use_own_odom=false —— 几何与相机位姿全取 lio/odom，与 A 完全同输入，
#           差异只剩节点内部实现，用于判断 B 的染色链路本身有没有独有问题。
#   b_own   color.use_own_odom=true  —— 世界变换与相机位姿都换成统一估计器的修正位姿，
#           看紧耦合是否真的让层裂/发糊变好（与 A 不再同输入）。
#
#   SUB_DIV=2 KNN_K=5 SYNC_MAX=0.15 bash experiments/octvox_ablation/16b_run_superlio_b_color.sh
# 产物：$OUT/b_{lio,own}/super_livo_visual_b_colored.pcd
set -u
REPO=/home/ywj/git/Super-LIVO
BAG=${BAG:-/home/ywj/slam_data/Retail_Street}
OUT=${OUT:-/home/ywj/slam_data/colored_superlio_b_20261115}
mkdir -p "$OUT"

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-86}
export RCUTILS_LOGGING_BUFFERED_STREAM=1

set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

sed '/^ *app\./d' "$REPO/config/retail_lio.yaml" > "$OUT/retail_lio_superlio.yaml"

# 地图层旋钮覆盖（retail_lio.yaml 未写，默认 sub_div=4 / K=8）。
LIO_EXTRA=()
if [ -n "${SUB_DIV:-}" ]; then
  LIO_EXTRA+=(-p lio.hash_map.sub_div:="$SUB_DIV")
fi
if [ -n "${KNN_K:-}" ]; then
  LIO_EXTRA+=(-p lio.hash_map.plane_knn_k:="$KNN_K")
fi
# Super-LIO 的 /lio/cloud_body_dense 时间戳与图像存在约 0.09 s 固定偏斜，
# 默认配对窗 0.05 s 会把 20 s 之后的帧全丢掉。
COLOR_EXTRA=()
if [ -n "${SYNC_MAX:-}" ]; then
  COLOR_EXTRA+=(-p super_livo_visual.color.sync_max_offset:="$SYNC_MAX")
fi
# 图像订阅用 reliable（与 rosbag2 回放端一致）。默认的 SensorDataQoS(best_effort)
# 在本机单线程 executor 下会被密集点云回调饿死，整段收不到图像（实测 0 帧）。
COLOR_EXTRA+=(-p super_livo_visual.image.qos_reliable:=true)
# 归因消融用：从环境注入额外的 -p（空格分隔的完整 token，如
# EXTRA_COLOR='-p super_livo_visual.color.own_res_max_pos:=0.1 -p super_livo_visual.color.own_res_max_rot:=0.0087'）。
# shellcheck disable=SC2206
COLOR_EXTRA+=(${EXTRA_COLOR:-})

run_one() {
  tag=$1        # lio | own
  own=$2        # false | true
  dir=$OUT/b_$tag
  mkdir -p "$dir"
  echo "=================== SUPERLIO + B ($tag, use_own_odom=$own) ==================="
  pkill -x super_lio_node 2>/dev/null; pkill -x visual_b_node 2>/dev/null; sleep 2
  ros2 run super_lio super_lio_node --ros-args \
    --params-file "$OUT/retail_lio_superlio.yaml" \
    "${LIO_EXTRA[@]}" > "$dir/slio.log" 2>&1 &
  for i in $(seq 1 60); do grep -aq "Map init done" "$dir/slio.log" && break; sleep 1; done
  ros2 run super_livo_visual_b visual_b_node --ros-args \
    --params-file "$REPO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_params.yaml" \
    "${COLOR_EXTRA[@]}" \
    -p super_livo_visual.color.use_own_odom:="$own" \
    -p super_livo_visual.pcd.save_dir:="$dir" -p super_livo_visual.pcd.save_interval_sec:=0.0 \
    > "$dir/vis.log" 2>&1 &
  sleep 3
  ros2 bag play "$BAG" --rate "${BAG_RATE:-1.0}" --disable-keyboard-controls < /dev/null \
    > "$dir/play.log" 2>&1
  sleep 3
  pkill -INT -x visual_b_node; sleep 50          # 退出时 drain + 落盘彩色地图
  pkill -x super_lio_node 2>/dev/null; pkill -x visual_b_node 2>/dev/null; sleep 3
  grep -a "彩色地图已保存" "$dir/vis.log" | tail -1
  grep -a "彩色统计" "$dir/vis.log" | tail -1
  ls -l "$dir"/*.pcd 2>/dev/null || echo "!!! $tag 无 PCD 输出"
}

# TAGS 用于只补跑某一臂（避免覆盖已跑好的产物），默认两臂都跑。
for tag in ${TAGS:-lio own}; do
  case "$tag" in
    lio) run_one lio false ;;
    own) run_one own true ;;
  esac
done
echo DONE_SUPERLIO_B_COLORED
