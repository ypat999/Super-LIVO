#!/usr/bin/env bash
# 对照组的另一半：同一份 Super-LIO 位姿（config/retail_lio.yaml，与 15 号脚本完全一致），
# 但把染色换回方案 A 的 colorizer（默认参数：z-buffer + 深度边缘剔除 + min_quality + 多帧择优）。
# 目的：排除「LIO 配置/轨迹不同」的干扰，单独看染色规则的贡献。
set -u
REPO=/home/ywj/git/Super-LIVO
BAG=${BAG:-/home/ywj/slam_data/Retail_Street}
OUT=${OUT:-/home/ywj/slam_data/colored_superlio_a_20261114}
mkdir -p "$OUT"

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-86}
export RCUTILS_LOGGING_BUFFERED_STREAM=1

set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

sed '/^ *app\./d' "$REPO/config/retail_lio.yaml" > "$OUT/retail_lio_superlio.yaml"

# 地图层旋钮覆盖（retail_lio.yaml 未写，用 Super-LIO 默认 sub_div=4 / K=8）：
#   SUB_DIV=2 子体素 2x2x2=8 槽；KNN_K=5 平面拟合邻域 5（代码钳制在 4..8）。
LIO_EXTRA=()
if [ -n "${SUB_DIV:-}" ]; then
  LIO_EXTRA+=(-p lio.hash_map.sub_div:="$SUB_DIV")
fi
if [ -n "${KNN_K:-}" ]; then
  LIO_EXTRA+=(-p lio.hash_map.plane_knn_k:="$KNN_K")
fi

pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null; sleep 2
ros2 run super_lio super_lio_node --ros-args \
  --params-file "$OUT/retail_lio_superlio.yaml" \
  "${LIO_EXTRA[@]}" > "$OUT/slio.log" 2>&1 &
for i in $(seq 1 60); do grep -aq "Map init done" "$OUT/slio.log" && break; sleep 1; done
# Super-LIO 的 /lio/cloud_body_dense 时间戳与图像实测存在约 0.09 s 固定偏斜，
# 默认配对窗 0.05 s 会把 20 s 之后的所有帧全部丢掉。SYNC_MAX 可放大配对窗。
EXTRA=()
if [ -n "${SYNC_MAX:-}" ]; then
  EXTRA+=(-p color.sync_max_offset:="$SYNC_MAX")
fi
ros2 run super_livo_visual_a visual_a_node --ros-args \
  --params-file "$REPO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml" \
  "${EXTRA[@]}" \
  -p pcd.save_dir:="$OUT" -p pcd.save_interval_sec:=0.0 \
  > "$OUT/vis.log" 2>&1 &
sleep 3
ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
  > "$OUT/play.log" 2>&1
sleep 3
pkill -INT -x visual_a_node; sleep 50
pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null; sleep 3
grep -a "彩色地图已保存" "$OUT/vis.log" | tail -1
ls -l "$OUT"/*.pcd 2>/dev/null || echo "!!! 无 PCD 输出"
echo "[superlio-a] done -> $OUT"
