#!/usr/bin/env bash
# Retail_Street 上分别跑"现在的方案 A / 方案 B"，同一条 LIO 主干、同一份官方标定。
#
#   A = visual_a_app（轻量视觉支路，不写回 ESKF，只做门控/着色）
#   B = visual_b_app（视觉约束经 ExternalObserve 写回 ESKF）
#
# 用法: ./run_ab_retail.sh <a|b>
# 产出: $OUT_BASE/<a|b>/{node.log, odom.csv, play.log, super_livo_visual_*_colored.pcd}
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
REPO=/home/ywj/git/Super-LIVO
LINE=${1:?用法: run_ab_retail.sh <a|b>}

BAG=${BAG:-/home/ywj/slam_data/Retail_Street_d2}
OUT_BASE=${OUT_BASE:-/home/ywj/slam_data/output/retail_ab_$(date +%m%d_%H%M)}
OUT=$OUT_BASE/$LINE
mkdir -p "$OUT"

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-86}
# 注意：不要设 ROS_LOCALHOST_ONLY=1 —— 本机 Cyclone 会把 lo 设为唯一接口并
# 关闭 multicast，导致播放端与节点互相发现不到（实测 imu_buf/lidar_buf 恒为 0）。
export RCUTILS_LOGGING_BUFFERED_STREAM=1

set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

if [ "$LINE" = "a" ]; then
  APP=super_livo_visual_a; EXE=visual_a_app
  VPARAM=$REPO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
  INJECT=false
else
  APP=super_livo_visual_b; EXE=visual_b_app
  VPARAM=$REPO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_params.yaml
  INJECT=true
fi

echo "[retail_ab] line=$LINE bag=$BAG out=$OUT"

# 视觉支路自己的彩色点云落盘目录（每 30 s 覆盖写 + 退出时再存一次）
sed "s|^ *pcd.save_dir:.*|    pcd.save_dir: \"$OUT\"|" "$VPARAM" > "$OUT/visual_params.yaml"

ARGS=(--params-file "$REPO/config/retail_lio.yaml" --params-file "$OUT/visual_params.yaml")
# 只有 B 声明了 app.visual_inject；给 A 传未声明参数会被 rclcpp 直接拒绝启动
[ "$LINE" = "b" ] && ARGS+=(-p app.visual_inject:=true)

ros2 run "$APP" "$EXE" --ros-args "${ARGS[@]}" > "$OUT/node.log" 2>&1 &
NODE_PID=$!
echo $NODE_PID > "$OUT/node.pid"
sleep 4

python3 "$REPO/third_party/Super-LIO/exp/odom_recorder.py" --out "$OUT/odom.csv" \
  > "$OUT/recorder.log" 2>&1 &
REC_PID=$!

ros2 bag play "$BAG" > "$OUT/play.log" 2>&1
echo "[retail_ab] bag 播完，等 3 s 收尾"
sleep 3

kill -INT $NODE_PID 2>/dev/null
for i in $(seq 1 20); do kill -0 $NODE_PID 2>/dev/null || break; sleep 1; done
kill -0 $NODE_PID 2>/dev/null && kill -9 $NODE_PID
sleep 2
kill -INT $REC_PID 2>/dev/null
sleep 1
kill -9 $REC_PID 2>/dev/null

{
  echo "line=$LINE"
  echo "bag=$BAG"
  echo "app=$APP/$EXE"
  echo "rmw=$RMW_IMPLEMENTATION domain=$ROS_DOMAIN_ID"
  echo "start=$(date -Is)"
  echo "odom_rows=$(wc -l < "$OUT/odom.csv" 2>/dev/null)"
  ls -l "$OUT"/*.pcd 2>/dev/null
} > "$OUT/meta.txt"
cat "$OUT/meta.txt"
echo "[retail_ab] done -> $OUT"
