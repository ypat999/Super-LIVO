#!/bin/bash
# ============================================================================
# 方案 A 实时彩色建图一键启动（RK3588 + Livox MID360 + 海康 MV-CB013）
# ----------------------------------------------------------------------------
# 用法：
#   ./visual_a_live.sh                    # 输出到 ~/slam_data/output/live_MMDD_HHMMSS
#   ./visual_a_live.sh /path/to/out       # 指定输出目录
#   DECIMATE=2 ./visual_a_live.sh         # VIO 抽帧（1/2/4，默认 4）
#   SHARD_SEC=5 ./visual_a_live.sh        # 彩色分片间隔（秒，默认 5）
#
# 停止：Ctrl-C（SIGINT 才会触发最终整图合并落盘），不要 kill -9。
# 产物：<out>/colored_00000.pcd ...（每 SHARD_SEC 秒一片增量）
#       <out>/super_livo_visual_a_colored.pcd（退出时合并全图）
#       <out>/app.log
# ============================================================================
# 注意：不要开 `set -u`，ROS 的 setup.bash 里有未定义变量引用，会直接退出。

ROOT=/home/ztl/git/sl_ws/Super-LIVO
LIVOX=/home/ztl/PointPanoramaTex/livox_driver2
HIK=/home/ztl/PointPanoramaTex/hik_camera

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93}
DECIMATE=${DECIMATE:-4}          # VIO 抽帧：4 ≈ 2.5 Hz（着色吞吐最好）
FILTER_RATE=${FILTER_RATE:-5}    # LIO 配准抽点，3→5 可把 LIO 提到约 9.6 Hz
SHARD_SEC=${SHARD_SEC:-5.0}
OUT=${1:-/home/ztl/slam_data/output/live_$(date +%m%d_%H%M%S)}

if pgrep -x visual_a_app >/dev/null; then
  echo "[WARN] 已有 visual_a_app 在运行，先停止：pkill -INT -f visual_a_app"
  exit 1
fi
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source "$ROOT/third_party/Super-LIO/install/setup.bash"
source "$LIVOX/install/setup.bash"
source "$ROOT/install/setup.bash"

# ---- 传感器（已在跑就复用；不在则后台拉起） ----
if ! pgrep -f 'livox_ros_driver2' >/dev/null; then
  echo "[start] Livox MID360 驱动 -> /tmp/livox_mid360.log"
  ( cd "$LIVOX" && nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
      > /tmp/livox_mid360.log 2>&1 & )
  sleep 3
else
  echo "[skip ] Livox 驱动已在运行"
fi

if ! pgrep -f 'hik_camera_publisher' >/dev/null; then
  echo "[start] 海康相机 -> /tmp/hik_camera.log"
  ( cd "$HIK" && nohup python3 src/hik_camera_publisher.py \
      > /tmp/hik_camera.log 2>&1 & )
  sleep 3
else
  echo "[skip ] 海康相机已在运行"
fi

echo "[run  ] ROS_DOMAIN_ID=$ROS_DOMAIN_ID decimate=$DECIMATE filter_rate=$FILTER_RATE"
echo "[run  ] 彩色分片每 ${SHARD_SEC}s 一片，输出：$OUT"
echo "[stop ] Ctrl-C（勿 kill -9）"

ros2 run super_livo_visual_a visual_a_app --ros-args \
  --params-file "$ROOT/common/super_livo_common/config/lio_move.yaml" \
  --params-file "$ROOT/solution_a_lightweight/super_livo_visual_a/config/visual_a_move.yaml" \
  -p super_livo_visual.pcd.save_dir:="$OUT" \
  -p super_livo_visual.pcd.save_interval_sec:=0.0 \
  -p super_livo_visual.pcd.save_interval_frames:=0 \
  -p super_livo_visual.pcd.shard_interval_sec:="$SHARD_SEC" \
  -p super_livo_visual.visual.decimate:="$DECIMATE" \
  -p lio.sensor.filter_rate:="$FILTER_RATE" \
  -p lio.ros.max_lidar_backlog:=2 \
  -p lio.publish.imu_odom_hz:=50 \
  -p lio.publish.cloud_body:=false \
  -p lio.publish.cloud_world:=false \
  -p lio.publish.cloud_body_dense:=false \
  2>&1 | tee "$OUT/app.log"

echo "[done ] 分片：$(ls "$OUT"/colored_*.pcd 2>/dev/null | wc -l) 个；合并图：$OUT/super_livo_visual_a_colored.pcd"
