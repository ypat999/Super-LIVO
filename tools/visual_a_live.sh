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
LOG="$OUT/app.log"
: > "$LOG"   # 先建日志文件，FAIL 退出时目录里也有现场说明

source /opt/ros/humble/setup.bash
source "$ROOT/third_party/Super-LIO/install/setup.bash"
source "$LIVOX/install/setup.bash"
source "$ROOT/install/setup.bash"

# ---- 传感器（已在跑就复用；不在则后台拉起） ----
# 判活用节点级进程名：`pgrep -f livox_ros_driver2` 会命中 launch 脚本乃至外部 shell
# 命令行，实测把"驱动正在退出"误判成"已在运行"，结果算法空等话题、无任何数据。
start_livox() {
  echo "[start] Livox MID360 驱动 -> /tmp/livox_mid360.log"
  ( cd "$LIVOX" && nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
      > /tmp/livox_mid360.log 2>&1 & )
}
start_hik() {
  echo "[start] 海康相机 -> /tmp/hik_camera.log"
  ( cd "$HIK" && nohup python3 src/hik_camera_publisher.py \
      > /tmp/hik_camera.log 2>&1 & )
}

if ! pgrep -f '[l]ivox_ros_driver2_node' >/dev/null; then
  start_livox; sleep 3
else
  echo "[skip ] Livox 驱动已在运行"
fi

if ! pgrep -f '[h]ik_camera_publisher\.py' >/dev/null; then
  start_hik; sleep 3
else
  echo "[skip ] 海康相机已在运行"
fi

echo "[run  ] ROS_DOMAIN_ID=$ROS_DOMAIN_ID decimate=$DECIMATE filter_rate=$FILTER_RATE"
echo "[run  ] 彩色分片每 ${SHARD_SEC}s 一片，输出：$OUT"
echo "[stop ] Ctrl-C（勿 kill -9）"

# 开机自启时传感器可能还没发布完，等两个话题都出现再起算法。
# 单次探测加 timeout：ros2 CLI 冷启动本就 2~4 s，DDS 异常时可能直接挂住。
probe_topics() {
  timeout 10 ros2 topic list 2>/dev/null
}
wait_topics() {
  for _ in $(seq 1 "$1"); do
    topics=$(probe_topics)
    if echo "$topics" | grep -qx '/livox/lidar' &&
       echo "$topics" | grep -qx '/left_camera/image'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

if ! wait_topics 30; then
  # 进程在 ≠ 驱动活着：实测残留的 livox 驱动收 SIGINT 后 10 s 不退出（launch 只能
  # 升级 SIGKILL），挂死后 pgrep 判活会误报"已在运行"，话题永远等不到。
  # 重试前先强杀再全新拉起。
  echo "[retry] 未等到话题，强杀疑似挂死的传感器进程后重启"
  pkill -9 -f '[l]ivox_ros_driver2_node'; pkill -9 -f '[m]sg_MID360_launch'
  pkill -9 -f '[h]ik_camera_publisher\.py'
  sleep 2
  start_livox; start_hik
  if ! wait_topics 30; then
    msg="[FAIL ] 缺少 /livox/lidar 或 /left_camera/image：$(probe_topics | grep -Ei 'lidar|camera' | tr '\n' ' ')"
    msg="$msg
        雷达日志 /tmp/livox_mid360.log，相机日志 /tmp/hik_camera.log"
    echo "$msg"
    echo "$msg" >> "$LOG"
    exit 1   # 交给 systemd Restart=on-failure 重试
  fi
fi

# 日志转发：应用直接写文件，tail -F 再把新增行喂给 stdout（systemd journal）。
# 不能写成 `app | tee log`：systemd 停止时给整个 cgroup 发 SIGINT，tee 先退出，
# 应用下一次写 stdout 就触发 SIGPIPE 被直接杀掉，来不及执行
# shutdownAndSave() 写合并图（实测合并图整个丢失，只剩分片）。
tail -n +1 -F "$LOG" 2>/dev/null &
TAIL_PID=$!

APP_PID=
trap '[ -n "$APP_PID" ] && kill -INT "$APP_PID" 2>/dev/null' INT TERM

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
  >> "$LOG" 2>&1 &
APP_PID=$!

# 收到 SIGINT 后 wait 会立刻返回，但应用还在收尾写合并图；必须继续等到它真正退出，
# 否则 bash 先走会让 systemd 把剩下的进程一并 SIGKILL，合并图照样丢。
while true; do
  wait "$APP_PID"
  RC=$?
  kill -0 "$APP_PID" 2>/dev/null || break
done
kill "$TAIL_PID" 2>/dev/null
wait "$TAIL_PID" 2>/dev/null

echo "[done ] rc=$RC 分片：$(ls "$OUT"/colored_*.pcd 2>/dev/null | wc -l) 个；合并图：$OUT/super_livo_visual_a_colored.pcd"
