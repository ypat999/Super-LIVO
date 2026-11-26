#!/bin/bash
# ============================================================================
# 方案 D（FAST-LIVO2）实时彩色建图一键启动（RK3588 + Livox MID360 + 海康）
# ----------------------------------------------------------------------------
# 用法：
#   ./run_d_live.sh                       # 输出到 ~/slam_data/output/live_d_MMDD_HHMMSS
#   ./run_d_live.sh /path/to/out          # 指定输出目录
#   IMG_TIME_OFFSET=-205383152 ./run_d_live.sh   # 跳过现场标定，手填钟差
#   USE_RVIZ=1 ./run_d_live.sh            # 附带 rviz2（本机一般不推荐）
#
# 停止：Ctrl-C（SIGINT 才会触发 FAST-LIVO2 退出时 savePCD 落盘），不要 kill -9。
# 产物：<out>/log/pcd/all_raw_points.pcd        彩色整图（未降采样）
#       <out>/log/pcd/all_downsampled_points.pcd（filter_size_pcd=0.02 降采样）
#       <out>/log/result/scene_d.txt          轨迹（t x y z qx qy qz qw）
#       <out>/app.log
# ============================================================================
# 注意：不要开 `set -u`，ROS 的 setup.bash 里有未定义变量引用，会直接退出。

ROOT=/home/ztl/git/sl_ws/Super-LIVO
SOL="$ROOT/solution_d_fastlivo2"
FASTLIVO="$ROOT/third_party/FAST-LIVO2-ROS2"
LIVOX=/home/ztl/PointPanoramaTex/livox_driver2
HIK=/home/ztl/PointPanoramaTex/hik_camera

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93}
OUT=${1:-/home/ztl/slam_data/output/live_d_$(date +%m%d_%H%M%S)}

# pgrep -x 不可用：comm 截断到 15 字符（fastlivo_mappi），用 -f 精确匹配
if pgrep -f '[f]astlivo_mapping' >/dev/null; then
  echo "[WARN] 已有 fastlivo_mapping 在运行，先停止：pkill -INT -f fastlivo_mapping"
  exit 1
fi
# 上次异常退出可能留下参数服务器，同名节点会让 camera_loader 读到空参数
if pgrep -f '[p]arameter_blackboard' >/dev/null; then
  echo "[clean] 清理残留 parameter_blackboard"
  pkill -9 -f '[p]arameter_blackboard'; sleep 1
fi
mkdir -p "$OUT/log/pcd" "$OUT/log/result" "$OUT/log/image"
LOG="$OUT/app.log"
: > "$LOG"

source /opt/ros/humble/setup.bash
source "$LIVOX/install/setup.bash"
source "$SOL/ws/install/setup.bash"

# vikit_common（plain CMake）装在 .deps/vikit，需要手动加库路径
export LD_LIBRARY_PATH="$SOL/.deps/vikit/lib:${LD_LIBRARY_PATH}"

# ---- 传感器（已在跑就复用；不在则后台拉起） ----
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

probe_topics() { timeout 10 ros2 topic list 2>/dev/null; }
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
  echo "[retry] 未等到话题，强杀疑似挂死的传感器进程后重启"
  pkill -9 -f '[l]ivox_ros_driver2_node'; pkill -9 -f '[m]sg_MID360_launch'
  pkill -9 -f '[h]ik_camera_publisher\.py'
  sleep 2
  start_livox; start_hik
  if ! wait_topics 30; then
    msg="[FAIL ] 缺少 /livox/lidar 或 /left_camera/image：$(probe_topics | grep -Ei 'lidar|camera' | tr '\n' ' ')"
    echo "$msg"; echo "$msg" >> "$LOG"
    exit 1
  fi
fi

# ---- 现场标定雷达钟与相机钟的偏移（相机用宿主钟、雷达用设备钟，差约 2.05e8 s）----
if [ -z "$IMG_TIME_OFFSET" ]; then
  echo "[calib] 标定 img_time_offset（采样 8 帧）"
  CALIB=$(python3 "$SOL/scripts/calib_time_offset.py" 2>>"$LOG" | tail -1)
  IMG_TIME_OFFSET=${CALIB#IMG_TIME_OFFSET=}
  if ! [[ "$IMG_TIME_OFFSET" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
    MSG="[FAIL ] 钟差标定失败（$CALIB），可手动 IMG_TIME_OFFSET=... 重跑"
    echo "$MSG"; echo "$MSG" >> "$LOG"; exit 1
  fi
fi

# FAST-LIVO2 的输出路径由编译期 ROOT_DIR 决定（third_party/FAST-LIVO2-ROS2/Log）。
# 把 Log 软链到本次运行目录，保证每次实验产物互不覆盖。
if [ -L "$FASTLIVO/Log" ] || [ ! -e "$FASTLIVO/Log" ]; then
  rm -f "$FASTLIVO/Log"
  ln -s "$OUT/log" "$FASTLIVO/Log"
else
  echo "[WARN] $FASTLIVO/Log 是真实目录，本次产物仍写入该目录（未建软链）"
fi

echo "[run  ] ROS_DOMAIN_ID=$ROS_DOMAIN_ID img_time_offset=$IMG_TIME_OFFSET"
echo "[run  ] 输出：$OUT"
echo "[stop ] Ctrl-C（勿 kill -9），退出时写彩色 PCD"

tail -n +1 -F "$LOG" 2>/dev/null &
TAIL_PID=$!

APP_PID=
trap '[ -n "$APP_PID" ] && kill -INT "$APP_PID" 2>/dev/null' INT TERM

ros2 launch "$SOL/launch/run_d.launch.py" \
  img_time_offset:="$IMG_TIME_OFFSET" \
  use_rviz:="${USE_RVIZ:-False}" \
  >> "$LOG" 2>&1 &
APP_PID=$!

# 收到 SIGINT 后 wait 立刻返回，但 FAST-LIVO2 还在 savePCD；必须等它真正退出
while true; do
  wait "$APP_PID"
  RC=$?
  kill -0 "$APP_PID" 2>/dev/null || break
done
kill "$TAIL_PID" 2>/dev/null
wait "$TAIL_PID" 2>/dev/null

echo "[done ] rc=$RC"
echo "       彩色整图：$OUT/log/pcd/all_raw_points.pcd"
echo "       降采样图：$OUT/log/pcd/all_downsampled_points.pcd"
echo "       轨迹文件：$OUT/log/result/scene_d.txt"
