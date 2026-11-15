#!/usr/bin/env bash
# 诊断：节点卡在哪个初始化阶段
set -o pipefail
CFG=/home/ywj/slam_data/fastlivo2_ref_20261110/avia_wrapped.yaml
BIN=/home/ywj/fastlivo2_ws/build/fast_livo/fastlivo_mapping
OUT=/tmp/probe_vio2
rm -rf "$OUT"; mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/fastlivo2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

pkill -f 'fastlivo_mapp''ing' >/dev/null 2>&1; sleep 2
stdbuf -oL -eL "$BIN" --ros-args --params-file "$CFG" > "$OUT/node.log" 2>&1 &
sleep 8
echo "=== ps tree ==="
ps -eo pid,ppid,stat,etime,comm | grep -E 'fastlivo|ros2' | grep -v grep
echo "=== ros2 node list ==="
timeout 10 ros2 node list --no-daemon 2>&1 | head -10
echo "=== topics ==="
timeout 10 ros2 topic list --no-daemon 2>&1 | head -20
echo "=== log ==="
cat "$OUT/node.log"
PID=$(pgrep -x fastlivo_mapping | head -1)
echo "=== pid=$PID ==="
if [ -n "$PID" ]; then
  echo "--- /proc/$PID/status State ---"; grep -E '^(State|Threads)' /proc/$PID/status
  echo "--- wchan ---"; cat /proc/$PID/wchan; echo
  if command -v gdb >/dev/null 2>&1; then
    echo "--- gdb bt ---"
    timeout 40 gdb -p "$PID" -batch -ex "thread apply all bt" 2>&1 | head -80
  else
    echo "no gdb"
  fi
fi
pkill -f 'fastlivo_mapp''ing' >/dev/null 2>&1
