# 跑批互斥的第二道闸 + "只杀同批"的收尾工具，由 run_fl2_rtk.sh / run_ntu.sh /
# run_rtk.sh 以及临时 /tmp/run_ntu_degen.sh 共用。必须在拿到 /tmp/ros_batch.lock
# 之后 source。
#
#   source /home/ywj/git/Super-LIVO/tools/ros_batch_guard.sh
#   ros2 run ... &  batch_pid $!      # 每起一个后台进程登记一次
#   batch_kill INT                    # 优雅收尾（SIGINT 才会触发 savePCD 等）
#   batch_kill KILL                   # 兜底清理
#
# 为什么光有 flock 不够：锁只拦得住"同样用这把锁"的脚本。/tmp 下的临时跑批、
# 手工 ros2 run 起来的节点不持锁，flock 会成功，然后各脚本开头的
# pkill -f fastlivo_mapping / image_transport/republish / -x super_lio_node
# 就会把别人正在用的节点打死 —— 实测毁掉过一整轮 NTU degen 建图（odom bag 只剩 24 KB）。
#
# 所以拿到锁以后还要确认场上根本没有活节点：有就说明有人在锁外跑东西，
# 直接拒绝启动，绝不去杀它（杀谁、等不等，是人的决定，不是脚本的）。
# 从此各脚本一律用 batch_kill 按登记过的 PID 收尾，不再按进程名全局 pkill。
# 注意模式必须 "^..." 锚定到命令行开头：pgrep -f 是子串匹配，不锚定时会命中
# "任何 cmdline 里写着这些词的进程" —— 实测我自己那条
#   bash -lc '... pgrep -a -f "ros2 bag play" ...'
# 的启动 shell 就被当成活节点，把整轮拦在门外。锚定后只认
#   /home/.../fast_livo/fastlivo_mapping --ros-args      （真节点）
#   /usr/bin/python3 /opt/ros/humble/bin/ros2 bag play   （真播包）
# 这类以可执行文件路径开头的 cmdline。
_LIVEPAT='^(/home/[^ ]+/)?(fastlivo_mapping|super_lio_node|visual_a_node|visual_b_node)( |$)'
_LIVEPAT="$_LIVEPAT|^(/usr/bin/)?python3? /tmp/cloud_crop.py"
_LIVEPAT="$_LIVEPAT|^(/usr/bin/)?python3? (/opt/ros/humble/bin/)?ros2 bag (play|record)"
_LIVEPAT="$_LIVEPAT|^/opt/ros/humble/lib/image_transport/republish"
_live=$(pgrep -a -f "$_LIVEPAT" 2>/dev/null | grep -vw "$$")
if [ -n "$_live" ]; then
  echo "拒绝启动：$LOCK 空闲，但场上还有在跑的 ROS 节点（不持锁的跑批或手工节点）。"
  echo "本脚本不会去杀它们；请先停掉那一轮，或把它也纳入 $LOCK。"
  echo "$_live"
  unset _live
  exit 1
fi
unset _live

BATCH_PIDFILE=/tmp/ros_batch_pids.$$
: > "$BATCH_PIDFILE"

batch_pid() { [ -n "$1" ] && echo "$1" >> "$BATCH_PIDFILE"; return 0; }

batch_kill() {
  local _sig=${1:-TERM} _p
  [ -f "$BATCH_PIDFILE" ] || return 0
  while read -r _p; do
    [ -z "$_p" ] && continue
    pkill -"$_sig" -P "$_p" 2>/dev/null   # ros2 run 的子节点（真正的 node）
    kill -"$_sig" "$_p" 2>/dev/null
  done < "$BATCH_PIDFILE"
  [ "$_sig" = "KILL" ] && rm -f "$BATCH_PIDFILE"
  return 0
}
