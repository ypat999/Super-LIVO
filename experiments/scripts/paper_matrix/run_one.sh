#!/usr/bin/env bash
# 论文对比矩阵：单序列 × 单前端的一次完整 1x 实时跑批。
#
#   bash experiments/scripts/paper_matrix/run_one.sh <seq> <arm>
#   seq: ntu01 ntu03 rtk retail move room
#   arm: fl2 a b sl
#
# 四臂定义：
#   fl2  官方 FAST-LIVO2（话题 /aft_mapped_to_init）
#   a    方案A：Super-LIO 主干 + super_livo_visual_a/visual_a_node（/super_livo_visual_a/odom）
#   b    方案B：Super-LIO 主干 + super_livo_visual_b/visual_b_node（/super_livo_visual_b/odom）
#   sl   官方 Super-LIO（/lio/odom）；RTK/room 用其自带视觉配置，其余数据集为 LIO-only
#
# 统一条件：FastDDS、同一 ROS_DOMAIN_ID、1x 实时播放、同一话题过滤、
# 同一 tools/latency_probe.py 外部探针。产物 <OUTROOT>/<seq>/<arm>/：
#   node.log  play.log  latency.csv  odom/  meta.txt  pcd/  done
#
# 环境变量：RATE(默认1.0) SMOKE(秒，>0 只播前 N 秒) FORCE=1 重跑已完成项
#           OUTROOT（默认 /home/ywj/slam_data/paper_20261117）
set -o pipefail

SEQ=${1:?用法: run_one.sh <seq> <arm>}
ARM=${2:?用法: run_one.sh <seq> <arm>}
RATE=${RATE:-1.0}
SMOKE=${SMOKE:-0}
OUTROOT=${OUTROOT:-/home/ywj/slam_data/paper_20261117}
REPO=/home/ywj/git/Super-LIVO
SLCFG=$REPO/third_party/Super-LIO/src/super_lio/config
EXP=$REPO/experiments/config
OUT=$OUTROOT/$SEQ/$ARM
DONE=$OUT/done

if [ -f "$DONE" ] && [ "${FORCE:-0}" != "1" ]; then
  echo "[skip] $SEQ/$ARM 已完成（$DONE）；FORCE=1 可重跑"
  exit 0
fi

# ---------------- 序列配置 ----------------
# SL_CFG  : Super-LIO 单独跑（sl 臂）用的完整参数
# SLK_CFG : A/B 臂的 LIO 主干参数（关闭内置视觉，避免与 A/B 视觉重复）
# A_CFG/B_CFG : 视觉节点参数
# FL2_CFG/FL2_CAM/FL2_MODE : 官方 FAST-LIVO2
# LIDAR / LIDAR_TYPE / IMG : 探针与播放用话题
case "$SEQ" in
  ntu01|ntu03)
    BAG=/home/ywj/slam_data/ntu_viral/bags2/eee_${SEQ#ntu}
    LIDAR=/os1_cloud_node1/points; LIDAR_TYPE=pc2
    IMG=/left/image_raw
    PLAY_TOPICS="/os1_cloud_node1/points /imu/imu /left/image_raw"
    SL_CFG=$SLCFG/NTU.yaml; SLK_CFG=$SLCFG/NTU.yaml
    A_CFG=$REPO/solution_a_lightweight/super_livo_visual_a/config/visual_a_ntu.yaml
    B_CFG=$REPO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_ntu.yaml
    FL2_BAG=$BAG; FL2_MODE=blackboard
    FL2_CFG=$REPO/third_party/FAST-LIVO2-ROS2/config/NTU_VIRAL.yaml
    FL2_CAM=$REPO/third_party/FAST-LIVO2-ROS2/config/camera_NTU_VIRAL.yaml
    ;;
  rtk)
    BAG=/home/ywj/slam_data/rtk_slam/construction_seq2_r2
    LIDAR=/livox/lidar; LIDAR_TYPE=custom
    IMG=/camera/image_raw
    PLAY_TOPICS="/livox/lidar /livox/imu /camera/image_raw/compressed"
    SL_CFG=$EXP/livox_360_visa.yaml; SLK_CFG=$EXP/livox_360_visa_off.yaml
    A_CFG=$REPO/solution_a_lightweight/super_livo_visual_a/config/visual_a_rtk.yaml
    B_CFG=$REPO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_rtk.yaml
    FL2_BAG=$BAG; FL2_MODE=env; NEED_REPUB=1
    FL2_CFG=/home/ywj/slam_data/rtk_slam/fl2_cfg/rtk_construction_lvi.yaml
    FL2_CAM=/home/ywj/slam_data/rtk_slam/fl2_cfg/camera_rtk_pinhole.yaml
    ;;
  retail)
    BAG=/home/ywj/slam_data/Retail_Street
    LIDAR=/livox/lidar; LIDAR_TYPE=custom
    IMG=/left_camera/image
    PLAY_TOPICS="/livox/lidar /livox/imu /left_camera/image"
    SL_CFG=$REPO/config/retail_lio.yaml; SLK_CFG=$REPO/config/retail_lio.yaml
    A_CFG=$REPO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
    B_CFG=$REPO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_params.yaml
    FL2_BAG=$BAG; FL2_MODE=blackboard
    FL2_CFG=$OUT/fl2_wrapped.yaml
    FL2_CAM=$BAG/camera_pinhole.yaml
    ;;
  move)
    BAG=/home/ywj/slam_data/livo_bag/move_livoxclk
    LIDAR=/livox/lidar; LIDAR_TYPE=custom
    IMG=/left_camera/image
    PLAY_TOPICS="/livox/lidar /livox/imu /left_camera/image"
    SL_CFG=$SLCFG/livox_360.yaml; SLK_CFG=$SLCFG/livox_360.yaml
    A_CFG=$REPO/solution_a_lightweight/super_livo_visual_a/config/visual_a_move.yaml
    B_CFG=$REPO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_move.yaml
    # FL2 用 move_ros2（同一段数据的另一份 bag，move_livoxclk 的 lidar 时间戳在 FL2 下不连续）
    FL2_BAG=/home/ywj/slam_data/livo_bag/move_ros2; FL2_MODE=blackboard
    FL2_CFG=$REPO/config/move_fastcalib.yaml
    FL2_CAM=$REPO/config/camera_move.yaml
    ;;
  room)
    BAG=/home/ywj/slam_data/rosbag2_fastlivo2_2026_10_30-18_00_25
    LIDAR=/livox/lidar; LIDAR_TYPE=pc2
    IMG=/camera/color/image_raw
    PLAY_TOPICS="/livox/lidar /livox/imu /camera/color/image_raw"
    SL_CFG=$EXP/livox_360_visa_room.yaml; SLK_CFG=$EXP/livox_360_visa_room_off.yaml
    A_CFG=$REPO/solution_a_lightweight/super_livo_visual_a/config/visual_a_room.yaml
    B_CFG=$REPO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_room.yaml
    # 官方 FL2 只订阅 CustomMsg，本 bag 雷达为 PointCloud2 → 该臂不适用
    FL2_MODE=none
    ;;
  *) echo "未知序列 $SEQ"; exit 2 ;;
esac

case "$ARM" in
  a) NODE_PKG=super_livo_visual_a; NODE_EXE=visual_a_node; VCFG=$A_CFG; OWN=/super_livo_visual_a/odom ;;
  b) NODE_PKG=super_livo_visual_b; NODE_EXE=visual_b_node; VCFG=$B_CFG; OWN=/super_livo_visual_b/odom ;;
  sl) OWN=/lio/odom ;;
  fl2) OWN=/aft_mapped_to_init ;;
  *) echo "未知臂 $ARM"; exit 2 ;;
esac
[ "$ARM" = "fl2" ] && [ "$FL2_MODE" = "none" ] && { echo "[skip] $SEQ 不支持官方 FL2（雷达为 PointCloud2）"; exit 0; }

mkdir -p "$OUT"
rm -rf "$OUT/odom" "$OUT/pcd" "$OUT/latency.csv" "$OUT/meta.txt" "$OUT/done"

# ---------------- 互斥锁 + 环境 ----------------
LOCK=/tmp/ros_batch.lock
exec 9>"$LOCK"
flock -n 9 || { echo "拒绝启动：$LOCK 被占用（禁止并行）"; exit 1; }
source "$REPO/tools/ros_batch_guard.sh"

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash      # livox_ros_driver2 / gazebo? 提供 msg
if [ "$ARM" = "fl2" ]; then
  source /home/ywj/fastlivo2_ws/install/setup.bash
else
  source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
  source /home/ywj/git/Super-LIVO/install/setup.bash
fi
export RMW_IMPLEMENTATION=${RMW_IMPL:-rmw_fastrtps_cpp}
# RMW 选择：本机（WSL2、无 root、net.core.rmem_max 仅 208 KB）用 CycloneDDS 回放
# move 时整条流被压到 4.5 Hz（60 s 仅 270 帧雷达，1x 实时性不成立）；
# 同条件下 FastDDS 为 9.9 Hz（593 帧/60 s）。四臂统一 FastDDS 保证可比。
# 也不要用 CYCLONEDDS_URI=tools/cyclone_lo.xml：绑 lo 后本机发现失败（收图 0）。
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93}
unset ROS_LOCALHOST_ONLY

# 清残留黑板（必须在删 shm 之前：删掉活参与者的 shm 会让它"看得见但不应答"，
# 表现为 FL2 构造时 SyncParametersClient 无限阻塞、最后抛
# "Camera model not correctly specified."）。注意 comm 被截断成 15 字符
# parameter_black，pkill -x parameter_blackboard 匹配不到，必须按可执行路径匹配。
pkill -f '/demo_nodes_cpp/parameter_[b]lackboard' 2>/dev/null
for _ in $(seq 1 10); do pgrep -f '/demo_nodes_cpp/parameter_[b]lackboard' >/dev/null || break; sleep 1; done
pkill -9 -f '/demo_nodes_cpp/parameter_[b]lackboard' 2>/dev/null

rm -f /dev/shm/fastrtps_* /dev/shm/fast_datachannel_* /dev/shm/sem.fastrtps_* 2>/dev/null

PIDS=()

# 任何路径退出都必须收掉子进程：ros2 run 的孤儿子进程会继承锁 fd（9），
# 让后续所有跑批都「锁被占用」而整段跳过（2026-11-17 rtk 四臂就是这么全丢的）。
cleanup_children() {
  trap - EXIT
  pkill -f '/demo_nodes_cpp/parameter_[b]lackboard' 2>/dev/null
  for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done
  pkill -9 -x super_lio_node 2>/dev/null
  pkill -9 -x visual_a_node 2>/dev/null
  pkill -9 -x visual_b_node 2>/dev/null
}
trap cleanup_children EXIT

echo "[$(date +%H:%M:%S)] $SEQ/$ARM start  bag=$(basename "$BAG") rate=$RATE smoke=$SMOKE"

# RTK 的相机是 CompressedImage：A/B 与 FL2 都订阅 raw，统一在这里转一次
# （Super-LIO 自带视觉可直接吃压缩图，不需要 republish）
if [ "${NEED_REPUB:-0}" = "1" ] && [ "$ARM" != "sl" ]; then
  ros2 run image_transport republish compressed raw --ros-args \
    -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_raw \
    > "$OUT/repub.log" 2>&1 &
  PIDS+=($!)
  sleep 2
fi

# ---------------- 起节点 ----------------
if [ "$ARM" = "fl2" ]; then
  # retail 的官方配置来自 bag 自带的 avia.yaml + camera_pinhole.yaml，需包装成 ROS2 参数文件
  if [ "$SEQ" = "retail" ]; then
    python3 "$REPO/experiments/octvox_ablation/07a_wrap_official_cfg.py" \
      "$BAG/avia.yaml" "$FL2_CFG" "$BAG/camera_pinhole.yaml" > /dev/null 2>&1
  fi
  if [ "$FL2_MODE" = "env" ]; then
    source /home/ywj/slam_data/rtk_slam/fl2_cfg/fl2_env.sh
    FL2_ARGS=(--ros-args --params-file "$FL2_CFG")
  else  # blackboard：FL2 的 camera_loader 只从 parameter_blackboard 取 cam_model
        # （LIVMapper.cpp:220），不起黑板会直接抛
        # "Camera model not correctly specified."（move/retail 曾因此 0 odom）
    ros2 run demo_nodes_cpp parameter_blackboard --ros-args \
      --params-file "$FL2_CAM" > "$OUT/blackboard.log" 2>&1 &
    BB=$!; PIDS+=("$BB")
    # 就绪校验：黑板参数服务必须真的应答，否则 FL2 会在构造里卡死（白跑一整段）
    BB_OK=0
    for _ in $(seq 1 30); do
      if timeout 5 ros2 param get /parameter_blackboard cam_model 2>/dev/null | grep -q Pinhole; then
        BB_OK=1; break
      fi
      sleep 1
    done
    if [ "$BB_OK" != "1" ]; then
      echo "[$SEQ/$ARM] 黑板相机参数不可读（参数服务无应答），放弃本次"; exit 1
    fi
    # vikit 的 getRemoteParam 只 wait_for_service(100 ms)（params_helper.h:117），
    # 服务没在 100 ms 内被本进程发现就静默退化成默认值 → 立即抛
    # "Camera model not correctly specified."。实测这是进程级发现竞态（黑板本身正常），
    # 故多留几秒再起节点，并对这种"秒退"重试。
    sleep 5
    FL2_ARGS=(--ros-args --params-file "$FL2_CFG" --params-file "$FL2_CAM")
  fi
  FASTLIVO_BIN=/home/ywj/fastlivo2_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping
  FL2_TRIES=3
  for FL2_TRY in $(seq 1 $FL2_TRIES); do
    "$FASTLIVO_BIN" "${FL2_ARGS[@]}" \
      -p evo.seq_name:="${SEQ}_${ARM}" -p pcd_save.pcd_save_en:=true -p pcd_save.interval:=200 \
      > "$OUT/node.log" 2>&1 &
    NODE_PID=$!
    sleep 8
    if kill -0 "$NODE_PID" 2>/dev/null; then break; fi
    if grep -q 'Camera model not correctly specified' "$OUT/node.log" && [ "$FL2_MODE" != "env" ]; then
      echo "[$SEQ/$ARM] FL2 相机内参远程读取竞态（第 $FL2_TRY/$FL2_TRIES 次），重试"
      sleep 3
      continue
    fi
    break
  done
else
  if [ "$ARM" = "sl" ]; then
    SL_ARGS=(--ros-args --params-file "$SL_CFG"
             -p lio.map.save_map:=true -p lio.map.save_map_dir:="$OUT/pcd")
  else
    SL_ARGS=(--ros-args --params-file "$SLK_CFG"
             -p lio.sc_pgo.enable:=false -p lio.map.save_map:=false)
    # NTU 沿用 tools/run_ntu.sh 的退化检测设置，保证与既有 A/B 结果同口径
    if [ "$SEQ" = "ntu01" ] || [ "$SEQ" = "ntu03" ]; then
      SL_ARGS+=(-p lio.observe.degeneracy_detect_en:=true -p lio.observe.degen_trans_only:=1)
    fi
  fi
  ros2 run super_lio super_lio_node "${SL_ARGS[@]}" > "$OUT/backbone.log" 2>&1 &
  NODE_PID=$!
  sleep 6
  if [ "$ARM" != "sl" ]; then
    ros2 run "$NODE_PKG" "$NODE_EXE" --ros-args --params-file "$VCFG" \
      -p pcd.save_dir:="$OUT/pcd" > "$OUT/node.log" 2>&1 &
    NODE_PID=$!
  fi
fi
PIDS+=("$NODE_PID")

# 等节点就绪（DDS 发现 + 首帧前初始化）
sleep 8
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "[$SEQ/$ARM] 节点启动失败，日志尾部："; tail -25 "$OUT/node.log" 2>/dev/null || tail -25 "$OUT/backbone.log"
  exit 1
fi

# ---------------- 探针 + 录 odom ----------------
python3 "$REPO/tools/latency_probe.py" --lidar-topic "$LIDAR" --lidar-type "$LIDAR_TYPE" \
  --odom-topic "$OWN" --out "$OUT/latency.csv" > "$OUT/probe.log" 2>&1 &
PROBE=$!; PIDS+=("$PROBE")

ros2 bag record -o "$OUT/odom" "$OWN" > "$OUT/rec.log" 2>&1 &
REC=$!; PIDS+=("$REC")
sleep 3

# ---------------- 播包 ----------------
PLAY_ARGS=(--rate "$RATE" --disable-keyboard-controls)
PLAY_BAG=$BAG
[ "$ARM" = "fl2" ] && PLAY_BAG=$FL2_BAG
if [ "$SMOKE" -gt 0 ]; then
  timeout "$SMOKE" ros2 bag play "$PLAY_BAG" "${PLAY_ARGS[@]}" --topics $PLAY_TOPICS < /dev/null > "$OUT/play.log" 2>&1
else
  ros2 bag play "$PLAY_BAG" "${PLAY_ARGS[@]}" --topics $PLAY_TOPICS < /dev/null > "$OUT/play.log" 2>&1
fi
echo "[$(date +%H:%M:%S)] 播完，收尾"

# ---------------- 收尾 ----------------
sleep 3
kill -INT "$REC" 2>/dev/null; sleep 4; kill -9 "$REC" 2>/dev/null
kill -INT "$PROBE" 2>/dev/null; sleep 2; kill -9 "$PROBE" 2>/dev/null

if [ "$ARM" = "fl2" ]; then
  kill -INT "$NODE_PID" 2>/dev/null
  # FL2 退出时才 savePCD（千万点级要几分钟）
  for _ in $(seq 1 180); do kill -0 "$NODE_PID" 2>/dev/null || break; sleep 2; done
  kill -9 "$NODE_PID" 2>/dev/null
  mkdir -p "$OUT/pcd"
  mv /home/ywj/fastlivo2_ws/src/fast_livo/Log/pcd/*.pcd "$OUT/pcd"/ 2>/dev/null
  # 位姿文件（FL2 自带）
  find /home/ywj/fastlivo2_ws/src/fast_livo/Log -name "*${SEQ}_${ARM}*.txt" -newermt "-2 hours" \
    -exec mv {} "$OUT"/ \; 2>/dev/null
  # 黑板是 ros2 run 起的，父进程 kill 后子进程会活着并占用 /parameter_blackboard，
  # 下一轮同名服务就会「可见但不应答」→ 按可执行路径连子进程一起收掉。
  pkill -f '/demo_nodes_cpp/parameter_[b]lackboard' 2>/dev/null
else
  # ros2 run 的 wrapper 与真正的节点是父子两个进程：只 kill wrapper 会留下孤儿节点
  # （实测方案A节点在下一轮启动时被 guard 拦下）。按可执行名精确收尾。
  if [ "$ARM" = "sl" ]; then EXE=super_lio_node; else EXE=$NODE_EXE; fi
  pkill -INT -x "$EXE" 2>/dev/null
  for _ in $(seq 1 40); do pgrep -x "$EXE" >/dev/null || break; sleep 2; done
  pkill -9 -x "$EXE" 2>/dev/null
  [ "$ARM" != "sl" ] && pkill -9 -x super_lio_node 2>/dev/null
fi
for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done
sleep 2

{
  echo "seq=$SEQ arm=$ARM"
  echo "bag=$PLAY_BAG"
  echo "topics=$PLAY_TOPICS"
  echo "rmw=$RMW_IMPLEMENTATION domain=$ROS_DOMAIN_ID rate=$RATE smoke=$SMOKE"
  echo "start_out=$OUT"
  echo "odom_topic=$OWN"
  echo "odom_rows=$(ros2 bag info "$OUT/odom" 2>/dev/null | grep -i 'Messages:' | grep -o '[0-9]\+' | head -1)"
  echo "latency_lines=$(wc -l < "$OUT/latency.csv" 2>/dev/null)"
  echo "pcd=$(du -sh "$OUT/pcd" 2>/dev/null | cut -f1) files=$(ls "$OUT/pcd"/*.pcd 2>/dev/null | wc -l)"
  echo "end=$(date -Is)"
} > "$OUT/meta.txt"
cat "$OUT/meta.txt"
# 失败门控：0 条 odom 说明该臂没跑起来（如 FL2 缺相机内参直接 abort），
# 不写 done，断点续跑时会自动重试；否则会被误判为已完成而永久跳过。
ODOM_N=$(sed -n 's/^odom_rows=\([0-9]*\).*/\1/p' "$OUT/meta.txt")
if [ "${ODOM_N:-0}" = "0" ]; then
  echo "[$SEQ/$ARM] 0 条 odom，未写 done；修复后 FORCE=1 重跑"
  exit 1
fi
[ "$SMOKE" -gt 0 ] && touch "$OUT/smoke"
[ "$SMOKE" -gt 0 ] || touch "$DONE"
echo "[$(date +%H:%M:%S)] $SEQ/$ARM done"
