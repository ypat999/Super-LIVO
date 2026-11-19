#!/usr/bin/env bash
# 论文对比矩阵：单序列 × 单前端的一次完整 1x 实时跑批。
#
#   bash experiments/scripts/paper_matrix/run_one.sh <seq> <arm>
#   seq: ntu01 ntu03 rtk retail move test4 room
#   arm: fl2 a b sl
#
# 四臂定义：
#   fl2  官方 FAST-LIVO2（话题 /aft_mapped_to_init）
#   a    方案A：Super-LIO 主干 + super_livo_visual_a/visual_a_node（/super_livo_visual_a/odom）
#   b    方案B：单体 visual_b_app（LIO 主干 + 视觉支路同进程，/super_livo_visual_b/odom）
#   sl   官方 Super-LIO（/lio/odom）；RTK/room 用其自带视觉配置，其余数据集为 LIO-only
#
# 统一条件：FastDDS、同一 ROS_DOMAIN_ID、1x 实时播放、同一话题过滤、
# 同一 tools/latency_probe_cpp 外部探针（C++ generic subscription，不反序列化）。
# 产物 <OUTROOT>/<seq>/<arm>/：
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
    # 相机参数走 parameter_blackboard：当前 vikit 的 getRemoteParam 没有环境变量
    # 回退分支（旧 fl2_env.sh 那套 FL2_<NODE>_<PARAM> 不生效），env 模式会直接抛
    # "Camera model not correctly specified."
    FL2_BAG=$BAG; FL2_MODE=blackboard; NEED_REPUB=1
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
  move|test4)
    # 原 paper 轮用 move_livoxclk（图像 header 已改写为 Livox 时钟，A/B 的 yaml 里
    # img_time_offset 写 0.0），该副本已不在磁盘；现存的是 move_ros2 与 test4_move：
    # 图像 header 均保留主机时钟，需用 -p 覆盖 img_time_offset。
    #   move_ros2：偏移 -204262818.9974 s（与 FL2 config/move_fastcalib.yaml 同源）
    #   test4_move：偏移 -204701439.4937 s（实测图像帧戳与雷达帧戳常量差，
    #               全段漂移 < 5 ms，即硬件触发同步成立，只是时钟域不同）
    if [ "$SEQ" = "test4" ]; then
      # 源 bag 是 mcap，本机没装 mcap 存储插件（无 root 不能 apt），ros2 bag play 打不开，
      # 故用 rosbags 无损转成 sqlite3（仅三路传感器话题，消息字节原样搬运）。
      BAG=/home/ywj/slam_data/livo_bag/test4_move_ros2
      MOVE_IMG_OFF=-204701439.4937
    else
      BAG=/home/ywj/slam_data/livo_bag/move_ros2
      MOVE_IMG_OFF=-204262818.9974
    fi
    LIDAR=/livox/lidar; LIDAR_TYPE=custom
    IMG=/left_camera/image
    PLAY_TOPICS="/livox/lidar /livox/imu /left_camera/image"
    SL_CFG=$SLCFG/livox_360.yaml; SLK_CFG=$SLCFG/livox_360.yaml
    A_CFG=$REPO/solution_a_lightweight/super_livo_visual_a/config/visual_a_move.yaml
    B_CFG=$REPO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_move.yaml
    FL2_BAG=$BAG; FL2_MODE=blackboard
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
  # b 臂已单体化：LIO 主干与视觉支路同一进程（visual_b_app），不再单独起 super_lio_node
  b) NODE_PKG=super_livo_visual_b; NODE_EXE=visual_b_app; VCFG=$B_CFG; OWN=/super_livo_visual_b/odom ;;
  sl) OWN=/lio/odom ;;
  fl2) OWN=/aft_mapped_to_init ;;
  *) echo "未知臂 $ARM"; exit 2 ;;
esac
[ "$ARM" = "fl2" ] && [ "$FL2_MODE" = "none" ] && { echo "[skip] $SEQ 不支持官方 FL2（雷达为 PointCloud2）"; exit 0; }

# 图像时间补偿：move 用 move_ros2（图像 header 为主机时钟），A/B 的 yaml 是按
# move_livoxclk 写的 0.0，这里按实测差值覆盖，否则 t_img 会落到 2026 年。
VOVER=()
[ -n "${MOVE_IMG_OFF:-}" ] && [ "$ARM" != "sl" ] && [ "$ARM" != "fl2" ] && \
  VOVER=(-p img_time_offset:=$MOVE_IMG_OFF)

mkdir -p "$OUT"
rm -rf "$OUT/odom" "$OUT/pcd" "$OUT/latency.csv" "$OUT/meta.txt" "$OUT/done"

# ---------------- 互斥锁 + 环境 ----------------
LOCK=/tmp/ros_batch.lock
exec 9>"$LOCK"
flock -n 9 || { echo "拒绝启动：$LOCK 被占用（禁止并行）"; exit 1; }

# ---------------- 上一臂残留清理 ----------------
# 必须在 ros_batch_guard 之前：guard 见到任何 ROS 节点就拒绝启动，而它拒绝的
# 往往正是上一臂泄漏的孤儿（rtk/a 的 republish 真身没被杀，rtk/b、rtk/fl2
# 四连拒就是这么来的）。此刻锁已在手上，别的跑批进不来，场上的都是孤儿，可安全清理。
#
# 为什么必须清干净：
#   1) 泄漏的参与者仍占着 FastDDS 共享内存端口，新参与者 init_port 失败
#      （rec.log 里 "open_and_lock_file failed"）→ 该臂主干 lidar_buf=0、
#      frame_num=0，整段 0 odom（retail/b 连续两轮 0 odom 就是这么来的）；
#   2) 黑板「看得见但不应答」→ FL2 构造期 SyncParametersClient 卡死/抛
#      "Camera model not correctly specified."。
# 注意 comm 被截断成 15 字符（parameter_black），pkill -x 匹配不到，只能按命令行匹配。
# republish 要匹配 ros2 run 的**真身** /opt/ros/humble/lib/image_transport/republish：
# ros2 run 只是个 wrapper，杀 wrapper 不会带走它 spawn 的子进程。
LEFTOVER_RE='/demo_nodes_cpp/parameter_[b]lackboard|super_lio_node|visual_a_node|visual_b_app|visual_b_node|fastlivo_mapping|latency_probe|image_transport[/ ]republish|ros2 bag play|ros2 bag record'
pkill -f "$LEFTOVER_RE" 2>/dev/null
for _ in $(seq 1 15); do
  pgrep -f "$LEFTOVER_RE" > /dev/null || break
  sleep 1
done
pkill -9 -f "$LEFTOVER_RE" 2>/dev/null
sleep 1

rm -f /dev/shm/fastrtps_* /dev/shm/fast_datachannel_* /dev/shm/sem.fastrtps_* 2>/dev/null

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

# （残留清理已上移到取锁之后、guard 之前，见上方 LEFTOVER_RE）
PIDS=()

# 任何路径退出都必须收掉子进程：ros2 run 的孤儿子进程会继承锁 fd（9），
# 让后续所有跑批都「锁被占用」而整段跳过（2026-11-17 rtk 四臂就是这么全丢的）。
cleanup_children() {
  trap - EXIT
  for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done
  # ros2 run 只是 wrapper：kill wrapper 带不走它 spawn 的真身
  # （super_lio_node / republish / fastlivo_mapping…），必须按 LEFTOVER_RE 再收一遍，
  # 否则孤儿会占着 FastDDS SHM 端口，并把下一臂的 guard 直接顶回去。
  pkill -9 -f "$LEFTOVER_RE" 2>/dev/null
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
    sleep 3
    # 就绪检查只作提示，不作门槛：ros2 param get --no-daemon 每次都要新起进程
    # 重新做一遍 DDS 发现，机器正在收上一臂的尾时 5 s 常常发现不完 → 连续 20 次
    # 判"不应答"，而 blackboard.log 里黑板其实一直 "serving '17' parameters"。
    # 之前 retail/fl2、ntu03/fl2 就是这么在起 FL2 之前被误杀的。
    # 真正的判据是下面 FL2 有没有打印 "^scale:"。
    if timeout 10 ros2 param get --no-daemon /parameter_blackboard cam_model 2>/dev/null | grep -q Pinhole; then
      echo "[$SEQ/$ARM] 黑板已就绪"
    else
      echo "[$SEQ/$ARM] ros2 param get 未读到黑板（仅提示，继续起 FL2）"
    fi
    # vikit 的 getRemoteParam 只 wait_for_service(100 ms)（params_helper.h:117），
    # 服务没在 100 ms 内被本进程发现就静默退化成默认值 → 立即抛
    # "Camera model not correctly specified."。实测这是进程级发现竞态（黑板本身正常），
    # 故多留几秒再起节点，并对这种"秒退"重试。
    sleep 5
    FL2_ARGS=(--ros-args --params-file "$FL2_CFG" --params-file "$FL2_CAM")
  fi
  FASTLIVO_BIN=/home/ywj/fastlivo2_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping
  # 清掉上一轮残留在 FL2 Log/pcd 的落盘文件，避免本轮的 pcd 与旧文件混在一起
  rm -f /home/ywj/fastlivo2_ws/src/fast_livo/Log/pcd/*.pcd
  FL2_TRIES=3
  FL2_OK=0
  for FL2_TRY in $(seq 1 $FL2_TRIES); do
    # interval<0：不在运行中按 N 帧切片，退出时 savePCD() 一次性写
    # all_raw_points.pcd(+all_downsampled_points.pcd)（LIVMapper.cpp:725）
    # vio.visual_map_window_sec=60：官方视觉地图点永不删除且每个点持有建点帧的
    # 整幅图像金字塔（实测 ≈50 MB/s 线性增长），rtk（599 s）峰值 22.9 GB 被 OOM
    # 杀掉 → 退出时 savePCD() 不执行、该臂 0 pcd。60 s 滚动窗口只为控内存，
    # 四臂统一口径都用这条参数。
    "$FASTLIVO_BIN" "${FL2_ARGS[@]}" \
      -p evo.seq_name:="${SEQ}_${ARM}" -p pcd_save.pcd_save_en:=true -p pcd_save.interval:=-1 \
      -p vio.visual_map_window_sec:=60.0 \
      > "$OUT/node.log" 2>&1 &
    NODE_PID=$!
    # 成功判定用相机参数是否真的取到：PinholeCamera 构造后立刻打印 "scale: ..."，
    # 失败是构造期抛异常秒退。只看进程活着会把"还在等黑板"误判成成功。
    for _ in $(seq 1 25); do
      if grep -q '^scale:' "$OUT/node.log" 2>/dev/null; then FL2_OK=1; break; fi
      kill -0 "$NODE_PID" 2>/dev/null || break
      sleep 1
    done
    [ "$FL2_OK" = "1" ] && break
    kill -9 "$NODE_PID" 2>/dev/null
    if [ "$FL2_MODE" = "env" ]; then break; fi
    echo "[$SEQ/$ARM] FL2 没取到相机内参（第 $FL2_TRY/$FL2_TRIES 次），重起黑板后重试"
    # 黑板可能已「可见但不应答」，连同 ros2 run 的子进程一起重起
    pkill -f '/demo_nodes_cpp/parameter_[b]lackboard' 2>/dev/null
    sleep 1
    pkill -9 -f '/demo_nodes_cpp/parameter_[b]lackboard' 2>/dev/null
    ros2 run demo_nodes_cpp parameter_blackboard --ros-args \
      --params-file "$FL2_CAM" > "$OUT/blackboard.log" 2>&1 &
    BB=$!; PIDS+=("$BB")
    # 重起后同样只等发现、不拿 ros2 param get 当门槛（原因见上）
    sleep 5
  done
  if [ "$FL2_OK" != "1" ]; then
    echo "[$SEQ/$ARM] FL2 三次都没取到相机内参，放弃本次（见 $OUT/node.log）"
    exit 1
  fi
else
  if [ "$ARM" = "sl" ]; then
    # save_interval<0：不做 500 帧切片（PCD/scans_*.pcd），改由 /map_save 服务
    # 调 saveMap() 一次性 voxel 降采样落成 map_name 单文件（livox_360.yaml 注释口径）
    # 服务名显式统一：各 SL 配置里是 /map_save_lio 或默认 /map_save（NTU.yaml），
    # 且机器上可能残留同名服务，用专属名字避免调错。
    SL_ARGS=(--ros-args --params-file "$SL_CFG"
             -p lio.map.save_map:=true -p lio.map.save_map_dir:="$OUT/pcd"
             -p lio.map.save_interval:=-1 -p lio.map.map_name:="${SEQ}_${ARM}_map.pcd"
             -p lio.ros.map_save_service_topic:=/map_save_paper)
  elif [ "$ARM" = "b" ]; then
    # 方案 B 已单体化（visual_b_app）：LIO 主干与视觉支路同一进程，不再单独起
    # super_lio_node。两份 params-file 一起喂给同一节点；SLK_CFG 与 B 配置的顶层
    # 都是 /** 通配块，所以节点名保持默认 super_lio 也能全部生效。
    APP_ARGS=(--ros-args --params-file "$SLK_CFG" --params-file "$VCFG"
              -p lio.sc_pgo.enable:=false -p lio.map.save_map:=false)
    # NTU 沿用 tools/run_ntu.sh 的退化检测设置，保证与既有 A/B 结果同口径
    if [ "$SEQ" = "ntu01" ] || [ "$SEQ" = "ntu03" ]; then
      APP_ARGS+=(-p lio.observe.degeneracy_detect_en:=true -p lio.observe.degen_trans_only:=1)
    fi
    APP_ARGS+=("${VOVER[@]}" -p pcd.save_dir:="$OUT/pcd")
    ros2 run "$NODE_PKG" "$NODE_EXE" "${APP_ARGS[@]}" > "$OUT/node.log" 2>&1 &
    NODE_PID=$!
  else
    SL_ARGS=(--ros-args --params-file "$SLK_CFG"
             -p lio.sc_pgo.enable:=false -p lio.map.save_map:=false)
    # NTU 沿用 tools/run_ntu.sh 的退化检测设置，保证与既有 A/B 结果同口径
    if [ "$SEQ" = "ntu01" ] || [ "$SEQ" = "ntu03" ]; then
      SL_ARGS+=(-p lio.observe.degeneracy_detect_en:=true -p lio.observe.degen_trans_only:=1)
    fi
    ros2 run super_lio super_lio_node "${SL_ARGS[@]}" > "$OUT/backbone.log" 2>&1 &
    NODE_PID=$!
    sleep 6
    ros2 run "$NODE_PKG" "$NODE_EXE" --ros-args --params-file "$VCFG" \
      "${VOVER[@]}" -p pcd.save_dir:="$OUT/pcd" > "$OUT/node.log" 2>&1 &
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
# 用 C++ 探针（tools/latency_probe_cpp）：generic subscription 拿序列化字节，
# 只读 12 字节 header.stamp，单帧开销亚微秒级。Python 版探针在重负载序列上
# 自身会滞后（lidar recv−stamp 抖动 41 s、odom 90~110 s，algo 出现负值）。
PROBE_BIN=$REPO/install/latency_probe_cpp/lib/latency_probe_cpp/latency_probe
if [ ! -x "$PROBE_BIN" ]; then
  echo "缺少探针 $PROBE_BIN（先 colcon build --packages-select latency_probe_cpp）"; exit 1
fi
"$PROBE_BIN" --lidar-topic "$LIDAR" --lidar-type "$LIDAR_TYPE" \
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
  # 只留最终一个 pcd：interval<0 时 FL2 写 all_raw_points.pcd（img_en 时是彩色全量）
  # 与 all_downsampled_points.pcd（体素降采样）。保留全量并统一命名，其余删除。
  if [ -f "$OUT/pcd/all_raw_points.pcd" ]; then
    mv -f "$OUT/pcd/all_raw_points.pcd" "$OUT/pcd/${SEQ}_${ARM}_map.pcd"
  elif [ -f "$OUT/pcd/all_downsampled_points.pcd" ]; then
    mv -f "$OUT/pcd/all_downsampled_points.pcd" "$OUT/pcd/${SEQ}_${ARM}_map.pcd"
  fi
  for f in "$OUT/pcd"/*.pcd; do
    [ -e "$f" ] || continue
    [ "$(basename "$f")" = "${SEQ}_${ARM}_map.pcd" ] || rm -f "$f"
  done
  # 位姿文件（FL2 自带）
  find /home/ywj/fastlivo2_ws/src/fast_livo/Log -name "*${SEQ}_${ARM}*.txt" -newermt "-2 hours" \
    -exec mv {} "$OUT"/ \; 2>/dev/null
  # 黑板是 ros2 run 起的，父进程 kill 后子进程会活着并占用 /parameter_blackboard，
  # 下一轮同名服务就会「可见但不应答」→ 按可执行路径连子进程一起收掉。
  pkill -f '/demo_nodes_cpp/parameter_[b]lackboard' 2>/dev/null
else
  # SL 臂：save_interval<0 时地图只在 /map_save 服务里一次性 voxel 降采样落盘
  # （SuperLIO::saveMap()，super_lio.cpp:1223-1245），必须在节点还活着时调用，
  # 否则本臂没有任何 pcd 产物。
  if [ "$ARM" = "sl" ]; then
    echo "[$(date +%H:%M:%S)] /map_save_paper 合并落盘（全量地图 voxel 降采样，可能数十秒）"
    timeout 600 ros2 service call /map_save_paper std_srvs/srv/Trigger "{}" > "$OUT/map_save.log" 2>&1 \
      || echo "[$SEQ/$ARM] /map_save_paper 调用失败，见 $OUT/map_save.log"
  fi
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

# SL 的 SC-PGO 还会按帧写 Scans/（每帧一个 pcd）与 PCD/ 分片、odom_poses.txt。
# 产物只保留一次性合并的 map 单文件，逐帧分片全部删掉，轨迹挪到臂目录。
if [ "$ARM" = "sl" ]; then
  rm -rf "$OUT/pcd/Scans" "$OUT/pcd/PCD"
  [ -f "$OUT/pcd/odom_poses.txt" ] && mv -f "$OUT/pcd/odom_poses.txt" "$OUT/odom_poses.txt"
fi

# 产物校验：每个臂必须只留一个最终 PCD（A/B 为节点自带的 *_colored.pcd）
PCD_N=$(find "$OUT/pcd" -name '*.pcd' 2>/dev/null | wc -l)
if [ "$PCD_N" != "1" ]; then
  echo "[$SEQ/$ARM] 警告：$OUT/pcd 下有 $PCD_N 个 pcd（预期 1）"
  find "$OUT/pcd" -name '*.pcd' 2>/dev/null | head
fi

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
# B 臂硬校验：一条 IMU 都没收到时，B 节点只能退化成"跟随 lio/odom"（不是紧耦合结果）。
# 批量跑批实测出现过 IMU 投递偶发失效，该次 path 1.33 m（单独重跑 9.77 m）。
# 判据用初始化日志行，它只在 imu_buffer_ 非空时打印；不写 done，断点续跑会自动重跑。
if [ "$ARM" = "b" ] && ! grep -q 'IMU 加计归一化' "$OUT/node.log" 2>/dev/null; then
  echo "[$SEQ/$ARM] 未收到 IMU（node.log 无「IMU 加计归一化」），该次不是有效 B 结果，未写 done"
  exit 1
fi
[ "$SMOKE" -gt 0 ] && touch "$OUT/smoke"
[ "$SMOKE" -gt 0 ] || touch "$DONE"
echo "[$(date +%H:%M:%S)] $SEQ/$ARM done"
