#!/usr/bin/env bash
# Retail_Street 官方 FAST-LIVO2「有图像 / 去除图像」成对消融：odom + PCD 同时落盘。
# 3 轮 × 2 条件，交错执行。odom 已打时基补丁（02_patch_odom_timebase.py），
# /aft_mapped_to_init.header.stamp 就是 lidar 帧时刻，无需再用 node.log 反推。
#
#   bash experiments/fl2_img_ablation/03_run_retail_ablation.sh
#
# 产物：OUT/retail/m{img_en}r{rep}_{odom/ bag, node.log, raw.pcd, down.pcd, pose.txt}
#
# 注意：必须直接起可执行文件而不是 `ros2 run`，否则 SIGINT 只杀掉 wrapper、
# 真正的节点收不到退出信号 → `run()` 的 while(rclcpp::ok()) 不结束 → savePCD()
# 永远不被调用（11-10 17:39 那次冒烟测试就是这么丢了 PCD）。
set -o pipefail

REPS=${REPS:-3}
OUT=/home/ywj/slam_data/fl2_img_abl_20261110
FLLOG=/home/ywj/git/Super-LIVO/third_party/FAST-LIVO2-ROS2/Log
BIN=/home/ywj/fastlivo2_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping
BAG=/home/ywj/slam_data/Retail_Street
QOS=/tmp/rec_pair_qos.yaml
mkdir -p "$OUT/retail"

CFG=$OUT/avia_wrapped_pair.yaml
python3 /home/ywj/git/Super-LIVO/experiments/octvox_ablation/07a_wrap_official_cfg.py \
    "$BAG/avia.yaml" "$CFG" "$BAG/camera_pinhole.yaml"

cat > "$QOS" <<'YAML'
/aft_mapped_to_init:
  reliability: reliable
  history: keep_all
  depth: 4000
YAML

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

for r in $(seq 1 "$REPS"); do
  for m in 1 0; do
    tag="m${m}r${r}"
    d="$OUT/retail/$tag"
    rm -rf "$d"; mkdir -p "$d"
    rm -f "$FLLOG/result/Retail_$tag.txt" "$FLLOG/pcd/all_raw_points.pcd" \
          "$FLLOG/pcd/all_downsampled_points.pcd"
    pkill -x fastlivo_mapping >/dev/null 2>&1; sleep 2
    echo "===== [$(date +%H:%M:%S)] Retail img_en=$m rep=$r ====="

    $BIN --ros-args --params-file "$CFG" \
        -p common.img_en:="$m" \
        -p evo.seq_name:="Retail_$tag" -p evo.pose_output_en:=true \
        -p pcd_save.pcd_save_en:=true -p pcd_save.interval:=-1 \
        > "$d/node.log" 2>&1 &
    NODE=$!
    sleep 10
    if ! kill -0 "$NODE" 2>/dev/null; then
      echo "[$tag] !! node 启动失败"; tail -25 "$d/node.log"; continue
    fi

    ros2 bag record -o "$d/odom" --qos-profile-overrides-path "$QOS" \
        /aft_mapped_to_init > "$d/rec.log" 2>&1 &
    REC=$!
    sleep 3
    T0=$(date +%s)
    ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
        > "$d/play.log" 2>&1
    echo "[$tag] 播放耗时 $(( $(date +%s) - T0 )) s（bag 本身 135.5 s）"
    sleep 3
    kill -INT "$REC" 2>/dev/null; wait "$REC" 2>/dev/null

    kill -INT "$NODE" 2>/dev/null
    for _ in $(seq 1 150); do          # 等 savePCD 写完（img_en=0 约 927 MB）
      kill -0 "$NODE" 2>/dev/null || break
      sleep 2
    done
    kill -9 "$NODE" 2>/dev/null
    sleep 2

    mv "$FLLOG/result/Retail_$tag.txt" "$d/pose.txt" 2>/dev/null
    mv "$FLLOG/pcd/all_raw_points.pcd" "$d/raw.pcd" 2>/dev/null
    mv "$FLLOG/pcd/all_downsampled_points.pcd" "$d/down.pcd" 2>/dev/null
    echo "[$tag] pose $(wc -l < "$d/pose.txt" 2>/dev/null || echo 缺) 行 | \
raw.pcd $(stat -c %s "$d/raw.pcd" 2>/dev/null || echo 缺) B | \
down.pcd $(stat -c %s "$d/down.pcd" 2>/dev/null || echo 无) B | \
dummy告警 $(grep -acE 'dummy|Dummy' "$d/node.log")"
    sleep 5
  done
done
echo "ALL RETAIL DONE"
