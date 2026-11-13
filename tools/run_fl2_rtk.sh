#!/usr/bin/env bash
# 官方 FAST-LIVO2 在 RTK-SLAM Dataset construction_seq2 上的整轮运行。
# 用途：为"点云偏差 vs 距离"评测提供参考位姿 + 参考稠密点云。
#
#   bash tools/run_fl2_rtk.sh              # 整轮 599 s
#   PLAY_TIMEOUT=120 bash tools/run_fl2_rtk.sh   # 冒烟
BAG=${BAG:-/home/ywj/slam_data/rtk_slam/construction_seq2_r2}
CFG=${CFG:-/home/ywj/slam_data/rtk_slam/fl2_cfg/rtk_construction_lvi.yaml}
CAM=${CAM:-/home/ywj/slam_data/rtk_slam/fl2_cfg/camera_rtk_pinhole.yaml}
OUT=${OUT:-/home/ywj/slam_data/rtk_slam/run_fl2_seq2}
TAG=${TAG:-fl2}
RATE=${RATE:-1.0}
PLAY_TIMEOUT=${PLAY_TIMEOUT:-0}
PCD_DIR=/home/ywj/fastlivo2_ws/src/fast_livo/Log/pcd

# 同一时刻只允许一个 ROS 跑批。并行会（1）让彼此的 pkill 前奏互相对杀，实测
# 曾把正在进行的整轮打死；（2）叠加 DDS 流量。锁随本进程退出自动释放。
LOCK=/tmp/ros_batch.lock
exec 9>"$LOCK"
flock -n 9 || { echo "拒绝启动：$LOCK 已被另一个 ROS 跑批持有（禁止并行）"; exit 1; }
# 第二道闸 + batch_pid/batch_kill（只杀自己登记过的 PID）。
source /home/ywj/git/Super-LIVO/tools/ros_batch_guard.sh

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash   # 提供 livox_ros_driver2
source /home/ywj/fastlivo2_ws/install/setup.bash
# 本机 FastDDS 的发现已坏（最小 pub/sub 也收不到），改用 CycloneDDS。
# 注意：Cyclone 在 WSL 下若只绑 lo（ROS_LOCALHOST_ONLY=1）会因 lo 不支持组播而
# 发现失败（日志 "selected interface lo is not multicast-capable"），故不设该变量。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 到这里上面的 guard 已经保证场上没有任何活节点，下面只是清一下"用了我这份
# params-file"的僵死残留。原先这里是全局 pkill -f fast_livo/fastlivo_mapping /
# image_transport/republish / parameter_blackboard，会顺手打死别人的跑批
# （run_ntu.sh 的 fl 分支和临时 degen 脚本都用得上这些进程名），故删掉。
pkill -f "fastlivo_mapping.*$(basename "$CFG")" 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/fast_datachannel_* 2>/dev/null

rm -rf "$OUT"; mkdir -p "$OUT"
mkdir -p "$PCD_DIR"; rm -f "$PCD_DIR"/*.pcd
rm -f "/tmp/fl2_${TAG}.done"

# 相机内参：WSL 下 SyncParametersClient 发现不到 parameter_blackboard，
# 改由 vikit 的 getRemoteParam 读环境变量（见 params_helper.h）。
source /home/ywj/slam_data/rtk_slam/fl2_cfg/fl2_env.sh

# 2) CompressedImage -> Image（bag 只有压缩图）
#    注意：image_transport 的订阅话题是 "in/compressed"，只 remap "in" 不生效
ros2 run image_transport republish compressed raw --ros-args \
  -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_raw \
  > "/tmp/fl2_repub_${TAG}.log" 2>&1 &
REPUB=$!; batch_pid $REPUB
sleep 2

# 3) FAST-LIVO2 本体
ros2 run fast_livo fastlivo_mapping --ros-args --params-file "$CFG" \
  > "/tmp/fl2_${TAG}.log" 2>&1 &
NODE=$!; batch_pid $NODE
sleep 15

# 4) 录 odom（/aft_mapped_to_init）
ros2 bag record -o "$OUT/odom" /aft_mapped_to_init > "/tmp/fl2_rec_${TAG}.log" 2>&1 &
REC=$!; batch_pid $REC
sleep 2

# 5) 播包（只播需要的 3 个话题：/livox/points 与 RTK 话题不需要，播了只会拖慢）
TOPICS=${TOPICS:-"/livox/lidar /livox/imu /camera/image_raw/compressed"}
if [ "$PLAY_TIMEOUT" = "0" ]; then
  ros2 bag play "$BAG" --rate "$RATE" --topics $TOPICS --disable-keyboard-controls < /dev/null \
    > "/tmp/fl2_play_${TAG}.log" 2>&1
else
  timeout "$PLAY_TIMEOUT" ros2 bag play "$BAG" --rate "$RATE" --topics $TOPICS --disable-keyboard-controls < /dev/null \
    > "/tmp/fl2_play_${TAG}.log" 2>&1
fi

sleep 3
kill -INT $REC 2>/dev/null; sleep 5; kill $REC 2>/dev/null
sleep 2
pkill -INT -P $NODE 2>/dev/null; kill -INT $NODE 2>/dev/null
# SIGINT 后 FAST-LIVO2 才在 run() 末尾走 savePCD()：interval>0 时只是把最后
# 不满一个 interval 的尾巴刷成一个分片；若是 interval=-1，整轮彩色点云全在内存里，
# 退出时先做 filter_size_pcd 全局降采样再写盘，千万点量级要几十秒。
# 宽限期必须显著大于这个耗时，否则下面的 KILL 会在写盘途中把它打死，
# 跑完的整轮同样颗粒无收（实测被 OOM SIGKILL 那轮就是 0 个 pcd）。
for i in $(seq 1 120); do
  kill -0 $NODE 2>/dev/null || break
  sleep 2
done
batch_kill KILL          # 只杀本脚本登记过的 NODE / REPUB / REC
ls -l "$PCD_DIR" 2>/dev/null

# 6) 把 Log/pcd 里的地图挪到本次目录
mkdir -p "$OUT/pcd"
mv "$PCD_DIR"/*.pcd "$OUT/pcd"/ 2>/dev/null
echo "pcd 分片: $(ls "$OUT/pcd"/*.pcd 2>/dev/null | wc -l) 个，合计 $(du -sh "$OUT/pcd" 2>/dev/null | cut -f1)"
echo "合并：python3 tools/fl2_pcd_merge.py --in $OUT/pcd --out $OUT/Cloud_merged.pcd --voxel 0.05"

echo "odom 消息数: $(ros2 bag info "$OUT/odom" 2>/dev/null | grep -i 'Total messages')"
echo "建图帧数: $(grep -ac 'Get image' "/tmp/fl2_${TAG}.log" 2>/dev/null)"
echo "dummy 相机告警: $(grep -c 'dummy pinhole' "/tmp/fl2_${TAG}.log" 2>/dev/null)"
echo "退化/失败: $(grep -c -i 'fail\|error' "/tmp/fl2_${TAG}.log" 2>/dev/null)"
touch "/tmp/fl2_${TAG}.done"
