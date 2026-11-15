#!/usr/bin/env bash
# 跑官方 FAST-LIVO2 并录制「染色阶梯」所需的统一输入：
#   /cloud_registered_body   LiDAR 系去畸变全分辨率扫描（feats_undistort）
#   /vio/odom_lidar          VIO 光度修正后的同刻 LiDAR 位姿（本轮在 handleVIO 新增）
#   /aft_mapped_lidar_to_init LIO 位姿（官方染色的 p_w 由它得到，见 handleLIO）
#   /cloud_registered        官方自带彩色世界云（R5 一致性校验的参考）
#
# 注意：本机 WSL 当前 127.0.0.1 的跨进程 UDP 已被破坏（多播/单播均不通），
# 只有经 eth0（本机 192.168.1.x）的 UDP 正常。因此这里生成一个只走 eth0 单播
# 的 FastDDS profile，取代 ROS_LOCALHOST_ONLY=1 的默认 setup。
set -o pipefail

OUT=/home/ywj/slam_data/official_vio_20261114
CFG=/home/ywj/slam_data/fastlivo2_ref_20261110/avia_wrapped.yaml
BAG=/home/ywj/slam_data/Retail_Street
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/fastlivo2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS RMW_IMPLEMENTATION CYCLONEDDS_URI

# 取默认路由网卡的 IPv4（WSL 里 127.0.0.1 跨进程不通，只能走这张网卡）
IP=$(ip -4 route get 192.168.1.1 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -1)
if [ -z "$IP" ]; then IP=192.168.1.5; fi
echo "[rec] using eth ip: $IP"
PROFILE=/tmp/fastdds_udp_unicast.xml
cat > "$PROFILE" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_eth</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>$IP</address></interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="dds_eth" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports><transport_id>udp_eth</transport_id></userTransports>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator><udpv4><address>$IP</address></udpv4></locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator><udpv4><address>$IP</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
XML
export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"

QOS=/tmp/rec_official_vio_qos.yaml
cat > "$QOS" <<'YAML'
/vio/odom_lidar:
  reliability: reliable
  history: keep_last
  depth: 10000
/aft_mapped_lidar_to_init:
  reliability: reliable
  history: keep_last
  depth: 10000
/cloud_registered_body:
  reliability: reliable
  history: keep_last
  depth: 3000
/cloud_registered:
  reliability: reliable
  history: keep_last
  depth: 3000
YAML

pkill -9 -f 'ros2-daemon' >/dev/null 2>&1
pkill -f fastlivo_mapping >/dev/null 2>&1
sleep 2
rm -rf "$OUT/bag" "$OUT/node.log" "$OUT/rec.log" "$OUT/play.log"

stdbuf -oL -eL ros2 run fast_livo fastlivo_mapping --ros-args --params-file "$CFG" > "$OUT/node.log" 2>&1 &
sleep 8
if ! pgrep -f fastlivo_mapping >/dev/null; then
  echo "[rec] node 启动失败"; tail -40 "$OUT/node.log"; exit 1
fi

ros2 bag record -o "$OUT/bag" --qos-profile-overrides-path "$QOS" \
    /cloud_registered_body /vio/odom_lidar \
    /aft_mapped_lidar_to_init /cloud_registered > "$OUT/rec.log" 2>&1 &
recpid=$!
sleep 3

ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls > "$OUT/play.log" 2>&1
sleep 3
kill -INT "$recpid" 2>/dev/null
sleep 5
pkill -f fastlivo_mapping >/dev/null 2>&1
sleep 2

ros2 bag info "$OUT/bag" 2>/dev/null | sed -n '1,30p'
echo "[rec] done -> $OUT"
