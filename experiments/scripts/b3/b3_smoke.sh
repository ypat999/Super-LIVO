#!/bin/bash
# 单独 smoke 跑批（前台，持锁）
exec 9>/tmp/ros_batch.lock
flock 9
source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=77
bash /tmp/run_b2.sh b3smoke /home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b3.yaml 40
echo "===== VisB3bg (FD 校核行) ====="
grep -a "VisB3bg" /tmp/slio_b3smoke.log | head -15
echo "===== VisB3 汇总行 ====="
grep -a "\[VisB3\]" /tmp/slio_b3smoke.log | tail -6
echo "===== 异常 ====="
grep -aiE 'FD-mismatch|diff|WARNING|ERROR|FATAL' /tmp/slio_b3smoke.log | grep -av 'Logging before' | head -10
echo SMOKE-DONE
