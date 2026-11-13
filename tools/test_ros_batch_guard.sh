#!/usr/bin/env bash
# 校验 tools/ros_batch_guard.sh 的"活节点"识别正则。
# pattern 用 eval 直接吃 guard 里的那几行 _LIVEPAT=，保证测的是线上那份、不会两处漂移。
# 正例 = 真在跑批的进程 cmdline；反例 = 命令行里顺便写了这些词的 shell / ros2 包装器。
eval "$(grep '^_LIVEPAT=' /home/ywj/git/Super-LIVO/tools/ros_batch_guard.sh)"
PAT=$_LIVEPAT
echo "pattern: $PAT"
echo

POS=' /home/ywj/fastlivo2_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping --ros-args --params-file /home/ywj/slam_data/rtk_slam/fl2_cfg/rtk_construction_lvi.yaml
 /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/super_lio/lib/super_lio/super_lio_node --ros-args --params-file /home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/NTU.yaml
 /home/ywj/git/Super-LIVO/install/super_livo_visual_b/lib/super_livo_visual_b/visual_b_node --ros-args --params-file x.yaml
 /home/ywj/git/Super-LIVO/install/super_livo_visual_a/lib/super_livo_visual_a/visual_a_node --ros-args
 python3 /tmp/cloud_crop.py
 /usr/bin/python3 /opt/ros/humble/bin/ros2 bag play /home/ywj/slam_data/ntu_viral/bags2/eee_03 --rate 0.5
 /usr/bin/python3 /opt/ros/humble/bin/ros2 bag record -o /home/ywj/slam_data/rtk_slam/run_fl2_seq2/odom /aft_mapped_to_init
 /opt/ros/humble/lib/image_transport/republish compressed -s in:=/camera/image_raw/compressed -r out:=/camera/image_raw'

NEG=' bash -lc cd /home/ywj/git/Super-LIVO; RATE=0.3 TAG=seq2b setsid nohup bash tools/run_fl2_rtk.sh >/tmp/fl2_seq2b_run.log 2>&1 </dev/null & sleep 20; pgrep -a -f ros2 bag play
 bash -lc pgrep -a -f fastlivo_mapping; pgrep -a -f super_lio_node; pgrep -a -f visual_b_node
 bash tools/run_fl2_rtk.sh
 bash /tmp/run_ntu_degen.sh b
 /usr/bin/python3 /opt/ros/humble/bin/ros2 run super_lio super_lio_node --ros-args --params-file x.yaml
 /usr/bin/python3 /opt/ros/humble/bin/ros2 launch super_lio super_lio.launch.py
 tail -f /tmp/slio_eee_03_degen_dm3novg.log'

fail=0
while IFS= read -r l; do
  l=${l# }                      # 样例每行前面那个空格只是排版用的，必须去掉，
  [[ -z $l ]] && continue       # 否则 ^ 锚点必然失配（第一版就是把正例全部判成漏判）
  if [[ $l =~ $PAT ]]; then
    echo "OK 命中:${l:0:70}"
  else
    echo "漏判(应命中):${l:0:70}"; fail=1
  fi
done <<< "$POS"
echo
while IFS= read -r l; do
  l=${l# }
  [[ -z $l ]] && continue
  if [[ $l =~ $PAT ]]; then
    echo "误判(不应命中):${l:0:70}"; fail=1
  else
    echo "OK 不命中:${l:0:70}"
  fi
done <<< "$NEG"
echo
if [ $fail -eq 0 ]; then echo "PASS"; else echo FAIL; fi
