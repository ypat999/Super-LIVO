#!/bin/bash
# 每个 tag 压成一行：终点位置/终点yaw|回环位置/回环yaw|acc%|rej|chi2max|dp_max|dyaw_max|dxr_max|dpjump|n20cm
for TAG in "$@"; do
  L=/tmp/slio_$TAG.log
  D=/home/ywj/slam_data/room_slam/run_$TAG
  [ -d "$D" ] || D=/home/ywj/slam_data/ntu_viral/run_$TAG
  E=""
  [ -d "$D" ] && E=$(python3 /tmp/eval_loop.py "$D" "$TAG" 2>/dev/null | sed -n '2,4p' | tr '\n' ' ')
  P=$(grep -a '\[Param\] visual/b2' "$L" 2>/dev/null | head -1 | sed 's/.*enable=//')
  G=$(grep -a '\[VisB2gate\]' "$L" 2>/dev/null | tail -1)
  A=$(echo "$G" | grep -oa 'acc=[0-9]*/[0-9]*([0-9.]*%)' | tail -1)
  C=$(echo "$G" | grep -oa 'chi2_max=[0-9eE+-]*' | tail -1)
  R=$(echo "$G" | grep -oa 'rej=[0-9]*/[0-9]*' | tail -1)
  DM=$(echo "$G" | grep -oa 'dp_max=[0-9.eEnaNaid]*' | tail -1)
  DY=$(echo "$G" | grep -oa 'dyaw_max=[0-9.eE+-]*' | tail -1)
  DX=$(echo "$G" | grep -oa 'max_dx_r=[0-9.eE+-]*' | tail -1)
  T=$(grep -a '\[VisB3\] t=' "$L" 2>/dev/null | tail -1 | grep -oa 'acc=[0-9]*/[0-9]*([0-9.]*%)' )
  N=$(grep -a 'VisB2. reject chi2' "$L" 2>/dev/null | wc -l)
  CO=""
  [ -d "$D" ] && CO=$(python3 /tmp/p1_cont.py "$D" "$TAG" 2>/dev/null | sed 's/.*beat=/beat=/;s/nan=[0-9]* //;s/tspan=/tsp=/' | tr '\n' ' ')
  echo "ROW $TAG | $P | $E | b2acc=$A $R $C $DM $DY $DX | b3acc=$T | rejlog=$N | $CO"
done
echo ROW-DONE
