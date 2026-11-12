#!/usr/bin/env bash
# bash /tmp/b2_one.sh <tag> <sigma>
TAG=$1
SIG=$2
CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b2.yaml
OUT=/home/ywj/slam_data/room_slam

bash /tmp/run_b2.sh "$TAG" "$CFG" 0 -p lio.visual.sigma_rot:="$SIG" > "/tmp/${TAG}.out" 2>&1
L=/tmp/slio_${TAG}.log
echo "########## $TAG sigma=$SIG ##########"
echo "--- 启用行:"; grep -E '视觉相对旋转已启用' "$L" | head -1
echo "--- 接受率(最后一个 pairs/ok):"; grep -oE 'pairs=[0-9]+ ok=[0-9]+' "$L" | tail -1
echo "--- applied 最大:"; grep -oE 'applied=[0-9]+' "$L" | cut -d= -f2 | sort -n | tail -1
echo "--- |e| 统计:"
grep -oE '\|e\|=[0-9.]+mrad' "$L" | cut -d= -f2 | sed 's/mrad//' \
  | awk '{n++;s+=$1;if($1>m)m=$1}END{printf "  n=%d mean=%.2f max=%.2f mrad\n",n,s/n,m}'
echo "--- 最后 2 条 none 行(含 e_mean):"; grep -E '\[VisRot\] none' "$L" | tail -2
echo "--- 最后 2 条注入行:"; grep -E '\[VisRot\] inl=' "$L" | tail -2
echo "--- 错误:"; grep -iE 'error|what\(\)|Segmentation|Exception|FATAL' "$L" | head -5
echo "--- 注入总次数(行):"; grep -cE '\[VisRot\] inl=' "$L"
python3 /tmp/eval_loop.py "$OUT/run_$TAG" "$TAG(s=$SIG)"
