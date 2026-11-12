#!/bin/bash
echo "== /tmp/eval_*.txt 汇总"
for f in /tmp/eval_room_off.txt /tmp/eval_b3off.txt /tmp/eval_b3off2.txt /tmp/eval_b2full.txt /tmp/eval_bd05s005.txt /tmp/eval_bd10s005.txt /tmp/eval_bd20s005.txt /tmp/eval_bd10s002.txt /tmp/eval_bd10s010.txt; do
  [ -f "$f" ] && { echo "--- $f"; sed -n '1,6p' "$f"; }
done
echo "== 现跑 eval_loop（若无文件）"
for t in room_off b3off2 b2full bd05s005 bd10s005 bd20s005 bd10s002 bd10s010; do
  d=/home/ywj/slam_data/room_slam/run_$t
  [ -d "$d" ] || continue
  printf "%-10s " "$t"
  python3 /tmp/eval_loop.py "$d" "$t" 2>/dev/null | sed -n '3,4p' | tr '\n' ' '
  echo
done
echo "== 内层仓库改动（third_party/Super-LIO）"
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO && git status --short | grep -aE 'src/super_lio' | head -30
git log --oneline -1
echo "ROOMTAB-DONE"
