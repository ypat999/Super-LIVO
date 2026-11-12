#!/bin/bash
bash /tmp/b3_evalall.sh room_off b3off b2full bd05s005 bd10s005 bd20s005 bd10s002 bd10s010
echo
echo "=== 诊断（末条 VisB3）==="
for t in bd05s005 bd10s005 bd20s005 bd10s002 bd10s010; do
  L=$(grep -a '\[VisB3\] t=' /tmp/slio_$t.log 2>/dev/null | tail -1)
  echo "$t: $(echo "$L" | grep -oP 'acc=\S+ sigma_eq_bg=\S+ .*? no_phase=\S+ gap=\S+ unordered=\S+')"
  echo "     $(echo "$L" | grep -oP 'pairs=\S+ ok=\S+ mean_inl=\S+')"
  echo "     reject行数=$(grep -ac 'reject chi2' /tmp/slio_$t.log 2>/dev/null)  注入次数=$(grep -ac '\[VisB3bg\]' /tmp/slio_$t.log 2>/dev/null)"
done
echo
echo "=== bg_z 时间序列（每 40 条采 1，单位 mrad/s）==="
for t in bd10s005 bd10s002; do
  echo "-- $t"
  grep -a '\[VisB3bg\]' /tmp/slio_$t.log | grep -oP 'bgz=\S+' | awk 'NR%40==1{print NR": "$0}'
done
echo
echo "=== 异常检查 ==="
for t in b2full bd05s005 bd10s005 bd20s005 bd10s002 bd10s010; do
  echo -n "$t: "
  grep -aicE 'FD-mismatch|Segmentation|FATAL' /tmp/slio_$t.log 2>/dev/null | tr '\n' ' '
  grep -a '\[VisRot\] inl=' /tmp/slio_$t.log | tail -1 | grep -oP 'applied=\S+'
done
