#!/bin/bash
# NTU 各档诊断汇总
for t in off b2 bd10 bd05 bd20; do
  L=/tmp/slio_ntu_$t.log
  echo "===== TAG=$t lines=$(wc -l < $L) ====="
  echo "-- VisRot/VisB2 末行"
  grep -a 'VisRot] ok\|VisB2\|applied=' "$L" | tail -2
  echo "-- VisB3 末行"
  grep -a '\[VisB3\] t=' "$L" | tail -2
  echo "-- VisB3-FD"
  grep -a '\[VisB3-FD' "$L" | head -12
  echo "-- reject 行数"
  grep -ac 'reject chi2' "$L"
  echo "-- 异常"
  grep -aiE 'fd.?mismatch|nan|segfault|assert|FATAL|E[0-9]{4} ' "$L" | head -5
  echo "-- bgz 序列(每20取1)"
  grep -a '\[VisB3bg\]' "$L" | awk 'NR%20==1{print $3,$4,$5,$6,$7}'
  echo "-- bgz 首尾"
  grep -a '\[VisB3bg\]' "$L" | head -2
  grep -a '\[VisB3bg\]' "$L" | tail -2
done
echo "=====NTU-DIAG-DONE"
