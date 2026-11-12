#!/bin/bash
# NTU 各档最终统计：以 [VisB3bg] 行数作为注入数，末行 [VisB3] 作为累计计数
for t in bd05 bd10 bd20 bd10s002 bd10s005; do
  L=/tmp/slio_ntu_$t.log
  n=$(grep -ac '\[VisB3bg\]' "$L")
  rj=$(grep -ac 'reject chi2' "$L")
  last=$(grep -a '\[VisB3\] t=' "$L" | tail -1)
  echo "TAG=$t 注入行=$n reject行=$rj"
  echo "  last: $(echo "$last" | sed -E 's/.*\] \[VisB3\] //')"
  echo "  dxbg_z 绝对值分布:"
  grep -a '\[VisB3bg\]' "$L" | sed -E 's/.*dxbg_z=([-0-9.e]+).*/\1/' | awk '{v=$1<0?-$1:$1; s+=v; n++; if(v>mx)mx=v} END{printf "    mean=%.4f max=%.4f n=%d mrad/s\n", s/n, mx, n}'
done
echo "SUM-DONE"
