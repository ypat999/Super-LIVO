#!/usr/bin/env bash
L=/tmp/slio_room_b2.log
echo "--- |e| 统计（来自抽样注入行）:"
grep -oE '\|e\|=[0-9.]+mrad' "$L" | cut -d= -f2 | sed 's/mrad//' \
  | awk '{n++;s+=$1;if($1>m)m=$1}END{printf "lines=%d mean=%.2f max=%.2f mrad\n", n, s/n, m}'
echo "--- none 原因分布:"
grep -oE 'none\([a-z-]+\)' "$L" | sort | uniq -c
echo "--- 最后一条状态行:"
grep -E '\[VisRot\] none' "$L" | tail -1
echo "--- applied 最大值:"
grep -oE 'applied=[0-9]+' "$L" | cut -d= -f2 | sort -n | tail -1
echo "--- reason 分布（最后一条 none 内）:"
grep -oE 'reason=[a-z0-9-]+' "$L" | sort | uniq -c
echo "--- 图像对成功率:"
grep -oE 'pairs=[0-9]+ ok=[0-9]+' "$L" | tail -1
