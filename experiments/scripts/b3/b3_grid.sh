#!/bin/bash
# room 网格：bash /tmp/b3_grid.sh <name>   （单条前台调用，自带清场与摘要）
OUT=/tmp/b3_out_$1.txt
case "$1" in
  b2full)   TAG=b2full   EXTRA=(-p lio.visual.b3.enable:=false) ;;
  bd05s005) TAG=bd05s005 EXTRA=(-p lio.visual.b3.baseline_dt:=0.5 -p lio.visual.b3.sigma_rot:=0.05) ;;
  bd10s005) TAG=bd10s005 EXTRA=(-p lio.visual.b3.baseline_dt:=1.0 -p lio.visual.b3.sigma_rot:=0.05) ;;
  bd20s005) TAG=bd20s005 EXTRA=(-p lio.visual.b3.baseline_dt:=2.0 -p lio.visual.b3.sigma_rot:=0.05) ;;
  bd10s002) TAG=bd10s002 EXTRA=(-p lio.visual.b3.baseline_dt:=1.0 -p lio.visual.b3.sigma_rot:=0.02) ;;
  bd10s010) TAG=bd10s010 EXTRA=(-p lio.visual.b3.baseline_dt:=1.0 -p lio.visual.b3.sigma_rot:=0.10) ;;
  *) echo "unknown $1"; exit 3 ;;
esac
shift
bash /tmp/b3_one.sh "$TAG" 0 "${EXTRA[@]}" > "$OUT" 2>&1
echo "GRID-EXIT=$? tag=$TAG out=$OUT"
sed -n '/### EVAL/,/### VisB3 tail/p' "$OUT" | head -6
grep -a '\[VisB3\] t=' "$OUT" | tail -1
grep -a 'VisB3bg' "$OUT" | head -1
grep -a 'VisB3bg' "$OUT" | tail -1
