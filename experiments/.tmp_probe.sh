#!/bin/bash
set -u
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO || exit 1

echo "=== visual dir ==="
ls -la src/super_lio/src/visual/ src/super_lio/include/visual/ 2>&1

echo "=== grep g_visual_enable ==="
grep -rn "g_visual_enable" src/ --include=*.h --include=*.hpp --include=*.cpp | grep -v "src/visual/visual_eskf_inject.cpp" | head -20

echo "=== grep declaration of b2 ==="
grep -rn "g_visual_b2_enable" src/ | head -10

echo "=== CRLF census (tracked modified files) ==="
n_lf2crlf=0; n_other=0
for f in $(git diff --name-only); do
  [ -f "$f" ] || continue
  h=$(git show "HEAD:$f" 2>/dev/null | grep -c $'\r$')
  w=$(grep -c $'\r$' "$f")
  if [ "$h" -eq 0 ] && [ "$w" -gt 0 ]; then n_lf2crlf=$((n_lf2crlf+1)); fi
  if [ "$h" -gt 0 ] && [ "$w" -eq 0 ]; then n_other=$((n_other+1)); fi
done
echo "HEAD_LF_worktree_CRLF=$n_lf2crlf  HEAD_CRLF_worktree_LF=$n_other"

echo "=== top-level README ==="
cd /home/ywj/git/Super-LIVO
h=$(git show HEAD:README.md | grep -c $'\r$'); w=$(grep -c $'\r$' README.md)
echo "README HEAD_CR=$h work_CR=$w"
