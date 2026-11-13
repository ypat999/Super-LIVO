#!/bin/bash
set -u
cd /home/ywj/git/Super-LIVO || exit 1
echo "=== any file containing g_visual_sigma_rot (all types, excl .git) ==="
grep -rl "g_visual_sigma_rot" . --exclude-dir=.git 2>/dev/null | head -20
echo "=== sigma_rot anywhere in src ==="
grep -rn "sigma_rot" third_party/Super-LIO/src --include=*.h --include=*.hpp --include=*.cpp | head -20
echo "=== top-level build/install ==="
ls -d build install log 2>/dev/null
ls -la build/super_lio/CMakeFiles/*.dir/src/visual 2>/dev/null | head -10
find . -maxdepth 4 -name "libsuper_lio*.so" -o -maxdepth 4 -name "super_lio_node" 2>/dev/null | head
echo "=== binary mtimes ==="
ls -la third_party/Super-LIO/install/super_lio/lib/super_lio/* 2>/dev/null | head
