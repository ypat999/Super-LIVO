#!/bin/bash
set -u
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO || exit 1
echo "=== g_visual* symbols in liblio.so (defined) ==="
nm -C --defined-only build/super_lio/liblio.so 2>/dev/null | grep -o 'g_visual[a-z0-9_]*' | sort -u
echo "=== mtimes ==="
ls -la --time-style=long-iso src/super_lio/src/lio/params.cpp src/super_lio/include/lio/params.h src/super_lio/src/ros/ROSWrapper.cpp src/super_lio/src/visual/visual_eskf_inject.cpp
ls -la --time-style=long-iso build/super_lio/CMakeFiles/lio.dir/src/lio/params.cpp.o build/super_lio/CMakeFiles/lio.dir/src/ros/ROSWrapper.cpp.o build/super_lio/CMakeFiles/lio.dir/src/visual/visual_eskf_inject.cpp.o
echo "=== backups of params in /tmp or exp ==="
find /tmp /home/ywj/git/Super-LIVO/exp /home/ywj/tmp -name '*params*' -newermt '2026-11-11' 2>/dev/null | head -20
echo "=== git reflog (submodule) ==="
git reflog -8 2>/dev/null | head -10
