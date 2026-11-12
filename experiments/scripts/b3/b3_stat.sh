#!/bin/bash
date
echo "=== FD lines in smoke log ==="
grep -c 'VisB3-FD' /tmp/slio_b3smoke.log || true
grep -n 'VisB3-FD' /tmp/slio_b3smoke.log | head -20
echo "=== tail of b2full log ==="
tail -5 /tmp/slio_b2full.log
echo "=== run dirs (find) ==="
ls -lat /home/ywj/slam_data/ 2>/dev/null | head -25
echo "=== build artifacts ==="
ls -la /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/super_lio/lib/super_lio/ 2>/dev/null | head
find /home/ywj/git/Super-LIVO/third_party/Super-LIO/build/super_lio -name 'visual_eskf_inject.cpp.o' -printf '%T@ %TY-%Tm-%Td %TH:%TM %p\n' 2>/dev/null
echo "=== source mtime ==="
stat -c '%y %n' /home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/visual/visual_eskf_inject.cpp
