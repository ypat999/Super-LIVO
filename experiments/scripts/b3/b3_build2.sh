#!/bin/bash
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO || exit 9
source /opt/ros/humble/setup.bash 2>/dev/null
colcon build --packages-select super_lio > /tmp/b3_build2.log 2>&1
rc=$?
echo "BUILD_RC=$rc"
grep -aiE 'error|warning' /tmp/b3_build2.log | head -25
echo "----- tail"
sed -n '$p' /tmp/b3_build2.log
ls -la install/super_lio/lib/super_lio/super_lio_node
echo BUILD2-DONE
