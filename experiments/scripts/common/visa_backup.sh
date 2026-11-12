#!/bin/bash
set -x
R=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio
SLIO=/home/ywj/git/Super-LIVO/third_party/Super-LIO
TS=$(date +%Y%m%d_%H%M)
OUT=/tmp/visa_removal_backup_${TS}.tgz

tar czf "$OUT" \
  -C "$R" \
  src/visual \
  include/visual \
  config/livox_360_visa.yaml \
  config/livox_360_visa_force.yaml \
  config/livox_360_visa_off.yaml \
  config/livox_360_visa_room.yaml \
  config/livox_360_visa_room_force.yaml \
  config/livox_360_visa_room_force2.yaml \
  config/livox_360_visa_room_latch.yaml \
  config/livox_360_visa_room_off.yaml
echo "tar1 rc=$?"

tar czf /tmp/visa_removal_backup_sa.tgz -C /home/ywj/git/Super-LIVO solution_a_lightweight
echo "tar2 rc=$?"

git -C "$SLIO" diff -- \
  src/super_lio/src/lio/super_lio.cpp \
  src/super_lio/include/lio/super_lio.h \
  src/super_lio/src/lio/params.cpp \
  src/super_lio/include/lio/params.h \
  src/super_lio/src/ros/ROSWrapper.cpp \
  src/super_lio/include/ros/ROSWrapper.h \
  src/super_lio/CMakeLists.txt > /tmp/visa_seams.patch
echo "git diff rc=$?"

# also save full pristine copies of seam files for safety
mkdir -p /tmp/visa_seam_files
cp "$SLIO"/src/super_lio/src/lio/super_lio.cpp /tmp/visa_seam_files/
cp "$SLIO"/src/super_lio/include/lio/super_lio.h /tmp/visa_seam_files/
cp "$SLIO"/src/super_lio/src/lio/params.cpp /tmp/visa_seam_files/
cp "$SLIO"/src/super_lio/include/lio/params.h /tmp/visa_seam_files/
cp "$SLIO"/src/super_lio/src/ros/ROSWrapper.cpp /tmp/visa_seam_files/
cp "$SLIO"/src/super_lio/include/ros/ROSWrapper.h /tmp/visa_seam_files/
cp "$SLIO"/src/super_lio/CMakeLists.txt /tmp/visa_seam_files/
ls -l /tmp/visa_removal_backup_${TS}.tgz /tmp/visa_removal_backup_sa.tgz /tmp/visa_seams.patch
ls -l /tmp/visa_seam_files
