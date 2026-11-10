#!/bin/bash
source /opt/ros/humble/setup.bash
for b in /home/ywj/slam_data/singleframe/sl_frames /home/ywj/slam_data/e1_append/e1_frames /home/ywj/slam_data/singleframe/fl2_frames; do
  echo "=== $b"
  BAGPATH="$b" python3 - <<'PY'
import os, rosbag2_py
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=os.environ["BAGPATH"], storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
m = r.get_metadata()
for t in m.topics_with_message_count:
    print("  ", t.topic_metadata.name, t.message_count, t.topic_metadata.type)
PY
done
