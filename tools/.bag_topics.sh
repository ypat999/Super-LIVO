#!/bin/bash
source /opt/ros/humble/setup.bash
rosbag2_py info /home/ywj/slam_data/singleframe/sl_frames 2>/dev/null || python3 - <<'PY'
import rosbag2_py
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri="/home/ywj/slam_data/singleframe/sl_frames", storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
info = r.get_metadata()
for t in info.topics_with_message_count:
    print(t.topic_metadata.name, t.message_count)
PY
