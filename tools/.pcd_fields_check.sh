#!/bin/bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

def peek(uri, topic_key):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    got = 0
    while r.has_next() and got < 2:
        topic, payload, _t = r.read_next()
        if topic_key not in topic or not isinstance(payload, (bytes, bytearray)):
            continue
        m = deserialize_message(payload, PointCloud2)
        n = m.width * m.height
        print(" ", topic, "n=", n, "step=", m.point_step,
              "fields=", [(f.name, f.count, f.datatype) for f in m.fields])
        got += 1

print("== sl")
peek("/home/ywj/slam_data/singleframe/sl_frames", "colored_cloud")
print("== e1")
peek("/home/ywj/slam_data/e1_append/e1_frames", "colored_cloud")
print("== fl2")
peek("/home/ywj/slam_data/singleframe/fl2_frames", "cloud_registered")

import numpy as np
def pcd_head(p):
    with open(p, "rb") as f:
        head = b""
        while True:
            line = f.readline()
            head += line
            if line.lower().strip().startswith(b"data"):
                break
    print(p.split("/")[-1])
    for ln in head.decode("ascii", "ignore").splitlines():
        if any(ln.lower().startswith(k) for k in ("fields", "size", "count", "points")):
            print("   ", ln.strip())

for p in ["/home/ywj/slam_data/e1_append/e1_map/super_livo_visual_a_colored.pcd",
          "/home/ywj/slam_data/singleframe/fl2_frames_out/all_raw_points.pcd"]:
    try:
        pcd_head(p)
    except Exception as e:
        print("  miss:", p, e)
import glob
print("fl2 pcd candidates:", glob.glob("/home/ywj/slam_data/**/all_raw_points.pcd", recursive=True)[:5],
      glob.glob("/home/ywj/slam_data/**/*raw*.pcd", recursive=True)[:5])
PY
