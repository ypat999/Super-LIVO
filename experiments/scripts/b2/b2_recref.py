#!/usr/bin/env python3
"""recorded(=bag playback) 时间戳 与 header 时间戳的差：判断回放时图像是否在 lidar 帧尾之前到达。"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Image

BAG = "/home/ywj/slam_data/rosbag2_fastlivo2_2026_10_30-18_00_25"
LIMIT = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
            rosb2_py_opts := rosbag2_py.ConverterOptions("cdr", "cdr"))
reader.set_filter(rosbag2_py.StorageFilter(topics=["/livox/lidar", "/camera/color/image_raw"]))
first_t = None
d = {"/livox/lidar": [], "/camera/color/image_raw": []}
hdr = {"/livox/lidar": [], "/camera/color/image_raw": []}
while reader.has_next():
    topic, data, ts = reader.read_next()
    h = ts * 1e-9
    if first_t is None:
        first_t = h
    if h - first_t > LIMIT:
        break
    m = deserialize_message(data, PointCloud2 if topic.endswith("lidar") else Image)
    hh = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    d[topic].append(h - hh)
    hdr[topic].append(hh)
for k, v in d.items():
    v = np.array(v)
    print("%-26s n=%4d  recorded-header: mean %+.4f s  std %.4f  p5 %+.4f p95 %+.4f"
          % (k, len(v), v.mean(), v.std(), np.percentile(v, 5), np.percentile(v, 95)))
a = np.array(hdr["/livox/lidar"]); b = np.array(hdr["/camera/color/image_raw"])
print("first lidar hdr %.6f  first img hdr %.6f  diff %+.6f" % (a[0], b[0], a[0] - b[0]))
