#!/bin/bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri="/home/ywj/slam_data/singleframe/fl2_frames",
                                 storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
got = 0
while r.has_next() and got < 3:
    topic, payload, _t = r.read_next()
    if "cloud_registered" not in topic or not isinstance(payload, (bytes, bytearray)):
        continue
    m = deserialize_message(payload, PointCloud2)
    n = m.width * m.height
    if n == 0:
        continue
    buf = np.frombuffer(m.data, np.uint8).reshape(n, m.point_step)
    fo = {f.name: f.offset for f in m.fields}
    P = np.frombuffer(buf[:, fo["x"]:fo["x"] + 12].tobytes(), np.float32).reshape(n, 3)
    rgb_f = np.frombuffer(buf[:, fo["rgb"]:fo["rgb"] + 4].tobytes(), np.float32)
    rgb_i = rgb_f.astype(np.float64).view(np.uint32) if False else None
    # FAST-LIVO2 常把 rgb 存成 float32 承载的 uint32 位模式
    bits = np.frombuffer(buf[:, fo["rgb"]:fo["rgb"] + 4].tobytes(), np.uint8
                         ).copy().view(np.uint32)
    lum = 0.299 * ((bits >> 16) & 0xFF) + 0.587 * ((bits >> 8) & 0xFF) + 0.114 * (bits & 0xFF)
    print(f"frame n={n} x范围=({P[:,0].min():.1f},{P[:,0].max():.1f}) "
          f"rgb_float(finite {np.isfinite(rgb_f).mean()*100:.0f}%) "
          f"bits: 非零 {(bits != 0).mean()*100:.1f}% lum均值 {lum.mean():.1f} "
          f"lum std {lum.std():.1f} 分位 {np.percentile(lum,[5,50,95]).round(0)}")
    got += 1

import struct
with open("/home/ywj/slam_data/pcd/fl2_bag_lio/all_raw_points.pcd", "rb") as f:
    head = b""
    while True:
        line = f.readline()
        head += line
        if line.lower().strip().startswith(b"data"):
            break
print(head.decode("ascii", "ignore"))
PY
