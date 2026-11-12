#!/usr/bin/env python3
"""统计 bag 内 lidar 帧头/帧尾 与最近图像 header 的时间差，确定 img_time_offset 到底对齐谁。"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2

BAG = "/home/ywj/slam_data/rosbag2_fastlivo2_2026_10_30-18_00_25"
LIMIT = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                        output_serialization_format="cdr"))
reader.set_filter(rosbag2_py.StorageFilter(
    topics=["/livox/lidar", "/camera/color/image_raw"]))

def ts_field(m):
    dtmap = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    names, fmts, offs = [], [], []
    for f in m.fields:
        base = dtmap[f.datatype]
        fmt = base if f.count == 1 else (base, f.count)
        names.append(f.name)
        fmts.append(fmt)
        offs.append(f.offset)
    dt = np.dtype({"names": names, "formats": fmts, "offsets": offs,
                   "itemsize": m.point_step}, align=False)
    arr = np.frombuffer(bytes(m.data), dtype=dt, count=m.width * m.height)
    return arr["timestamp"].astype(np.float64) if "timestamp" in names else None


lid = []   # (hdr, pts_min, pts_max)
img = []
t0 = None
while reader.has_next():
    topic, data, ts = reader.read_next()
    m = deserialize_message(data, PointCloud2 if topic.endswith("lidar") else Image)
    h = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if t0 is None:
        t0 = h
    if h - t0 > LIMIT:
        break
    if topic.endswith("lidar"):
        tt = ts_field(m)
        if tt.max() > 1e15:      # 绝对纳秒
            tt = tt * 1e-9
        tt = tt[tt > 1e8]
        lid.append((h, float(tt.min()), float(tt.max()), len(tt)))
    else:
        img.append((h, m.encoding, m.width, m.height, m.step))
        enc = (m.encoding, m.width, m.height, m.step)

lid = np.array([(a, b, c) for a, b, c, _ in lid])
img = np.array([a for a, *_ in img])
print("lidar frames %d  images %d  window %.1fs" % (len(lid), len(img), LIMIT))
print("img encoding/size:", enc)
print("lidar hdr period  mean %.4f s  (n=%d)" % (np.mean(np.diff(lid[:, 0])), len(lid)))
print("image hdr period  mean %.4f s  (n=%d)" % (np.mean(np.diff(img)), len(img)))
print("pt_ts_min - hdr: mean %+.5f  pt_ts_max - hdr: mean %+.5f  帧内跨度 mean %.4f"
      % (np.mean(lid[:, 1] - lid[:, 0]), np.mean(lid[:, 2] - lid[:, 0]),
         np.mean(lid[:, 2] - lid[:, 1])))
start = lid[:, 1]
end = lid[:, 2]
for name, ref in (("lidar_start", start), ("lidar_end", end)):
    idx = np.searchsorted(img, ref)
    lo = np.clip(idx - 1, 0, len(img) - 1)
    hi = np.clip(idx, 0, len(img) - 1)
    d_lo = ref - img[lo]
    d_hi = img[hi] - ref
    pick = np.where(np.abs(d_lo) <= np.abs(d_hi), d_lo, d_hi)  # ref - nearest img
    print("%s - nearest_img_header: mean %+.5f median %+.5f p5 %+.5f p95 %+.5f 帧内跨度 mean %.3f"
          % (name, pick.mean(), np.median(pick), np.percentile(pick, 5),
             np.percentile(pick, 95), np.mean(lid[:, 2] - lid[:, 1])))
