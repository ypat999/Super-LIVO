#!/bin/bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri="/home/ywj/slam_data/singleframe/fl2_frames",
                                 storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
clouds, odom = [], []
while r.has_next():
    topic, payload, _t = r.read_next()
    if not isinstance(payload, (bytes, bytearray)):
        continue
    if "cloud_registered" in topic:
        m = deserialize_message(payload, PointCloud2)
        n = m.width * m.height
        if n == 0:
            continue
        buf = np.frombuffer(m.data, np.uint8).reshape(n, m.point_step)
        fo = {f.name: f.offset for f in m.fields}
        P = np.frombuffer(buf[:, fo["x"]:fo["x"] + 12].tobytes(), np.float32).reshape(n, 3)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        clouds.append((t, P))
    elif "odom" in topic or "mapped" in topic:
        m = deserialize_message(payload, Odometry)
        p = m.pose.pose.position
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        odom.append((t, np.array([p.x, p.y, p.z])))
odom.sort()
print(f"clouds={len(clouds)} odom={len(odom)}")
op = np.array([o[1] for o in odom])
print("odom xyz 范围:", op.min(0).round(2), op.max(0).round(2))
c0, c1 = clouds[5][1], clouds[len(clouds) - 5][1]
print("cloud5 centroid:", c0.mean(0).round(2), " cloud-5 centroid:", c1.mean(0).round(2))
print("cloud5 x med:", np.median(c0[:, 0]).round(2), " cloud-5 x med:", np.median(c1[:, 0]).round(2))
# 世界系判据：两帧相距 >10s 的点云，若质心差 > 数米 => 各自体坐标系；相近 => 世界系
dt = clouds[len(clouds) - 5][0] - clouds[5][0]
print(f"时间差 {dt:.1f}s，质心差 {np.linalg.norm(c1.mean(0)-c0.mean(0)):.2f} m")
# 重叠判据（世界系则静态场景大量重叠）
from scipy.spatial import cKDTree
sub = c0[::7]
d, _ = cKDTree(c1[::7]).query(sub, k=1)
print(f"kNN(帧5→末帧) 中位 {np.median(d):.3f} m  p10 {np.percentile(d,10):.3f}")

# 场景是否 = ntu eee_03：找 GT
import glob, os
for pat in ["/home/ywj/slam_data/dataset_raw/**/ground_truth*",
            "/home/ywj/slam_data/ntu_viral/**/*gt*",
            "/home/ywj/slam_data/**/positionGroundTruth*"]:
    for p in glob.glob(pat, recursive=True)[:4]:
        print("GT candidate:", p, os.path.getsize(p))
PY
