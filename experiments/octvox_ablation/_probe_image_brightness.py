#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中段误差归因（第四步）：直接看原始图像流本身是否在中段发生变化。

前一步（1/z 检验）结论：段内 MAE(0-3m)/MAE(10-15m) ≈ 1.0（0.93~1.15），
即误差不随 1/z 放大 → **不是帧时间对齐残差（Δt 型）**；
但中段（段1-4）在各距离上整体抬高，且 >30 的坏点占比从 0.6% 涨到 11-14%（重尾）
→ 指向"中段有一批帧的着色整体变差"，而不是几何/时间的小偏差。

这里查两个最可能的直接原因：
  1) 曝光/亮度漂移（自动曝光）：逐帧均值亮度按段统计；
  2) 运动模糊/失焦：逐帧 Laplacian 方差按段统计（越小越模糊）。

用法: python3 _probe_image_brightness.py
"""
import numpy as np
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = "/home/ywj/slam_data/Retail_Street"
TOPIC = "/left_camera/image"
SEGS = 6


def main():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    typ = get_message("sensor_msgs/msg/Image")
    ts, mean, lap = [], [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, typ)
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        buf = np.frombuffer(m.data, np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if img is None:
            if "bgr" in m.encoding or "rgb" in m.encoding:
                raw = buf.reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
                img = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
            else:
                img = buf.reshape(m.height, m.step)[:, :m.width]
        mean.append(float(img.mean()))
        lap.append(float(cv2.Laplacian(img, cv2.CV_64F).var()))
    ts = np.array(ts)
    mean = np.array(mean)
    lap = np.array(lap)
    t0 = ts[0]
    t = ts - t0
    dt = np.diff(ts)
    print("图像 %d 帧，span %.1fs，dt p50=%.3fs max=%.3fs，>0.2s 间隔 %d 处" % (
        len(ts), t[-1], np.median(dt), dt.max(), int((dt > 0.2).sum())))
    bad = np.where(dt > 0.2)[0]
    for i in bad[:10]:
        print("  间隔 @t=%.1fs dt=%.3fs" % (t[i], dt[i]))

    edges = np.linspace(t[0], t[-1], SEGS + 1)
    print("\n段   t起(s)  亮度mean  亮度std  lap_var(p50)  lap_var(p10)  帧数")
    for s in range(SEGS):
        m = (t >= edges[s]) & (t < edges[s + 1]) if s < SEGS - 1 else (t >= edges[s])
        print("%d  %8.1f  %8.1f  %7.1f  %11.0f  %11.0f  %5d" % (
            s, edges[s], mean[m].mean(), mean[m].std(),
            np.percentile(lap[m], 50), np.percentile(lap[m], 10), int(m.sum())))

    print("\n亮度逐帧轨迹（每 5 帧取 1，t/mean/lap）：")
    for i in range(0, len(t), 5):
        print("  %7.1f  %6.1f  %9.0f" % (t[i], mean[i], lap[i]))
    print("DONE_IMAGE_BRIGHTNESS")


if __name__ == "__main__":
    main()
