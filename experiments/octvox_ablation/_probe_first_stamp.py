#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印两个 bag 中各 topic 首条消息的 header.stamp 与录制时间，用于对齐回放。"""
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAGS = {
    "fl2": "/home/ywj/slam_data/official_vio_20261114/bag",
    "raw": "/home/ywj/slam_data/Retail_Street",
}
for tag, path in BAGS.items():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {i.name: i.type for i in r.get_all_topics_and_types()}
    seen = {}
    while r.has_next():
        topic, data, t = r.read_next()
        if topic in seen:
            continue
        m = deserialize_message(data, get_message(types[topic]))
        h = getattr(m, "header", None)
        hs = (h.stamp.sec + h.stamp.nanosec * 1e-9) if h is not None else float("nan")
        seen[topic] = (t, hs)
        if len(seen) >= len(types):
            break
    print("== %s (%s)" % (tag, path))
    for k in sorted(seen):
        print("   %-30s rec=%.6f header=%.6f" % (k, seen[k][0], seen[k][1]))
