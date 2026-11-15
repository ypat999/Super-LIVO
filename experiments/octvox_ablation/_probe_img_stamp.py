#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 raw bag 中 /left_camera/image 的首末 header 时间。"""
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

path = "/home/ywj/slam_data/Retail_Street"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("cdr", "cdr"))
types = {i.name: i.type for i in r.get_all_topics_and_types()}
st = r.get_metadata().starting_time if hasattr(r, "get_metadata") else None
cls = get_message(types["/left_camera/image"])
first = None
last = None
while r.has_next():
    topic, data, t = r.read_next()
    if topic != "/left_camera/image":
        continue
    m = deserialize_message(data, cls)
    hs = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if first is None:
        first = (t, hs)
    last = (t, hs)
print("/left_camera/image 首: rec=%.6f header=%.6f" % first)
print("/left_camera/image 末: rec=%.6f header=%.6f" % last)
