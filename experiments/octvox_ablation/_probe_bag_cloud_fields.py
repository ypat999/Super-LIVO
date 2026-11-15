#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查录制包里 /cloud_registered_body 与 /cloud_registered 的字段布局。"""
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

path = "/home/ywj/slam_data/official_vio_20261114/bag"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("cdr", "cdr"))
types = {i.name: i.type for i in r.get_all_topics_and_types()}
print(types)
want = {"/cloud_registered_body", "/cloud_registered"}
cls = {t: get_message(types[t]) for t in want}
got = set()
while r.has_next() and got != want:
    topic, data, t = r.read_next()
    if topic not in want or topic in got:
        continue
    m = deserialize_message(data, cls[topic])
    print("== %s type=%s w=%d h=%d step=%d fields=%s"
          % (topic, types[topic], m.width, m.height, m.point_step,
             [(f.name, f.offset, f.datatype, f.count) for f in m.fields]))
    got.add(topic)
