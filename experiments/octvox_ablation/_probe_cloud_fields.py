#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印几个点云话题的字段布局，定位 pcl::fromROSMsg 匹配失败的原因。"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

TOPICS = [
    "/cloud_registered",
    "/cloud_registered_body",
]

rclpy.init()
node = Node("probe_cloud_fields")
qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT
got = {}


def cb(topic):
    def inner(msg):
        if topic in got:
            return
        got[topic] = True
        print("=== %s" % topic)
        print("  width=%d height=%d point_step=%d row_step=%d is_dense=%s"
              % (msg.width, msg.height, msg.point_step, msg.row_step, msg.is_dense))
        print("  fields: " + ", ".join(
            "%s(off=%d type=%d)" % (f.name, f.offset, f.datatype) for f in msg.fields))
        print("  data_len=%d -> 推测点数=%d"
              % (len(msg.data), len(msg.data) // max(1, msg.point_step)))
    return inner


for t in TOPICS:
    node.create_subscription(PointCloud2, t, cb(t), qos)

for _ in range(200):
    rclpy.spin_once(node, timeout_sec=0.1)
    if len(got) == len(TOPICS):
        break
missing = [t for t in TOPICS if t not in got]
if missing:
    print("未收到:", missing)
node.destroy_node()
rclpy.shutdown()
