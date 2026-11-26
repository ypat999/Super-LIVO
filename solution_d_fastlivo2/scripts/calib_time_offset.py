#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""现场标定 雷达钟 - 相机钟 的偏移（img_time_offset）。

原理与方案 A 的在线标定一致：把两路消息的接收时刻当作宿主钟，
    offset = (lidar_stamp - lidar_arrival) - (img_stamp - img_arrival)
等价于  lidar_stamp - img_stamp 在"同一宿主时刻"的差，
因此只包含传感器自身延迟（毫秒级），不会引入两帧之间的时间间隔误差。

用中位数抵抗抖动，输出一行：
    IMG_TIME_OFFSET=<秒>

用法：
    python3 calib_time_offset.py [--lidar /livox/lidar] [--image /left_camera/image]
                                 [--samples 8] [--timeout 20]
"""

import argparse
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# livox_ros_driver2 的 CustomMsg；没 source 到驱动时会 ImportError，直接提示
try:
    from livox_ros_driver2.msg import CustomMsg
except Exception:  # noqa: BLE001
    print("ERR livox_ros_driver2.msg 不可用，请先 source 驱动 install/setup.bash",
          file=sys.stderr)
    raise

from sensor_msgs.msg import Image


class Calib(Node):
    def __init__(self, lidar_topic: str, image_topic: str):
        super().__init__("d_time_offset_calib")
        # 本机 livox_ros_driver2 用 create_publisher(topic, queue_size)（reliable），
        # 海康相机也是 reliable；best_effort 订阅会匹配不上，收不到数据。
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.lid_samples = []  # (stamp - arrival)
        self.img_samples = []
        self.create_subscription(CustomMsg, lidar_topic, self.on_lidar, qos)
        self.create_subscription(Image, image_topic, self.on_img, qos)

    def on_lidar(self, msg):
        arrival = time.time()
        self.lid_samples.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - arrival)

    def on_img(self, msg):
        arrival = time.time()
        self.img_samples.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - arrival)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lidar", default="/livox/lidar")
    ap.add_argument("--image", default="/left_camera/image")
    ap.add_argument("--samples", type=int, default=8)
    ap.add_argument("--timeout", type=float, default=25.0)
    args = ap.parse_args()

    rclpy.init()
    node = Calib(args.lidar, args.image)
    deadline = time.time() + args.timeout
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(node.lid_samples) >= args.samples and len(node.img_samples) >= 3:
                break
    finally:
        lid = list(node.lid_samples)
        img = list(node.img_samples)
        node.destroy_node()
        rclpy.shutdown()

    if not lid or not img:
        print(f"ERR 采样不足 lidar={len(lid)} image={len(img)}", file=sys.stderr)
        return 2

    # 每路各自取中位数，再相减
    offset = statistics.median(lid) - statistics.median(img)
    print(f"IMG_TIME_OFFSET={offset:.6f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
