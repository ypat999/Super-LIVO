#!/usr/bin/env python3
"""把 Super-LIO 输出的 odom / body cloud / world cloud 时间戳抬到主机时钟域（20a/20b 用）。

自采 move 数据图像 header 是主机时钟（1.79e9），点云与 odom 是设备时钟（1.59e9）。
方案 A 按原始 header 做图像-点云配对，这里只平移 LIO 侧时间戳（+204262818.9974 s），
图像通道不做任何中转，避免大图 relay 丢帧。

⚠️ 只平移上面三个话题的 header，**绝不平移 /tf**：super_lio 发布 odom 时会用 transformAt
把姿态换算到 tf 时间域，平移 tf 会破坏父子关系（实测相机中心被推到 345 m）。
⚠️ 偏移量是**每个 bag 一个常数**（设备钟在每次上电时重新对齐，跨 session 差几十万公里），
由环境变量 IMG_OFFSET_SEC 传入 = median(bag 录制时刻 - /livox/imu header)，
20a/20b 脚本会强制要求设置；改这里必须同步把 20_move_{a,b}_node.yaml 的 img_time_offset 置 0。
"""
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data

# 缺省值只对应旧的 move_ros2；换 bag 必须显式给 IMG_OFFSET_SEC
IMG_OFFSET_SEC = float(os.environ.get('IMG_OFFSET_SEC', '204262818.9974'))
IMG_OFFSET_NS = int(round(IMG_OFFSET_SEC * 1e9))


class ClockShift(Node):
    def __init__(self):
        super().__init__('clock_shift_relay')
        cloud_qos = qos_profile_sensor_data
        self.p_odom = self.create_publisher(Odometry, 'lio/odom_shift', 50)
        self.p_body = self.create_publisher(PointCloud2, 'lio/cloud_body_dense_shift', cloud_qos)
        self.p_world = self.create_publisher(PointCloud2, 'lio/cloud_world_shift', cloud_qos)
        self.create_subscription(Odometry, 'lio/odom', self.cb_odom, 50)
        self.create_subscription(PointCloud2, 'lio/cloud_body_dense', self.cb_body, cloud_qos)
        self.create_subscription(PointCloud2, 'lio/cloud_world', self.cb_world, cloud_qos)
        self.n = {'odom': 0, 'body': 0, 'world': 0}
        self.get_logger().info(f'clock_shift_relay: IMG_OFFSET_SEC={IMG_OFFSET_SEC:.6f}')

    @staticmethod
    def shift(msg):
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        ns += IMG_OFFSET_NS
        msg.header.stamp.sec = ns // 1_000_000_000
        msg.header.stamp.nanosec = ns % 1_000_000_000

    def cb_odom(self, msg):
        self.shift(msg)
        self.p_odom.publish(msg)
        self.n['odom'] += 1
        if self.n['odom'] % 200 == 0:
            self.get_logger().info(f"odom={self.n['odom']} body={self.n['body']} world={self.n['world']}")

    def cb_body(self, msg):
        self.shift(msg)
        self.p_body.publish(msg)
        self.n['body'] += 1

    def cb_world(self, msg):
        self.shift(msg)
        self.p_world.publish(msg)
        self.n['world'] += 1


def main():
    rclpy.init()
    node = ClockShift()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
