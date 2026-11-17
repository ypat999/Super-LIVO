#!/usr/bin/env python3
"""外部延时探针：同一份 CSV 里记录「雷达帧到达」与「odom 到达」两个事件。

用途：比较不同前端（FL2 / 方案A / 方案B / Super-LIO）的 odom 输出节拍与整帧延时。
不侵入各节点内部计时，只用 ROS 订阅观测：

  kind,recv_ns,stamp_ns
  lidar,<墙钟到达 ns>,<header.stamp ns>
  odom ,<墙钟到达 ns>,<header.stamp ns>

后处理（tools/latency_summary.py）按 stamp 最近邻配对，得到
  algo_ms   = odom 到达 − 对应雷达帧到达          （纯处理+传输延时，口径无歧义）
  e2e_ms    = odom 到达 − 对应雷达帧 header.stamp （传感器时刻→odom 可用）
  period_ms = 相邻 odom header.stamp 间隔          （odom 输出节拍）

注意各臂 stamp 约定不同：Super-LIO / A / B 的 odom stamp = 雷达帧 start_time，
FL2 的 /aft_mapped_to_init stamp = last_lio_update_time（≈帧尾），故 e2e 里
FL2 天然多出约一个扫描周期（~100 ms），跨臂比较以 algo_ms 为准，e2e 作参考。

用法：
  python3 tools/latency_probe.py --lidar-topic /livox/lidar --lidar-type custom \
      --odom-topic /lio/odom --out /path/latency.csv
收到 SIGINT/SIGTERM 后 flush 退出。
"""
import argparse
import csv
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry


def _stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _recv_ns():
    return time.time_ns()


class LatencyProbe(Node):
    def __init__(self, args):
        super().__init__("paper_latency_probe")
        self._f = open(args.out, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(["kind", "recv_ns", "stamp_ns"])
        self._n_lidar = 0
        self._n_odom = 0

        # odom：方案A/B 的发布端是 BEST_EFFORT，Super-LIO/FL2 是 RELIABLE；
        # 订阅端统一用 BEST_EFFORT，两种发布端都兼容（反之会因 QoS 不兼容收不到）。
        qos_be = QoSProfile(depth=2000, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        # 雷达：用 BEST_EFFORT + 小队列。之前用 RELIABLE/depth=2000，等于让回放端
        # 为这个 reader 保留 2000 个样本（move 的 CustomMsg 单帧 ~0.8 MB ≈ 1.6 GB），
        # 实测会导致两类假象：
        #   1) move a/b/fl2 约 76% 的 odom 出现负 algo（odom 比"对应雷达帧到达"更早），
        #      即探针自己的雷达队列积压了 ~1.6 s，odom（小消息）却没积压；
        #   2) room/sl、retail/sl 的回放端被这个慢 reader 拖住，160 s 的包播了 299 s。
        # 改 BEST_EFFORT 后：队列满时丢最旧的，到达时刻永远是"真实收到时刻"，
        # 且不会反压回放端；漏掉的帧只影响配对样本数，不影响每个已配对样本的取值。
        # depth=20（≈2 s）足以吸收调度抖动，正常 10 Hz 回放不会丢帧。
        qos_lidar = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)

        if args.lidar_type == "custom":
            from livox_ros_driver2.msg import CustomMsg
            self.create_subscription(CustomMsg, args.lidar_topic, self._cb_lidar, qos_lidar)
        else:
            from sensor_msgs.msg import PointCloud2
            self.create_subscription(PointCloud2, args.lidar_topic, self._cb_lidar, qos_lidar)

        self.create_subscription(Odometry, args.odom_topic, self._cb_odom, qos_be)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"probe: lidar={args.lidar_topic}({args.lidar_type}) odom={args.odom_topic} -> {args.out}")

    def _cb_lidar(self, msg):
        self._w.writerow(["lidar", _recv_ns(), _stamp_ns(msg)])
        self._n_lidar += 1

    def _cb_odom(self, msg):
        self._w.writerow(["odom", _recv_ns(), _stamp_ns(msg)])
        self._n_odom += 1

    def _tick(self):
        self._f.flush()

    def close(self):
        try:
            self._f.flush()
            self._f.close()
        except Exception:
            pass
        print(f"[latency_probe] lidar={self._n_lidar} odom={self._n_odom}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lidar-topic", required=True)
    ap.add_argument("--lidar-type", choices=["custom", "pc2"], default="pc2")
    ap.add_argument("--odom-topic", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    rclpy.init()
    node = LatencyProbe(args)

    def _stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
