#!/usr/bin/env python3
"""退化注入：把 LiDAR 点云裁成纯平面（只留地面带），让 Super-LIO 的
computeDegeneracy() 真实报出高条件数。相机不动 —— 视觉仍能看到全部纹理。

用法：
  MODE=ground Z_HALF=0.3 python3 /tmp/cloud_crop.py     # 只留 |z|<0.3 的地面
  MODE=corridor Y_HALF=1.2 python3 /tmp/cloud_crop.py   # 只留 |y|<1.2 的走廊
Super-LIO 侧：-r /os1_cloud_node1/points:=/degen/points
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

MODE = os.environ.get("MODE", "ground")
Z_HALF = float(os.environ.get("Z_HALF", "0.3"))
# 地面带中心：传感器系下地面的 z。|z|<0.3 是"传感器同高度的竖直薄层"（不是地面），
# 会把所有结构点删掉导致 n_effect=0。要造平移退化应保留 z∈[C-H, C+H] 的地面带。
Z_CENTER = float(os.environ.get("Z_CENTER", "0.0"))
Y_HALF = float(os.environ.get("Y_HALF", "1.2"))
IN_TOPIC = os.environ.get("IN_TOPIC", "/os1_cloud_node1/points")
OUT_TOPIC = os.environ.get("OUT_TOPIC", "/degen/points")
# 退化注入的起始时刻（秒，相对本节点收到的第一帧）：之前原样透传，让 Super-LIO
# 先用完整点云建好地图/初始化；之后才开始裁剪，模拟"健康 -> 退化"的真实切换。
START_OFFSET = float(os.environ.get("START_OFFSET", "0"))


class Cropper(Node):
    def __init__(self):
        super().__init__("cloud_crop")
        self.pub = self.create_publisher(PointCloud2, OUT_TOPIC, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2, IN_TOPIC, self.cb, qos_profile_sensor_data)
        self.kept = 0
        self.total = 0
        self.t_first = None
        self.get_logger().info(
            f"裁剪模式={MODE} z_half={Z_HALF} y_half={Y_HALF} start={START_OFFSET}s "
            f"{IN_TOPIC} -> {OUT_TOPIC}")

    def cb(self, msg):
        if self.t_first is None:
            self.t_first = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if now - self.t_first < START_OFFSET:
            self.pub.publish(msg)  # 透传，保留完整几何
            return
        fields = {f.name: f.offset for f in msg.fields}
        if not {"x", "y", "z"} <= set(fields):
            return
        ox, oy, oz = fields["x"], fields["y"], fields["z"]
        ps = msg.point_step
        w = msg.width
        h = msg.height
        row_step = msg.row_step
        raw = np.frombuffer(msg.data, dtype=np.uint8)

        if row_step == w * ps and h == 1:
            arr = raw.reshape(-1, ps)
        else:
            rows = []
            for r in range(h):
                s = r * row_step
                rows.append(raw[s:s + w * ps].reshape(-1, ps))
            arr = np.concatenate(rows, axis=0)

        def f32(col):
            return np.ascontiguousarray(arr[:, col:col + 4]).view(np.float32).ravel()

        x, y, z = f32(ox), f32(oy), f32(oz)
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if MODE == "ground":
            mask = finite & (np.abs(z - Z_CENTER) < Z_HALF)
        elif MODE == "corridor":
            mask = finite & (np.abs(y) < Y_HALF)
        else:  # passthrough
            mask = finite

        keep = arr[mask]
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(keep.shape[0])
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = ps
        out.row_step = int(keep.shape[0]) * ps
        out.is_dense = False
        out.data = keep.tobytes()
        self.pub.publish(out)

        self.kept += int(keep.shape[0])
        self.total += int(arr.shape[0])
        self.get_logger().info(
            f"裁剪 {keep.shape[0]}/{arr.shape[0]} 点，累计保留率 "
            f"{100.0 * self.kept / max(1, self.total):.1f}%", throttle_duration_sec=2.0)


def main():
    rclpy.init()
    n = Cropper()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
