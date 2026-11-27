#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时采集 Hikrobot 图像并发布到 ROS2 话题（sensor_msgs/Image, bgr8）"""

import array
import datetime as dt
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

import hik_camera_common as hik


class HikCameraPublisher(Node):
    def __init__(self):
        super().__init__("hik_camera_publisher")

        self.declare_parameter("topic", "/left_camera/image")
        self.declare_parameter("frame_id", "left_camera")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("trigger", True)
        self.declare_parameter("exposure_us", 12000.0)
        # 快门优先自动曝光（室外防过曝/室内防拖影）；标定场景置 False 回固定曝光
        self.declare_parameter("auto_exposure", True)
        self.declare_parameter("exposure_min_us", 100.0)
        self.declare_parameter("exposure_max_us", 3000.0)
        # Gamma > 1 提亮中间调、压缩高光（硬件节点只读，用软件 LUT 实现）
        # 默认 1.0=关；实测白天室内 raw mean≈120 曝光正确，开 1.5 反而过亮
        self.declare_parameter("gamma", 1.0)
        # 自动白平衡：修正工业相机默认 WB Off 在 LED/阴天下偏青黄
        self.declare_parameter("auto_white_balance", True)
        # USB2 带宽有限：BayerGB8 约 15 fps，BGR8Packed 仅约 6 fps
        self.declare_parameter("pixel_format", "BayerGB8")
        self.declare_parameter("timeout_ms", 1000)
        self.declare_parameter("log_every", 50)

        self.topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.camera_index = self.get_parameter("camera_index").value
        self.trigger = self.get_parameter("trigger").value
        self.exposure_us = self.get_parameter("exposure_us").value
        self.auto_exposure = self.get_parameter("auto_exposure").value
        self.exposure_min_us = self.get_parameter("exposure_min_us").value
        self.exposure_max_us = self.get_parameter("exposure_max_us").value
        self.gamma = self.get_parameter("gamma").value
        self.auto_white_balance = self.get_parameter("auto_white_balance").value
        # 软件 Gamma LUT（硬件 Gamma 节点只读，0x80000106）
        if self.gamma > 0 and self.gamma != 1.0:
            inv = 1.0 / self.gamma
            self._gamma_lut = np.array(
                [int(np.clip(255 * np.power(i / 255.0, inv), 0, 255))
                 for i in range(256)], dtype=np.uint8)
        else:
            self._gamma_lut = None
        self.pixel_format = self.get_parameter("pixel_format").value
        self.timeout_ms = self.get_parameter("timeout_ms").value
        self.log_every = self.get_parameter("log_every").value

        # 用 reliable：rosbag2 默认按 reliable 订阅，best effort 会录不到；
        # ros2 topic echo/hz 用 sensor data QoS，订阅端更弱也能匹配
        qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(Image, self.topic, qos)

        self.cam = None
        self.buffer = None
        self.payload_size = 0

    def start(self):
        self.cam = hik.open_camera(self.camera_index)
        hik.configure_camera(
            self.cam,
            trigger=self.trigger,
            exposure_us=self.exposure_us,
            pixel_format=self.pixel_format,
            auto_exposure=self.auto_exposure,
            exposure_min_us=self.exposure_min_us,
            exposure_max_us=self.exposure_max_us,
            gamma=self.gamma,
            auto_white_balance=self.auto_white_balance,
        )
        self.buffer, self.payload_size = hik.start_grabbing(self.cam)

        self.get_logger().info(
            "相机已启动，发布到 {}（模式：{}，采集格式：{}，发布编码：bgr8，曝光：{}，Gamma={}，AWB={}）".format(
                self.topic,
                "Line2 硬件触发" if self.trigger else "free-run",
                self.pixel_format,
                "AE 100-3000us" if self.auto_exposure
                else "固定 {:.0f}us".format(self.exposure_us),
                self.gamma,
                "Continuous" if self.auto_white_balance else "Off",
            )
        )

    def spin(self):
        count = 0
        t_start = time.time()
        # 用 list 存最近 dev 间隔（ms），只每帧 append，打印时求均值——
        # 这样不管 log_every 是多少，统计的始终是「相邻帧间隔」。
        self._dev_interval_ms = []
        self._last_dev_ts = None

        while rclpy.ok():
            image, ts = hik.grab_frame(
                self.cam,
                self.buffer,
                self.payload_size,
                self.timeout_ms,
                self.pixel_format,
            )
            if image is None:
                self.get_logger().warn("获取到空图像")
                continue
            # 软件 Gamma 校正（硬件节点只读，用 LUT 实现）
            if self._gamma_lut is not None:
                image = cv2.LUT(image, self._gamma_lut)
            # 取帧阻塞期间可能已收到 Ctrl-C，避免关闭后再发布
            if not rclpy.ok():
                break

            height, width = image.shape[:2]

            # 记录时间戳信息（仅用于日志，真正发布时以 host_ts 为准）
            dev_ts_ns  = ts.get("dev_ts_ns", 0)
            host_ts_ns = ts.get("host_ts_ns", 0)

            # 每帧都算一下 dev_ts 间隔并累积（只看相邻帧差，和 log_every 无关）
            if self._last_dev_ts and dev_ts_ns:
                self._dev_interval_ms.append((dev_ts_ns - self._last_dev_ts) * 1e-6)
            self._last_dev_ts = dev_ts_ns

            msg = Image()
            # 优先用 SDK 填充的 host_ts（MV_CC 直接取的墙钟 ns，无 Python 开销），
            # 若该字段为 0 再退回 self.get_clock().now()。
            msg.header.stamp = (
                Time(nanoseconds=host_ts_ns).to_msg()
                if host_ts_ns
                else self.get_clock().now().to_msg()
            )
            msg.header.frame_id = self.frame_id
            msg.height = height
            msg.width = width
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = width * 3
            # 必须用 array.array('B')：直接赋 bytes 会被 rclpy 逐元素校验，
            # 390 万像素时单帧要 3 秒左右
            msg.data = array.array("B", image.tobytes())

            try:
                self.pub.publish(msg)
            except Exception:
                # Ctrl-C 已关闭上下文时发布失败，按正常退出处理
                if not rclpy.ok():
                    break
                raise

            count += 1
            if self.log_every > 0 and count % self.log_every == 0:
                t_now = time.time()
                fps = self.log_every / max(t_now - t_start, 1e-6)
                t_start = t_now

                dev_interval_str = ""
                if self._dev_interval_ms:
                    ds = self._dev_interval_ms
                    dev_interval_str = (
                        f", dev 间隔 mean={sum(ds)/len(ds):.1f} ms"
                        f" (min={min(ds):.1f}, max={max(ds):.1f}, n={len(ds)})"
                    )
                    self._dev_interval_ms.clear()

                # host_ts_ns: SDK 填充的主机墙钟 ns，和 time.time() 秒级对齐
                host_ts_str = ""
                if host_ts_ns:
                    host_ts_sec = host_ts_ns / 1e9
                    readable = dt.datetime.fromtimestamp(host_ts_sec).strftime(
                        "%Y-%m-%d %H:%M:%S.%f"
                    )[:-3]
                    host_ts_str = f", host_ts={host_ts_ns} ({readable})"

                self.get_logger().info(
                    "已发布 {} 帧 ({}x{}, {:.1f} fps{}{})".format(
                        count, width, height, fps, dev_interval_str, host_ts_str
                    )
                )

    def _make_stamp(self, dev_ts_ns):
        """保留占位，不再实际调用（之前用于 dev_ts 校验逻辑）。"""
        return self.get_clock().now().to_msg()

    def stop(self):
        if self.cam is not None:
            hik.close_camera(self.cam)
            self.cam = None
            self.get_logger().info("相机已释放")


def main():
    rclpy.init()
    node = HikCameraPublisher()
    try:
        node.start()
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
