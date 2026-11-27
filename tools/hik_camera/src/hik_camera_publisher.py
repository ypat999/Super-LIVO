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
        # AE 模式：camera=相机内置（测光目标固件写死 ~128，无补偿节点）；
        # software=自研闭环（raw mean 反馈调快门/增益，目标可调）。
        # 固件实测无 ExposureAutoAdjustTarget/ExposureCompensation/AOI-AE，
        # 想"硬件降曝光+Gamma 提回"必须走 software。
        self.declare_parameter("ae_mode", "software")
        self.declare_parameter("ae_target_mean", 88.0)      # 闭环目标（≈-0.5EV）
        self.declare_parameter("ae_shutter_min_us", 100.0)
        self.declare_parameter("ae_shutter_max_us", 9900.0)  # 10Hz 触发周期内
        self.declare_parameter("ae_gain_max_db", 16.0)
        # Gamma > 1 提亮中间调、压缩高光（硬件节点只读，用软件 LUT 实现）
        self.declare_parameter("gamma", 1.5)
        # 静态白平衡微调（B,G,R 通道增益）：抵消 AWB 在大面积绿叶等
        # 非灰场景下的偏暖漂移。相机端无 BalanceRatio 节点，只能主机侧做。
        self.declare_parameter("wb_trim", "1.04,1.0,0.96")
        # 饱和度增益：new = s*img + (1-s)*gray（保亮度提彩度）。
        # 相机端 Saturation/Hue/Contrast 节点均不存在，只能主机侧做。
        self.declare_parameter("saturation_gain", 1.3)
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
        self.ae_mode = self.get_parameter("ae_mode").value
        self.ae_target = float(self.get_parameter("ae_target_mean").value)
        self.ae_shutter_min = float(self.get_parameter("ae_shutter_min_us").value)
        self.ae_shutter_max = float(self.get_parameter("ae_shutter_max_us").value)
        self.ae_gain_max = float(self.get_parameter("ae_gain_max_db").value)
        self._shutter_us = 5000.0
        self._gain_db = 0.0
        self._ae_window = []
        self.wb_trim = [float(x) for x in
                        str(self.get_parameter("wb_trim").value).split(",")]
        self.saturation_gain = float(self.get_parameter("saturation_gain").value)
        # 合成 LUT：每通道 = 静态白平衡微调 ∘ Gamma（均为逐通道映射，零额外开销）
        inv = 1.0 / self.gamma if self.gamma > 0 else 1.0
        ramp = np.arange(256, dtype=np.float64) / 255.0
        lut = np.empty((256, 1, 3), dtype=np.uint8)
        for c in range(3):
            g = self.wb_trim[c] if len(self.wb_trim) == 3 else 1.0
            v = 255.0 * g * np.power(ramp, inv)
            lut[:, 0, c] = np.clip(v, 0, 255).astype(np.uint8)
        self._gamma_lut = lut if (inv != 1.0 or any(
            abs(x - 1.0) > 1e-3 for x in self.wb_trim)) else None
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
        # software 闭环自己管快门/增益：相机内置 AE 必须关，否则两边抢执行器
        use_cam_ae = self.auto_exposure and self.ae_mode != "software"
        hik.configure_camera(
            self.cam,
            trigger=self.trigger,
            exposure_us=self.exposure_us,
            pixel_format=self.pixel_format,
            auto_exposure=use_cam_ae,
            exposure_min_us=self.exposure_min_us,
            exposure_max_us=self.exposure_max_us,
            gamma=self.gamma,
            auto_white_balance=self.auto_white_balance,
        )
        if self.ae_mode == "software":
            # 以当前固定曝光值起步；增益从 0 起
            from MvImport.MvCameraControl_class import MVCC_FLOATVALUE
            import ctypes
            v = MVCC_FLOATVALUE()
            ctypes.memset(ctypes.byref(v), 0, ctypes.sizeof(v))
            if self.cam.MV_CC_GetFloatValue("ExposureTime", v) == 0 and v.fCurValue > 0:
                self._shutter_us = float(v.fCurValue)
            self._gain_db = 0.0
            self.cam.MV_CC_SetFloatValue("Gain", self._gain_db)
        self.buffer, self.payload_size = hik.start_grabbing(self.cam)

        self.get_logger().info(
            "相机已启动，发布到 {}（模式：{}，采集格式：{}，发布编码：bgr8，曝光：{}，Gamma={}，AWB={}）".format(
                self.topic,
                "Line2 硬件触发" if self.trigger else "free-run",
                self.pixel_format,
                "软件闭环 目标{:.0f}".format(self.ae_target)
                if self.ae_mode == "software"
                else ("相机AE" if self.auto_exposure
                      else "固定 {:.0f}us".format(self.exposure_us)),
                self.gamma,
                "Continuous" if self.auto_white_balance else "Off",
            )
            + "，WB微调={} 饱和度={:.2f}".format(
                self.wb_trim, self.saturation_gain,
            )
        )

    def _tune_exposure(self, mean):
        """软件闭环：raw mean 反馈，先调快门、快门到限再调增益。"""
        if mean <= 1.0:
            return
        err = max(0.4, min(2.5, self.ae_target / mean))  # >1 偏暗需加曝光
        s = self._shutter_us * (err ** 0.7)
        s = max(self.ae_shutter_min, min(self.ae_shutter_max, s))
        if abs(s - self._shutter_us) > self._shutter_us * 0.05:
            self.cam.MV_CC_SetFloatValue("ExposureTime", s)
            self._shutter_us = s
            return
        # 快门已贴限：用增益继续逼近
        g = self._gain_db
        if self._shutter_us >= self.ae_shutter_max * 0.95 and err > 1.15:
            g = min(self.ae_gain_max, self._gain_db + 2.0)
        elif self._shutter_us <= self.ae_shutter_min * 1.05 and err < 0.87:
            g = max(0.0, self._gain_db - 2.0)
        if abs(g - self._gain_db) > 0.1:
            self.cam.MV_CC_SetFloatValue("Gain", g)
            self._gain_db = g

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
            # 软件闭环 AE：用 LUT 前的 raw 灰度均值反馈，每 10 帧调一次（≈1 s）
            if self.ae_mode == "software":
                self._ae_window.append(float(cv2.cvtColor(
                    image, cv2.COLOR_BGR2GRAY).mean()))
                if len(self._ae_window) >= 10:
                    self._tune_exposure(
                        sum(self._ae_window) / len(self._ae_window))
                    self._ae_window = []
            # 软件 Gamma 校正（硬件节点只读，用 LUT 实现；WB 微调已并入 LUT）
            if self._gamma_lut is not None:
                image = cv2.LUT(image, self._gamma_lut)
            # 饱和度提升：new = s*img + (1-s)*gray，保亮度只提彩度
            s = self.saturation_gain
            if s > 1.01:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                gray = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                image = cv2.addWeighted(image, s, gray, 1.0 - s, 0)
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
                    "已发布 {} 帧 ({}x{}, {:.1f} fps{}{}{})".format(
                        count, width, height, fps, dev_interval_str, host_ts_str,
                        ", 快门={:.0f}us 增益={:.1f}dB".format(
                            self._shutter_us, self._gain_db)
                        if self.ae_mode == "software" else ""
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
