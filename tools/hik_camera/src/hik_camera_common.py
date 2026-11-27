#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hikrobot MV-CB013 相机公共初始化与取图封装"""

import ctypes
import os
import platform
import sys

import numpy as np
import time

import cv2

# 根据 CPU 架构自动选择 MVS SDK Python 样例目录（x86_64 -> 64，ARM64 -> aarch64）
_ARCH = "aarch64" if platform.machine() in ("aarch64", "arm64") else "64"
SDK_ROOT = os.environ.get("MVCAM_SDK_PATH", "/opt/MVS")
SDK_PYTHON_PATH = os.path.join(SDK_ROOT, "Samples", _ARCH, "Python")
if not os.path.isdir(SDK_PYTHON_PATH):
    SDK_PYTHON_PATH = os.path.join(SDK_ROOT, "Samples", "64", "Python")

# MvImport 通过这些环境变量定位 libMvCameraControl.so
os.environ.setdefault("MVCAM_SDK_PATH", SDK_ROOT)
os.environ.setdefault("MVCAM_COMMON_RUNENV", os.path.join(SDK_ROOT, "lib"))
os.environ.setdefault(
    "MVCAM_GENICAM_CLPROTOCOL",
    os.path.join(SDK_ROOT, "lib", "CLProtocol"),
)
os.environ.setdefault(
    "LD_LIBRARY_PATH",
    os.path.join(SDK_ROOT, "lib", _ARCH)
    + ":"
    + os.environ.get("LD_LIBRARY_PATH", ""),
)

if SDK_PYTHON_PATH not in sys.path:
    sys.path.append(SDK_PYTHON_PATH)

from MvImport.MvCameraControl_class import *  # noqa: E402,F401,F403


def check_ret(ret, operation):
    """检查 SDK 返回值"""
    if ret != 0:
        raise RuntimeError(
            "{} failed, error code: 0x{:08x}".format(operation, ret)
        )


def set_enum_string(cam, name, value):
    """通过 GenICam 枚举字符串设置参数"""
    ret = cam.MV_CC_SetEnumValueByString(name, value)
    check_ret(ret, "Set {}={}".format(name, value))


def get_enum_string(cam, name):
    """读取枚举节点当前值的符号名，失败返回 None"""
    st_value = MVCC_ENUMVALUE()
    ctypes.memset(ctypes.byref(st_value), 0, ctypes.sizeof(st_value))

    ret = cam.MV_CC_GetEnumValue(name, st_value)
    if ret != 0:
        return None

    st_entry = MVCC_ENUMENTRY()
    ctypes.memset(ctypes.byref(st_entry), 0, ctypes.sizeof(st_entry))
    st_entry.nValue = st_value.nCurValue

    ret = cam.MV_CC_GetEnumEntrySymbolic(name, st_entry)
    if ret != 0:
        return None

    return st_entry.chSymbolic.decode(errors="ignore")


def get_payload_size(cam):
    """获取图像缓存大小"""
    st_param = MVCC_INTVALUE()
    ctypes.memset(ctypes.byref(st_param), 0, ctypes.sizeof(st_param))

    check_ret(cam.MV_CC_GetIntValue("PayloadSize", st_param), "Get PayloadSize")

    return int(st_param.nCurValue)


def open_camera(camera_index=0):
    """枚举并打开相机，返回 MvCamera 句柄"""
    cam = MvCamera()
    device_list = MV_CC_DEVICE_INFO_LIST()

    check_ret(
        MvCamera.MV_CC_EnumDevices(
            MV_GIGE_DEVICE | MV_USB_DEVICE,
            device_list,
        ),
        "EnumDevices",
    )

    if device_list.nDeviceNum == 0:
        raise RuntimeError("未找到 Hikrobot 相机，请检查 USB 连接")

    if camera_index >= device_list.nDeviceNum:
        raise RuntimeError(
            "camera_index={} 超出范围，当前仅 {} 台相机".format(
                camera_index, device_list.nDeviceNum
            )
        )

    device_info = ctypes.cast(
        device_list.pDeviceInfo[camera_index],
        ctypes.POINTER(MV_CC_DEVICE_INFO),
    ).contents

    check_ret(cam.MV_CC_CreateHandle(device_info), "CreateHandle")

    ret = cam.MV_CC_OpenDevice()
    if ret != 0:
        cam.MV_CC_DestroyHandle()
        if ret == 0x80000203:
            raise RuntimeError(
                "OpenDevice failed: 0x80000203 (MV_E_ACCESS_DENIED)，"
                "相机已被其他进程占用，请先退出上一次运行的实例"
            )
        raise RuntimeError(
            "OpenDevice failed, error code: 0x{:08x}".format(ret)
        )

    return cam


def configure_camera(
    cam,
    trigger=False,
    exposure_us=5000.0,
    line="Line2",
    edge="RisingEdge",
    pixel_format="BGR8Packed",
    auto_exposure=False,
    exposure_min_us=100.0,
    exposure_max_us=3000.0,
    gamma=1.5,
    auto_white_balance=True,
):
    """配置采集模式（free-run 或外部硬件触发）

    auto_exposure=True: 快门优先自动曝光。ExposureAuto=Continuous，快门被夹在
    [exposure_min_us, exposure_max_us] 之间（防拖影 + 防过曝），增益锁死防噪点。
    适合室外/光照变化场景。VIO 是特征法，不受自动曝光影响。

    gamma>1.0: 提亮中间调并压缩高光，缓解"大面积暗部 + 光源过曝"共存的高动态场景。
    海康 Gamma 节点默认 1.0（线性，比手机 sRGB 曲线暗），建议 1.5~2.0。
    标定场景（auto_exposure=False）保持 1.0 以保证光度一致性。

    auto_white_balance=True: BalanceWhiteAuto=Continuous，修正工业相机默认
    WB Off + 比率 1:1:1 在 LED/阴天下偏青黄的问题。标定场景保持 Off。

    auto_exposure=False: 固定曝光，用于标定等需要光度一致性的场景。
    """
    set_enum_string(cam, "AcquisitionMode", "Continuous")

    # 先关触发，避免上次会话残留 TriggerMode=On 导致线路节点不可写
    set_enum_string(cam, "TriggerMode", "Off")

    set_enum_string(cam, "PixelFormat", pixel_format)

    if auto_exposure:
        set_enum_string(cam, "ExposureAuto", "Continuous")
        # 快门上下限可夹时优先夹住（防暗处拖影/亮处闪爆）；
        # MV-CB013 固件无 ExposureTimeUpper/Lower 节点（0x80000109），忽略即可
        for node, val in (
            ("ExposureTimeLower", exposure_min_us),
            ("ExposureTimeUpper", exposure_max_us),
        ):
            try:
                check_ret(
                    cam.MV_CC_SetFloatValue(node, float(val)), "Set " + node
                )
            except RuntimeError:
                print("[WARN] {} 不可设置（固件不支持），AE 快门不夹范围".format(node))
        # 增益锁死：防止自动增益引入噪点和帧间亮度跳变
        try:
            set_enum_string(cam, "GainAuto", "Off")
        except RuntimeError:
            pass
        # Gamma > 1 提亮中间调、压缩高光，缓解"又暗又过曝"的高动态场景
        # MV-CB013 固件 Gamma 节点只读（0x80000106），改用软件 Gamma（publisher 内 LUT）
        if gamma > 1.0:
            try:
                check_ret(cam.MV_CC_SetFloatValue("Gamma", float(gamma)),
                          "Set Gamma")
            except RuntimeError:
                print("[WARN] Gamma 硬件不可设置（固件只读），将由 publisher 软件 LUT 实现")
        # 自动白平衡：修正 WB Off + 比率 1:1:1 在 LED/阴天下偏青黄
        if auto_white_balance:
            try:
                set_enum_string(cam, "BalanceWhiteAuto", "Continuous")
            except RuntimeError:
                print("[WARN] BalanceWhiteAuto 不可设置")
    else:
        # 固定曝光，避免自动曝光在标定场景下漂移
        set_enum_string(cam, "ExposureAuto", "Off")
        check_ret(
            cam.MV_CC_SetFloatValue("ExposureTime", float(exposure_us)),
            "Set ExposureTime",
        )

    # 尽量开启硬件时间戳：MV_FRAME_OUT_INFO_EX 的 nDevTimeStamp* 字段才会非零。
    # 部分低端型号不开放 TimestampEnable，失败时忽略，后续回退为主机时间戳。
    try:
        check_ret(cam.MV_CC_SetBoolValue("TimestampEnable", True),
                  "Set TimestampEnable")
    except RuntimeError:
        pass

    if not trigger:
        return

    set_enum_string(cam, "LineSelector", line)

    # 部分型号（如 MV-CB013）Line2 的 LineMode 只读且固定为 Input
    if get_enum_string(cam, "LineMode") != "Input":
        set_enum_string(cam, "LineMode", "Input")

    # MV-CB013 只支持 FrameBurstStart，不支持 FrameStart
    set_enum_string(cam, "TriggerSelector", "FrameBurstStart")
    set_enum_string(cam, "TriggerSource", line)

    # 部分型号不开放 TriggerActivation 节点，失败时使用默认边沿
    try:
        set_enum_string(cam, "TriggerActivation", edge)
    except RuntimeError as e:
        print("[WARN] TriggerActivation 不可设置，沿用默认边沿:", e)

    set_enum_string(cam, "TriggerMode", "On")


def start_grabbing(cam):
    """开始采集并返回 SDK 图像缓存"""
    payload_size = get_payload_size(cam)
    check_ret(cam.MV_CC_StartGrabbing(), "StartGrabbing")

    return (ctypes.c_ubyte * payload_size)(), payload_size


def raw_to_bgr(data, width, height, pixel_format):
    """按相机像素格式把原始数据转成 BGR 图像"""
    if pixel_format == "BGR8Packed":
        return data[: width * height * 3].reshape((height, width, 3))

    if pixel_format == "RGB8Packed":
        image = data[: width * height * 3].reshape((height, width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if pixel_format == "Mono8":
        image = data[: width * height].reshape((height, width))
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    if pixel_format == "BayerGB8":
        # Hik 的 BayerGB8 在 OpenCV 中对应 BayerGR 相位，经实测像素差最小
        image = data[: width * height].reshape((height, width))
        return cv2.cvtColor(image, cv2.COLOR_BayerGR2BGR)

    raise ValueError("不支持的像素格式: " + pixel_format)


def grab_frame(cam, buffer, payload_size, timeout_ms=1000,
               pixel_format="BGR8Packed"):
    """阻塞取一帧并转换为 BGR，超时返回 None。

    返回值:
        (image, ts_dict) 二元组；ts_dict 包含:
            "dev_ts_ns" : 相机硬件计数器时间戳（非 UTC，仅供相对比较）
            "host_ts_ns": 主机收包时间戳（ns，墙钟，推荐直接用）
        若时间戳不可用对应值为 0。
    """
    frame_info = MV_FRAME_OUT_INFO_EX()
    ctypes.memset(ctypes.byref(frame_info), 0, ctypes.sizeof(frame_info))

    ret = cam.MV_CC_GetOneFrameTimeout(
        buffer,
        payload_size,
        frame_info,
        timeout_ms,
    )
    if ret != 0:
        print("GetOneFrameTimeout failed")
        return None, {}

    width = int(frame_info.nWidth)
    height = int(frame_info.nHeight)

    data = np.frombuffer(
        buffer,
        dtype=np.uint8,
        count=payload_size,
    )

    # 硬件时间戳：高 32 位 + 低 32 位拼起来，单位 ns。
    # 注意：这是相机内部计数器（reset 起算），不是 UTC 墙钟，
    # 只能用来做相对间隔，跨设备/跨重启绝对不可直接比较。
    dev_ts_ns = (int(frame_info.nDevTimeStampHigh) << 32) | int(frame_info.nDevTimeStampLow)

    # 主机端时间戳：MV SDK 在收包时直接取的墙钟值。
    # 不同平台单位不一致：Windows 是 ns，Linux aarch64 (Jetson) 实测是 ms。
    # 用「和主机真实时间量级差」判断：差 > 1e3 倍 就乘 1e6 转成 ns。
    host_ts_raw = int(frame_info.nHostTimeStamp)
    host_ts_ns = host_ts_raw
    if host_ts_raw:
        now_ns = time.time_ns()
        ratio = now_ns / host_ts_raw
        if ratio > 1_000:   # 说明 host_ts 量级比现在小 ~1000 倍 → 是 ms，转 ns
            host_ts_ns = host_ts_raw * 1_000_000
        elif ratio < 1/1_000:   # host_ts 比现在大 ~1000 倍（罕见，但保险）→ 当作 us，转 ns
            host_ts_ns = host_ts_raw // 1_000

    img = raw_to_bgr(data, width, height, pixel_format)
    return img, {"dev_ts_ns": dev_ts_ns, "host_ts_ns": host_ts_ns}


def close_camera(cam):
    """停止采集并释放相机"""
    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
