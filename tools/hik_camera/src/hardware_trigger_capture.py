
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import json
import ctypes
import platform
from datetime import datetime

import numpy as np
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

from MvImport.MvCameraControl_class import *


# =========================
# 用户配置
# =========================

SAVE_DIR = os.path.expanduser("./images")

CAMERA_INDEX = 0

# 外部触发输入线路
TRIGGER_LINE = "Line2"

# 上升沿触发
TRIGGER_EDGE = "RisingEdge"

# 单次等待超时（毫秒）
GRAB_TIMEOUT_MS = 1000

# JPG质量
JPEG_QUALITY = 95

# 是否开启图像预览窗口（无显示环境会自动降级为关闭）
ENABLE_PREVIEW = True

# 预览窗口名称
PREVIEW_WINDOW = "Hik Trigger Capture"


def check_ret(ret, operation):
    """检查SDK返回值"""
    if ret != 0:
        raise RuntimeError(
            "{} failed, error code: 0x{:08x}".format(
                operation, ret
            )
        )


def set_enum_string(cam, name, value):
    """通过GenICam枚举字符串设置参数"""
    if not hasattr(cam, "MV_CC_SetEnumValueByString"):
        raise RuntimeError(
            "当前SDK Python接口没有 "
            "MV_CC_SetEnumValueByString，"
            "请检查SDK版本和MvCameraControl_class.py"
        )

    ret = cam.MV_CC_SetEnumValueByString(
        name,
        value
    )

    check_ret(ret, "Set " + name + "=" + value)


def get_enum_string(cam, name):
    """读取枚举节点当前值的符号名，失败返回 None"""
    st_value = MVCC_ENUMVALUE()
    ctypes.memset(
        ctypes.byref(st_value),
        0,
        ctypes.sizeof(st_value)
    )

    ret = cam.MV_CC_GetEnumValue(name, st_value)
    if ret != 0:
        return None

    st_entry = MVCC_ENUMENTRY()
    ctypes.memset(
        ctypes.byref(st_entry),
        0,
        ctypes.sizeof(st_entry)
    )
    st_entry.nValue = st_value.nCurValue

    ret = cam.MV_CC_GetEnumEntrySymbolic(name, st_entry)
    if ret != 0:
        return None

    return st_entry.chSymbolic.decode(
        errors="ignore"
    )


def get_payload_size(cam):
    """获取图像缓存大小"""
    st_param = MVCC_INTVALUE()
    ctypes.memset(
        ctypes.byref(st_param),
        0,
        ctypes.sizeof(st_param)
    )

    ret = cam.MV_CC_GetIntValue(
        "PayloadSize",
        st_param
    )

    check_ret(ret, "Get PayloadSize")

    return int(st_param.nCurValue)


def configure_camera(cam):
    """配置相机外部触发"""

    # 连续采集模式，实际曝光由外部触发控制
    set_enum_string(
        cam, "AcquisitionMode", "Continuous"
    )

    # 先关闭触发
    # 相机参数在重开 SDK 后会保留，若上次会话残留 TriggerMode=On，
    # LineMode 会变为不可写（0x80000106 MV_E_NODATA）
    set_enum_string(
        cam, "TriggerMode", "Off"
    )

    # 先选择外部输入线路
    set_enum_string(
        cam, "LineSelector", TRIGGER_LINE
    )

    # 将该线路设置为输入
    # 部分型号（如 MV-CB013）Line2 的 LineMode 为只读节点且固定为 Input，
    # 此时再次写入会返回 0x80000106 (MV_E_GC_ACCESS)，因此已是 Input 就跳过
    current_line_mode = get_enum_string(cam, "LineMode")
    if current_line_mode == "Input":
        print("[INFO] LineMode 已是 Input（只读节点），跳过设置")
    else:
        set_enum_string(
            cam, "LineMode", "Input"
        )

    # 设置触发选择器
    # 注意：MV-CB013 系列只支持 FrameBurstStart，不支持 FrameStart
    set_enum_string(
        cam, "TriggerSelector", "FrameBurstStart"
    )

    # 开启触发模式
    set_enum_string(
        cam, "TriggerMode", "On"
    )

    # 设置触发来源
    set_enum_string(
        cam, "TriggerSource", TRIGGER_LINE
    )

    # 设置触发边沿
    # 部分型号不开放 TriggerActivation 节点，失败时使用相机默认边沿
    try:
        set_enum_string(
            cam, "TriggerActivation", TRIGGER_EDGE
        )
    except RuntimeError as e:
        print("[WARN] TriggerActivation 不可设置，沿用相机默认边沿:", e)

    # 设置输出图像格式为BGR8
    # 相机支持的格式和实际SDK枚举以MVS为准
    set_enum_string(
        cam, "PixelFormat", "BGR8Packed"
    )

    # 关闭自动曝光，使用固定曝光
    # 数值单位通常为微秒
    ret = cam.MV_CC_SetEnumValueByString(
        "ExposureAuto", "Off"
    )
    check_ret(ret, "Disable ExposureAuto")

    ret = cam.MV_CC_SetFloatValue(
        "ExposureTime", 5000.0
    )
    check_ret(ret, "Set ExposureTime")

    print("[INFO] Camera trigger configured")
    print("[INFO] Trigger line:", TRIGGER_LINE)
    print("[INFO] Trigger edge:", TRIGGER_EDGE)


def save_image(buffer, frame_info, frame_index):
    """将BGR8图像保存为JPG，返回 image 数组以供预览"""

    width = int(frame_info.nWidth)
    height = int(frame_info.nHeight)

    expected_size = width * height * 3

    image_data = np.frombuffer(
        buffer,
        dtype=np.uint8,
        count=expected_size
    )

    image = image_data.reshape(
        (height, width, 3)
    )

    filename = "{:08d}.jpg".format(frame_index)
    filepath = os.path.join(SAVE_DIR, filename)

    ok = cv2.imwrite(
        filepath,
        image,
        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    )

    if not ok:
        raise RuntimeError("Failed to save: " + filepath)

    # 主机接收到图像的时间，不等同于曝光时刻
    host_time = datetime.now().astimezone().isoformat(
        timespec="microseconds"
    )

    record = {
        "frame_index": frame_index,
        "filename": filename,
        "host_receive_time": host_time,
        "camera_frame_number": int(frame_info.nFrameNum),
        "width": width,
        "height": height,
        "pixel_format": "BGR8Packed"
    }

    with open(
        os.path.join(SAVE_DIR, "timestamps.jsonl"),
        "a",
        encoding="utf-8"
    ) as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")

    print(
        "[SAVED] {}  {}x{}  frame={}".format(
            filename,
            width,
            height,
            frame_info.nFrameNum
        )
    )

    return image


def main():

    os.makedirs(SAVE_DIR, exist_ok=True)

    cam = MvCamera()

    device_list = MV_CC_DEVICE_INFO_LIST()

    # 枚举USB和GigE相机
    ret = MvCamera.MV_CC_EnumDevices(
        MV_GIGE_DEVICE | MV_USB_DEVICE,
        device_list
    )
    check_ret(ret, "EnumDevices")

    if device_list.nDeviceNum == 0:
        raise RuntimeError("No Hikrobot camera found")

    print("[INFO] Found cameras:", device_list.nDeviceNum)

    if CAMERA_INDEX >= device_list.nDeviceNum:
        raise RuntimeError("Invalid CAMERA_INDEX")

    device_info = ctypes.cast(
        device_list.pDeviceInfo[CAMERA_INDEX],
        ctypes.POINTER(MV_CC_DEVICE_INFO)
    ).contents

    created = False
    opened = False
    grabbing = False
    preview_enabled = False

    try:
        # 创建句柄
        ret = cam.MV_CC_CreateHandle(device_info)
        check_ret(ret, "CreateHandle")
        created = True

        # 打开相机
        ret = cam.MV_CC_OpenDevice()
        check_ret(ret, "OpenDevice")
        opened = True

        print("[INFO] Camera opened")

        # 配置触发模式
        configure_camera(cam)

        # 获取PayloadSize
        payload_size = get_payload_size(cam)

        print("[INFO] PayloadSize:", payload_size)

        # 分配SDK图像缓存
        data_buffer = (ctypes.c_ubyte * payload_size)()

        # 开始采集
        ret = cam.MV_CC_StartGrabbing()
        check_ret(ret, "StartGrabbing")
        grabbing = True

        print("-----------------------------------")
        print("[READY] Waiting for hardware trigger")
        print("[READY] Press Ctrl+C to stop")
        print("-----------------------------------")

        # 初始化预览窗口（无显示环境会自动降级关闭）
        if ENABLE_PREVIEW:
            try:
                cv2.namedWindow(
                    PREVIEW_WINDOW,
                    cv2.WINDOW_NORMAL
                )
                cv2.resizeWindow(PREVIEW_WINDOW, 960, 768)
                preview_enabled = True
                print(
                    "[INFO] Preview enabled. "
                    "Press 'q' or ESC in the preview window to quit."
                )
            except cv2.error as e:
                print(
                    "[WARN] 无法创建预览窗口，"
                    "可能无显示环境（DISPLAY 未设置），"
                    "已自动关闭预览:", e
                )
                preview_enabled = False

        frame_index = 0

        while True:

            frame_info = MV_FRAME_OUT_INFO_EX()

            ctypes.memset(
                ctypes.byref(frame_info),
                0,
                ctypes.sizeof(frame_info)
            )

            # 阻塞等待一帧
            ret = cam.MV_CC_GetOneFrameTimeout(
                data_buffer,
                payload_size,
                frame_info,
                GRAB_TIMEOUT_MS
            )

            if ret == 0:

                frame_index += 1

                image = save_image(
                    data_buffer,
                    frame_info,
                    frame_index
                )

                # 预览：每次触发采集后更新窗口
                if preview_enabled:
                    try:
                        cv2.imshow(PREVIEW_WINDOW, image)
                        key = cv2.waitKey(1) & 0xFF
                        if key in (ord("q"), 27):
                            print(
                                "\n[INFO] Quit key pressed "
                                "in preview window"
                            )
                            break
                    except cv2.error as e:
                        print(
                            "[WARN] 预览显示失败，关闭预览:", e
                        )
                        preview_enabled = False

            else:
                # 超时通常意味着当前没有收到触发帧
                # 不要把超时当作相机故障
                # 超时时也轮询一次窗口事件，保证按键响应
                if preview_enabled:
                    try:
                        key = cv2.waitKey(1) & 0xFF
                        if key in (ord("q"), 27):
                            print(
                                "\n[INFO] Quit key pressed "
                                "in preview window"
                            )
                            break
                    except cv2.error:
                        preview_enabled = False
                else:
                    time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[INFO] User stopped capture")

    finally:

        if grabbing:
            cam.MV_CC_StopGrabbing()

        if opened:
            cam.MV_CC_CloseDevice()

        if created:
            cam.MV_CC_DestroyHandle()

        if preview_enabled:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

        print("[INFO] Camera closed")


if __name__ == "__main__":
    main()
