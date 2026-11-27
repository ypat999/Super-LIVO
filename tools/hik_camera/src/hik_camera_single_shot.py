#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hikrobot 相机单张采集（用于标定），保存无损 PNG"""

import argparse
import os
import time
from datetime import datetime

import cv2

import hik_camera_common as hik


def parse_args():
    parser = argparse.ArgumentParser(
        description="Hikrobot 相机单张采集（标定用）"
    )
    parser.add_argument(
        "-o", "--output",
        default=None,
        help="输出图片路径，默认 ./images/calib_<时间戳>.png",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=3,
        help="丢弃的预热帧数，默认 3",
    )
    parser.add_argument(
        "--exposure-us",
        type=float,
        default=12000.0,
        help="曝光时间（微秒），默认 5000",
    )
    parser.add_argument(
        "--pixel-format",
        default="BGR8Packed",
        help="相机采集像素格式，默认 BGR8Packed（可选 BayerGB8 降低带宽）",
    )
    parser.add_argument(
        "--trigger",
        action="store_true",
        help="使用 Line2 硬件触发（默认 free-run）",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="相机索引，默认 0",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    output = args.output
    if not output:
        output = os.path.expanduser(
            "./images/calib_{}.png".format(
                datetime.now().strftime("%Y%m%d_%H%M%S")
            )
        )

    output = os.path.abspath(output)
    output_dir = os.path.dirname(output)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    cam = hik.open_camera(args.camera_index)
    try:
        hik.configure_camera(
            cam,
            trigger=args.trigger,
            exposure_us=args.exposure_us,
            pixel_format=args.pixel_format,
        )
        buffer, payload_size = hik.start_grabbing(cam)

        # 丢弃预热帧，等待曝光稳定
        for _ in range(max(0, args.warmup)):
            hik.grab_frame(
                cam, buffer, payload_size, 2000, args.pixel_format
            )

        image = None
        deadline = time.time() + 5.0
        while image is None and time.time() < deadline:
            image = hik.grab_frame(
                cam, buffer, payload_size, 1000, args.pixel_format
            )

        if image is None:
            raise RuntimeError("5 秒内未取到图像，请检查触发信号或相机状态")

        if not cv2.imwrite(output, image):
            raise RuntimeError("保存图片失败: " + output)

        print(
            "[SAVED] {}  {}x{}".format(
                output,
                image.shape[1],
                image.shape[0],
            )
        )
    finally:
        hik.close_camera(cam)


if __name__ == "__main__":
    main()
