# Hikrobot 相机启动说明（ROS 2 Humble）

相机型号：MV-CB013-A0UC-S（USB3 Vision，当前经 USB2 Hub 接入）
MVS SDK：`/opt/MVS`
运行环境：Ubuntu 22.04 + ROS 2 Humble + Python 3.10

---

## 1. 环境准备

每次新开终端，先加载 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
cd /home/ztl/PointPanoramaTex/hik_camera/src
```

SDK 路径、`MVCAM_*` 环境变量由 `hik_camera_common.py` 自动设置（自动区分 aarch64 / x86_64），无需手动 export。

ROS 2 没有 `roscore` / master，节点启动后通过 DDS 自动发现。脚本按源码方式直接 `python3` 运行，**不需要 `colcon build`**。
多机通信或需要隔离话题时，需在所有终端使用相同的 `ROS_DOMAIN_ID`（本机已在 `~/.bashrc` 中设为 13）：

```bash
export ROS_DOMAIN_ID=13
```

---

## 2. 文件说明

| 文件 | 作用 |
|---|---|
| `hik_camera_common.py` | 公共封装：SDK 初始化、开关相机、参数配置、取帧 |
| `hik_camera_publisher.py` | 实时采集并用 rclpy 发布 `sensor_msgs/Image` 到 ROS 2 话题 |
| `hik_camera_single_shot.py` | 只取一张图并保存 PNG（标定用，与 ROS 无关） |
| `hardware_trigger_capture.py` | 外部硬件触发连续采集并存 JPG（与 ROS 无关） |

> 同一台相机同一时间只能被一个进程打开。启动新程序前先 `Ctrl-C` 退出旧实例。

---

## 3. 功能一：ROS 2 实时推流

```bash
python3 hik_camera_publisher.py
```

默认参数（ROS 2 参数名）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `topic` | `/left_camera/image` | 发布话题 |
| `frame_id` | `left_camera` | 消息 frame_id |
| `camera_index` | `0` | 相机索引 |
| `trigger` | `true` | `true`=Line2 外部触发，`false`=free-run |
| `exposure_us` | `12000.0` | 曝光时间（微秒，浮点） |
| `pixel_format` | `BayerGB8` | 相机采集格式，可改 `BGR8Packed` |
| `timeout_ms` | `1000` | 取帧阻塞超时 |
| `log_every` | `50` | 每发布多少帧打印一次日志，`0` 关闭 |

常用启动方式：

```bash
# 默认 Line2 硬件触发，发布到 /left_camera/image
python3 hik_camera_publisher.py

# 改为 free-run（不需要触发信号）
python3 hik_camera_publisher.py --ros-args -p trigger:=false

# 如需 BGR8 直出（USB2 下约 6 fps）
python3 hik_camera_publisher.py --ros-args -p pixel_format:=BGR8Packed

# 换话题 / frame_id
python3 hik_camera_publisher.py --ros-args -p topic:=/hik/image -p frame_id:=hik_camera

# 调整曝光（浮点，注意写成 8000.0）
python3 hik_camera_publisher.py --ros-args -p exposure_us:=8000.0
```

消息格式：`sensor_msgs/Image`，`1280x1024`，`encoding=bgr8`，`step=3840`。
发布端 QoS 用 reliable（depth 5），`ros2 bag record` 默认即可录制；`ros2 topic echo/hz` 用更弱的 best effort，也能匹配。

验证：

```bash
ros2 topic hz /left_camera/image        # BayerGB8：free-run 约 15 Hz，外触发约 10 Hz
ros2 topic info -v /left_camera/image   # 发布者应为 /hik_camera_publisher
ros2 node info /hik_camera_publisher    # 查看节点发布的 topic
```

> `ros2 topic hz` 是 Python 订阅端，单帧 3.93 MB 时来不及处理会丢包、低估频率（外触发 10 Hz 时 hz 只有 6~7 Hz，free-run 30 fps 时约 10~12 Hz）。实际帧率以发布端日志“已发布 N 帧”为准。

### 关于频率（重要）

相机采集格式直接决定 USB2 带宽占用：

| 采集格式 | 单帧数据量 | 实测帧率 |
|---|---|---|
| `BGR8Packed` | 3.93 MB | 约 6 fps（USB2 带宽饱和） |
| `BayerGB8`（默认） | 1.31 MB | 约 15 fps；外部触发可稳定 10 fps |

- 默认 `BayerGB8`，主机端用 `cv2.cvtColor` 转成 `bgr8` 后发布；Hik 的 `BayerGB8` 对应 OpenCV 的 `COLOR_BayerGR2BGR`（实测相位差最小）。
- MVS 客户端能显示 10 Hz，是因为它同样以 Bayer 格式传输、在主机端解码；程序里若强制 `BGR8Packed`，就会掉到约 6 fps。
- 若改回 `pixel_format:=BGR8Packed`，USB2 下上限仍约 6 fps；要保持 10 Hz 就用默认的 Bayer 采集，或改用 USB3 直连。

---

## 4. 功能二：单张标定图

```bash
python3 hik_camera_single_shot.py
# [SAVED] /home/ztl/PointPanoramaTex/hik_camera/src/images/calib_20261110_112909.png  1280x1024
```

参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `-o, --output` | `./images/calib_<时间戳>.png` | 输出路径 |
| `--warmup` | `3` | 丢弃的预热帧数 |
| `--exposure-us` | `12000.0` | 曝光时间（微秒） |
| `--pixel-format` | `BGR8Packed` | 相机采集格式 |
| `--trigger` | 关闭 | 使用 Line2 硬件触发后再取一帧 |
| `--camera-index` | `0` | 相机索引 |

示例：

```bash
# 指定保存路径
python3 hik_camera_single_shot.py -o ~/calib/board1.png

# 延长曝光 + 多丢几帧预热
python3 hik_camera_single_shot.py --exposure-us 8000 --warmup 5

# 等硬件触发再拍
python3 hik_camera_single_shot.py --trigger -o ~/calib/board_trigger.png
```

保存格式为 **PNG（无损）**，适合标定使用。程序取完一张即自动释放相机退出。该脚本不使用 ROS。

---

## 5. 功能三：硬件触发连续存图

```bash
python3 hardware_trigger_capture.py
```

行为：

- 触发源：`Line2` 上升沿，`TriggerSelector=FrameBurstStart`
- 每来一个触发保存一张 JPG 到 `./images/`（相对于当前工作目录）
- 同时追加时间戳记录到 `./images/timestamps.jsonl`
- 终端打印 `[READY] Waiting for hardware trigger` 后即等待触发
- `Ctrl-C` 正常退出并释放相机

可调常量在文件顶部：`SAVE_DIR`、`TRIGGER_LINE`、`TRIGGER_EDGE`、`GRAB_TIMEOUT_MS`、`JPEG_QUALITY`。该脚本不使用 ROS。

---

## 6. 常见问题

| 现象 | 原因与处理 |
|---|---|
| `OpenDevice failed 0x80000203` | 相机已被占用。用 `pgrep -af hik_camera` 找到旧进程并 `kill -INT <pid>`，或在其终端 `Ctrl-C` |
| `error -71` / `Cannot enable. Maybe the USB cable is bad?` | USB3 线缆或接口信号问题。换合格 USB3 短线、直连板载 USB3 口，不要走 Hub |
| 发布频率只有 5~6 Hz | 多半是 `pixel_format` 用了 `BGR8Packed`（3 字节/像素，USB2 带宽饱和）。保持默认 `BayerGB8`，详见第 3 节 |
| 停在 `[READY]` 但没有 `[SAVED]` | Line2 没有触发脉冲，检查触发源是否上电/接线是否正确 |
| `MV_E_GC_ACCESS 0x80000106` | `Line2` 的 `LineMode` 是只读节点，代码已自动跳过写入 |
| `Set TriggerSelector=FrameStart failed 0x80000004` | 该型号只支持 `FrameBurstStart`，代码已适配 |
| `ros2 topic hz` 有频率但 `echo` 收不到 | 检查是否都 `source /opt/ros/humble/setup.bash`、`ROS_DOMAIN_ID` 是否一致；发布端为 best effort，订阅端不要指定 reliable |
| 节点重启后相机打不开 | 确认上一个实例已完全退出：`pgrep -af "^python3 hik_camera"` |

---

## 7. 快速检查命令

```bash
# 相机是否枚举
lsusb -d 2bdf:0001

# USB 链路速度（480 = USB2，5000 = USB3）
for d in /sys/bus/usb/devices/*; do
  if [ -f "$d/idVendor" ] && [ "$(cat "$d/idVendor")" = "2bdf" ]; then
    echo "$d speed=$(cat "$d/speed")M"
  fi
done

# 是否有相机进程在运行
pgrep -af "^python3 hik_camera"

# ROS 2 话题频率与消息字段
ros2 topic hz /left_camera/image
ros2 topic echo --once --field encoding /left_camera/image
ros2 topic echo --once --field step /left_camera/image

# 录制图像话题（示例）
ros2 bag record -o hik_image_bag /left_camera/image
```
