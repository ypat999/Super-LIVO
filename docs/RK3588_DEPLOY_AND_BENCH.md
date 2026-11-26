# RK3588 本机部署与测试记录（Super-LIVO 方案 A）

> 记录时间：2026-11-24
> 机器：4× Cortex-A55 @1.99 GHz，Ubuntu 22.04 + ROS 2 Humble（aarch64）
> 背景：从原机器（用户 ywj，x86/WSL）换到本机（用户 ztl，RK 平台），只做方案 A，不做 B。

## 1. 代码与数据位置

| 用途 | 路径 |
|---|---|
| 主仓库 | `/home/ztl/git/sl_ws/Super-LIVO` |
| LIO 核（第三方） | `third_party/Super-LIO`，安装到 `third_party/Super-LIO/install` |
| 方案 A 可执行 | `install/super_livo_visual_a/lib/super_livo_visual_a/visual_a_app` |
| 雷达驱动 | `/home/ztl/PointPanoramaTex/livox_driver2` |
| 相机发布端 | `/home/ztl/PointPanoramaTex/hik_camera`（纯 Python，无需 colcon build） |
| 测试 bag | `/home/ztl/PointPanoramaTex/lidar_cam_extr_calib/FAST-Calib/calib_data/test4_move` |
| 输出根 | `/home/ztl/slam_data/output` |

## 2. 构建（已完成）

```bash
source /opt/ros/humble/setup.bash
cd /home/ztl/git/sl_ws/Super-LIVO

# 1) LIO 核（实测 893.9 s）
(cd third_party/Super-LIO && \
  colcon build --packages-select super_lio --cmake-args -DCMAKE_BUILD_TYPE=Release)

# 2) 公共库 + 着色 + 方案 A/B（实测 5 min 35 s）
source third_party/Super-LIO/install/setup.bash
colcon build --packages-select super_livo_common pointcloud_colorizer \
  super_livo_visual_a super_livo_visual_b --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- **必须 `-DCMAKE_BUILD_TYPE=Release`**：Debug 下着色耗时从 4~5 ms 劣化到几十 ms。
- 校验动态库：`ldd install/super_livo_visual_a/lib/super_livo_visual_a/visual_a_app | grep "not found"`
  为空才算通过；检查前必须先 `source /home/ztl/PointPanoramaTex/livox_driver2/install/setup.bash`（提供 `liblivox`）。

## 3. 关键配置

### 3.1 LIO 主干 `common/super_livo_common/config/lio_move.yaml`

- 话题：`/livox/lidar`（CustomMsg）、`/livox/imu`
- `qos_reliable: false`（bag 录制端是 RELIABLE，订阅端 best_effort 可兼容）
- `save_map: false`（地图由视觉支路存彩色图）
- `degeneracy_detect_en: true`（视觉门控依赖退化签名）
- `embed/inject: true`（进程内直连，不走 DDS）

### 3.2 方案 A `solution_a_lightweight/super_livo_visual_a/config/visual_a_move.yaml`

已按本机改过（未提交）：

- `calib.camera_intrinsic_yaml` → `/home/ztl/git/sl_ws/Super-LIVO/config/camera_intrinsic.yaml`
- `calib.fast_calib_result` → `/home/ztl/git/sl_ws/Super-LIVO/third_party/FAST-Calib-ROS2/output/calib_result.txt`
- `pcd.save_dir` → `/home/ztl/slam_data/output/move_a`
- `img_time_offset: -204701439.380`（见 3.3）
- `color.sync_max_offset: 0.1`
- `gate.max_reprojection_rmse: 6.0`（实测 rmse 中位 2.92 / p90 3.97 / max 4.62 px）
- `pcd.save_interval_frames: 100`（分片写 `colored_%05d.pcd`，退出时另写合并图）

参数命名约定：顶层 `/**` + `ros__parameters`，视觉段前缀 `super_livo_visual.*`，主干段扁平 `lio.*`。
命令行覆盖必须写全名，例如 `-p super_livo_visual.pcd.save_dir:=/xxx/pcd`。

### 3.3 双时钟域（最容易踩的坑）

| 数据 | header.stamp 时钟域 |
|---|---|
| LiDAR `CustomMsg` / IMU | 设备时钟（Livox 自 2020-01-01 起算，无 GNSS/PTP 时与真实 UTC 有偏移） |
| 海康图像 | 主机墙钟（`host_ts_ns`，2026） |

`img_time_offset` 的作用是**把图像时间拉回雷达时钟域**，参与两件事：

1. 图像-点云配对：`dt = |image.arrival_stamp + img_time_offset - t_cloud|`，落入 `color.sync_max_offset` 才算配对成功
2. 位姿查询时刻

它不是曝光延迟。本机 test4_move 实测设备→主机 epoch 差 **204701439.3799 s**（IMU 18113 条，stdev 7 ms），故取负值：

```yaml
img_time_offset: -204701439.380
color.sync_max_offset: 0.1
```

离线验证：826/829 帧入窗，中位 `|dt|` = 13.8 ms。
**这个值只对那段时间录的 bag 有效**。实时场景设备计时与主机时钟的关系会变，必须现场重标（见 §6.2）。

## 4. 回放测试方法

单实例编排脚本：`tools/run_a_replay_local.sh`（本记录同时保存于 `/tmp/run_a_move.sh`）

```bash
cd /home/ztl/git/sl_ws/Super-LIVO

# 0.5×（推荐，稳定）
OUT=/home/ztl/slam_data/output/move_a RATE=0.5 \
  setsid nice -n 0 bash tools/run_a_replay_local.sh > /tmp/run_a.log.out 2>&1 < /dev/null &

# 1.0×（容量测试）
OUT=/home/ztl/slam_data/output/move_a_rt1 RATE=1.0 \
  setsid nice -n 0 bash tools/run_a_replay_local.sh > /tmp/run_a_rt1.log.out 2>&1 < /dev/null &
```

脚本做的事：起 `visual_a_app` → 等 6 s → `ros2 bag play --rate $RATE` → 等 25 s 排空 →
`SIGINT`（触发 `shutdownAndSave()`）→ 等退出 → `ALL DONE`。

**不要 `kill -9`**，会丢最终合并彩色地图。**不要并行跑两轮**，会抢 DDS 共享内存与 CPU。

### 4.1 结果怎么看

```bash
OUT=/home/ztl/slam_data/output/move_a_rt1
tail -40 $OUT/run_a.log
grep -a '彩色统计' $OUT/run_a.log          # 总计行
grep -a 'Img->VisOdom' $OUT/run_a.log      # 视觉端到端延迟
grep -a '积压丢帧累计' $OUT/run_a.log | tail -3
grep -a 'Watchdog' $OUT/run_a.log
cat $OUT/load.log                          # 系统 load / app CPU 曲线
ls -l $OUT/pcd/                            # colored_*.pcd + 合并图
```

健康标准：`实际着色帧 / 总帧数` 高、`输入丢弃=0`、`积压丢帧累计` 小、无 `Watchdog`。

## 5. 已完成的测试与结果

同一份 bag：test4_move（829 帧图像，10 Hz 雷达）。

| 轮次 | OUT 目录 | RMW | 后台 Trae | 着色帧 | 地图点数 | odom 积压丢帧 | 输入丢弃 | load |
|---|---|---|---|---|---|---|---|---|
| 0.5× 基线 | `output/move_a` | FastRTPS | 正常 | **791 / 829** | 3,535,103 | 少量（告警 1 次） | 0 | — |
| 1× 基线 | `output/move_a_rt1` | FastRTPS | 正常 | 664 | 2,928,487 | 1601 | 132 | 6.17 |
| 1× + renice | `output/move_a_rt1_renice` | FastRTPS | nice 19 | **735** | 3,250,533 | **1001** | **0** | 5.58 |

说明：

- 0.5× 产物：`pcd/super_livo_visual_a_colored.pcd` 56.5 MB，8 个分片，颜色覆盖率 100%，
  全景 extent 19.28 × 14.3 × 3.82 m；视觉单帧耗时 p50 88 / p90 127 / max 204 ms；
  退化 151 帧全 `sev=0.00`（mode0=70、mode3=81，`w=0` 属健康门控设计）。
- 1× 基线：`[Img->VisOdom] avg 43,970 ms`、`Watchdog` 3 次，视觉线程严重滞后。
  统计行：`调用=1585 实际着色帧=664 丢弃(无图像)=122 丢弃(等位姿超时)=10 未完成=1 地图点数=2928487 输入丢弃=132`。
- 1× + renice：`调用=1596 实际着色帧=735 丢弃(无图像)=65 丢弃(等位姿超时)=0 未完成=1 地图点数=3250533 输入丢弃=0`；
  app CPU 恒定 ~210%（约 2.1 核）；`Watchdog` 1 次；drain 阶段仍有积压（末段 `Img->VisOdom avg 55 s`）。
- 两轮"调用"数不同（1585 / 1596）是因为 1× 下统计窗口随滞后漂移，比较请用**着色帧**与**地图点数**。

**结论**：本机 4× A55 撑不住 1× 全速实时；0.5×（等效 5 Hz）稳定可用。
把后台 Trae 降到 nice 19 即可把着色帧从 664 提到 735（+10.7%）、odom 丢帧降 37%，
说明后台占用确实在抢算力，但仍差约 10%。

## 6. 实时链路（Livox MID360 + 海康相机）

### 6.1 硬件实测（2026-11-24 当前在线）

| 项 | 实测结果 |
|---|---|
| 雷达连通 | `ping 192.168.144.201` 0% 丢包，rtt avg 1.5 ms |
| 点云推流 | 绑定 UDP 56301，3 s 收 601 包 / 829,380 B（约 200 pkt/s，1380 B/包） |
| IMU 推流 | 绑定 UDP 56401，3 s 收 601 包（200 Hz） |
| 主机网口 | `eth0 UP 192.168.144.100/24` |
| 相机 | `Bus 006 Device 002: ID 2bdf:0001 Hikrobot MV-CB013-A0UC-S`（USB3 root hub） |
| MVS SDK | `/opt/MVS` 完整（含 aarch64 样例） |
| 驱动配置 | `livox_driver2/install/.../config/MID360_config.json` 已是本机 `192.168.144.100` / 雷达 `192.168.144.201`，`msg_MID360_launch.py` 默认加载它 |

相机发布参数（`hik_camera/src/hik_camera_publisher.py` 默认值）：话题 `/left_camera/image`、
Line2 硬件触发、`exposure_us=12000`、采集 `BayerGB8`、发布编码 `bgr8` 1280×1024、
QoS RELIABLE depth 5、`header.stamp` 用 SDK 的 `host_ts_ns`（墙钟）。

### 6.2 实时跑之前必须先改的 4 处

1. **统一 `ROS_DOMAIN_ID`**：驱动脚本 `livox_driver2/start.sh` 和 `hik_camera/start_publisher.sh` 用 13，
   方案 A 用 93。建议实时统一为 93（或 13，只要一致）。
2. **去掉 `taskset -c 1`**：`livox_driver2/start.sh` 把驱动绑在单核，4 核机器上会和 LIO 抢核。
   手起即可：`ros2 launch livox_ros_driver2 msg_MID360_launch.py`。
3. **`img_time_offset` 已改为在线自动标定**（2026-11-24）：`onDenseCloud` 新增
   `receive_time`（该帧到达宿主 ROS 回调的墙钟），用 `offset = stamp - receive_time`
   前 20 帧取中位数收敛，再以 EMA(0.02) 跟踪漂移，直接覆盖参数初值。
   实测偏移约 `-205231095.9 s`，且以约 1.3 ms/s 漂移——手工填一次只能撑几分钟。
   参数 `img_time_offset` 现在只是初值/回放 bag 用。
4. **不要再用「降雷达频率」保吞吐**：真正的瓶颈不是雷达帧率（见 6.4）。

### 6.3 参考启动顺序（未在本机完整验证）

```bash
# 终端 1：雷达驱动
export ROS_DOMAIN_ID=93
source /home/ztl/PointPanoramaTex/livox_driver2/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端 2：相机（SDK 环境按 hik_camera/README.md）
export ROS_DOMAIN_ID=93
cd /home/ztl/PointPanoramaTex/hik_camera && python3 src/hik_camera_publisher.py

# 终端 3：方案 A（不带 bag play）
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash
source /home/ztl/git/sl_ws/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /home/ztl/PointPanoramaTex/livox_driver2/install/setup.bash
source /home/ztl/git/sl_ws/Super-LIVO/install/setup.bash
/home/ztl/git/sl_ws/Super-LIVO/install/super_livo_visual_a/lib/super_livo_visual_a/visual_a_app \
  --ros-args \
  --params-file /home/ztl/git/sl_ws/Super-LIVO/common/super_livo_common/config/lio_move.yaml \
  --params-file /home/ztl/git/sl_ws/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_move.yaml \
  -p super_livo_visual.timing:=true \
  -p super_livo_visual.img_time_offset:=<-实测值> \
  -p super_livo_visual.pcd.save_dir:=/home/ztl/slam_data/output/realtime
```

跑完用 `Ctrl-C`（SIGINT）触发落盘。

### 6.4 实时丢帧根因与修复（2026-11-24，真机开灯）

**现象**：`实际着色帧` 只有 60~90，`丢弃(无图像)` 300+，`pending_clouds_` 溢出，
且随运行时间越来越差；单看图像/雷达各自都正常（图像 9~10 Hz，`[LidarIn] drift≈0`）。

**定位过程**：
1. 在 `livoxHandler` 与 `onDenseCloud` 各加一条节流日志，量化「到达时刻 − 时间戳」。
   雷达输入 `drift` 全程 ±50 ms（驱动/DDS 正常）；dense 输出 `lag` 从 0 单调涨到 10 s，
   且时间戳严格连续（0.1 s 步进、无跳变）⇒ **没有任何一帧被丢弃，只是越处理越落后**。
2. `livoxHandler` 只把帧塞进 `lidar_buffer_`（当时无上限），`process()` 每次取最旧帧。
   LIO 单帧约 141 ms > 雷达 100 ms 帧间隔 ⇒ 缓冲无界增长，dense 云时间戳相对图像越来越旧，
   配对窗口（0.1 s）必然全部落空。

**修复**（`ROSWrapper::sync_measure`）：新增 `lio.ros.max_lidar_backlog`（默认 2，0=不限制）。
积压超过上限时丢弃最旧帧，把输出时间戳拉回当前时刻；丢帧不影响 IMU 区间消费。
副作用：被丢帧造成的 200 ms 间隔会触发 `[Guard] skip untrusted frame`，LIO 自身仍约 7 Hz。

**配套**：`filter_rate 3→5`（LIO 配准点数 6.6k→4k）可把 LIO 从 7.4 Hz 提到 9.6 Hz，
`Guard skip` 从 ~50 次/70 s 降到 ~5 次；`visual.decimate` 是着色吞吐的主杠杆
（VIO 单帧 ~100–180 ms，占满 worker 线程会挤掉点云着色事件）。

**120 s 真机对比**（`filter_rate=5`，`qos_depth=10`，`sync=0.1`，自动标定）：

| decimate | 调用 | 实际着色帧 | 无图像 | 排队超时 | 地图点数 |
|---|---|---|---|---|---|
| 2（VIO 5 Hz） | 1006 | **533**（4.4 Hz） | 71 | 718 | 2.38 M |
| 4（VIO 2.5 Hz） | 1150 | **816**（6.8 Hz） | 108 | 204 | 3.63 M |

**推荐实时命令**（在 6.3 基础上加两项）：

```bash
-p lio.ros.max_lidar_backlog:=2 \
-p lio.sensor.filter_rate:=5 \
-p super_livo_visual.visual.decimate:=4
```

诊断日志：`[LidarIn] drift`（雷达输入是否积压）、`[DenseLag] 标定后残差 / LIO延迟`
（残差应≈LIO 处理延迟 0.1~0.2 s，持续增大说明又退化成 FIFO 积压）、
`[ClockCalib]`（自动标定结果）、`[Lidar Drop]`（积压丢帧累计）。

## 7. 待做：CycloneDDS 对比（第 2/3 轮）

```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

- 第 2 轮（renice 生效时）：`OUT=/home/ztl/slam_data/output/move_a_rt1_cyc_renice RATE=1.0 RMW=rmw_cyclonedds_cpp ...`
- 第 3 轮（恢复 nice 0）：`OUT=/home/ztl/slam_data/output/move_a_rt1_cyc RATE=1.0 RMW=rmw_cyclonedds_cpp ...`

预期：本机 FastRTPS 已在用共享内存传输（`rmw-fastrtps-shared-cpp`），CycloneDDS 默认走 UDP loopback，
大图像消息要拆包，理论上**未必更好，甚至更差**。之所以测，是因为本机已证明算力竞争影响显著，
需要把「DDS 实现」与「CPU 竞争」两个变量分开。

对照关系：

| 轮次 | RMW | 后台 Trae |
|---|---|---|
| 1（已完成） | FastRTPS | nice 19 |
| 2（待做） | CycloneDDS | nice 19 |
| 3（待做） | CycloneDDS | nice 0 |

## 8. 常见坑

1. **测完一定要 SIGINT**，`shutdownAndSave()` 才写最终合并彩色地图。
2. **单实例**：脚本并发跑两轮会共用日志、抢 `/dev/shm/fastrtps_*` 与 CPU，结果全废。
   清理：`pkill -INT -f visual_a_app` 后确认 `pgrep -a visual_a_app` 为空。
3. **后台进程**：沙箱/终端退出会带走后台 app，必须 `setsid ... < /dev/null &` 启动。
4. **`taskset -c 1`**：驱动脚本默认绑核，实时跑前必须去掉。
5. **nice 继承**：脚本里 app 与 bag play 已显式 `nice -n 0`，否则从低优先级 shell 启动会整体降级。
6. **参数前缀**：`-p` 必须写全名 `super_livo_visual.xxx`，写错会被静默忽略（无任何告警）。
7. **ros2 bag info -v** 在 Humble 不支持；解析 CDR 时间戳可用 `struct.unpack_from("<iI", data, 4)`
   读 `header.stamp`，比 rclpy 反序列化快两个数量级。
8. **日志噪声**：`crashpad` / `prctl` 行可忽略，过滤用 `grep -avE 'crashpad|prctl'`。

## 9. 环境恢复

本轮实验曾把 Trae 后台降到 nice 19（`ckg_server`、`ai-agent`、`node`），
关掉 Trae 后这些进程消失，无需处理。若只是临时恢复：

```bash
for n in ckg_server ai-agent node; do pgrep -x $n | xargs -r renice 0 -p; done
```

另外 `htop` 常驻时约占 4% CPU，测容量前建议退出。
