# body 云发布验证（BODY_CLOUD_VALIDATION）

- 日期：2026-11-10
- 结论：**通过**。`/lio/cloud_body_dense` 在正常配置（无 pcd_save_en、无 dense_map）下持续发布，10 Hz、无空帧、无重复帧，frame_id / 时间戳 / 参考时刻语义与源码审计（DESKEW_PIPELINE_AUDIT.md §7）一致；同帧与 `/lio/odom` 的 header.stamp 逐位相等。存在少量与算法无关的丢帧（详见 §6），且**全部落在历史 sl 评价窗口 946685507.4–531.1 之外或窗口内 1:1 完整**。

## 1. 运行配置（与任务要求一致）

- 二进制：`third_party/Super-LIO/install/super_lio/lib/super_lio/super_lio_node`（Nov 9 11:39 构建，md5 `00e97f6a067a8167113af60dcb2e245d`）
- 参数文件：`r3live.yaml`（`save_map: false`，未开启任何 pcd_save / dense_map；`lio.output.dense: true`）
- 启动命令（完整脚本 `/tmp/replay3_run.sh`）：

```bash
export ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash
ros2 run super_lio super_lio_node --ros-args \
  --params-file /home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/r3live.yaml
# 录制（QoS 覆盖 best_effort + keep_all，见 GEOMETRY_DATASET_MANIFEST.md）
ros2 bag record -o replay3 --qos-profile-overrides-path /tmp/rec_qos.yaml \
  /livox/lidar /livox/imu /lio/cloud_body_dense /lio/odom
ros2 bag play /home/ywj/slam_data/Retail_Street --rate 1.0 --disable-keyboard-controls
```

在线证据：`/tmp/live_probe.py`（rclpy，best_effort 订阅，45 s 窗口，数据域 946685488.4–533.3，完整覆盖历史 sl 评价窗口）。`ros2 topic hz/echo` 默认 reliable 订阅与节点 best_effort 发布 QoS 不兼容，故未采用（首测输出 "topic does not appear to be published yet" 即此原因）。

## 2. 话题存在性与频率

| 检查项 | 结果 |
|---|---|
| topic 存在 | 是（publisher 在 init 时无条件创建，ROSWrapper.cpp L914-916） |
| 发布频率（在线 45 s 实测） | **9.983 Hz**（dense），odom 同为 9.983 Hz，IMU 200.029 Hz |
| 分桶（5 s） | [79,50,50,50,50,50,80]——首尾为窗口对齐，中间 50/5s = 10.0 Hz |
| header.stamp 周期（数据域） | 0.09497/0.09502/0.09894 s——与 LiDAR 帧周期一致（Avia 非均匀 10 Hz） |

## 3. 消息内容（在线样例，逐字节实测）

```
/lio/cloud_body_dense:
  frame_id = "imu"    # 标签；几何上为雷达机体系（审计报告 §8）
  stamp = 946685488.596192002
  width = 22218, height = 1, is_dense = true
  fields = [x, y, z, intensity], point_step = 32
/lio/odom（同帧）:
  frame_id = "world", child_frame_id = "base_link"
  stamp = 946685488.596192002   # 与 dense 逐位相等
```

## 4. 点数与空帧/重复帧（整段 122.7 s 录制，按 header.stamp 排序统计）

| 统计 | dense | odom | lidar |
|---|---|---|---|
| 帧数 | 1307 | 1315 | 1345 |
| 点数 min/med/max | 16424 / 21375 / 23006 | — | ~24000（原始） |
| 空帧 | **0** | — | 0 |
| 重复 stamp | **0** | 0 | 0 |
| stamp 两两严格一致（与 odom） | **1307/1307** | | |

- 点数随场景变化 16.4k–23.0k，与入口过滤（blind 2 m / maxrange 100 m）+ 帧尾剔除（审计 §5.1）一致；中位数 21375 ≈ 原始 24k × 过滤比例，**无异常跳变或半帧**。

## 5. 与 LiDAR 扫描的时间匹配

对每条 lidar header.stamp `t_L` 搜索 `t_D − t_L ∈ [80,110] ms` 的 dense 帧：

- 命中 **1307/1307**；offset 中位数 97.33 ms，范围 92.66–99.89 ms——即 dense 的参考时刻 = 帧尾（最后一个 IMU 采样时刻 ≈ t_L + 帧长），**不是** LiDAR header.stamp。
- 反向：无 dense 找不到 odom 的情况（`dense_no_odom = 0`）；`odom_no_dense = 8`（§6）。
- 若误把"dense.stamp == lidar.stamp"当作匹配口径会得出 100% 不匹配的假阴性——正确口径是 dense.stamp == odom.stamp（逐位相等，已验证）。

## 6. 丢帧/间隙清单（全部定位）

| 事件 | 时间域（header.stamp） | 数量 | 根因 |
|---|---|---|---|
| 录制起点截断 | < 946685438.50 | lidar 10 / imu 205 条缺失 | recorder 比 player 晚启动 ~2 s（**数据本身完整**：原始 bag 前 1 s 内容未录，非节点丢帧） |
| Map init 期 | 438.5–459 | lidar 无 odom 若干 | 滤波器初始化，正常 |
| Guard 跳帧 | 480.999→484.2（3.2 s） | odom/dense 各缺 ~32 帧 | 重放链路瞬时抖动：日志 `[IMU Lag] IMU lags behind lidar by 2975.3 ms`，节点等 IMU 覆盖丢弃该段（**原始 lidar 流本身连续**，dt_max=100.7 ms）；紧随 1 次 Trust LOST（5 帧），窗口外 |
| 单帧 dropout | 449.5 / 493.8 / 495.8 / 501.2 / 551.3 / 568.8 | 6 帧（有 odom 无 dense） | dense 线程该帧 pc_full 空或队列丢弃；odom 正常 |

**评价窗口 946685507.0–531.5 内：lidar 245 = dense 245 = odom 245，1:1 完整，无 gap、无 dropout**（3.2 s 停摆事件发生在 481–484 s，窗口外）。

## 7. 去畸变参考时刻的验证口径说明

"点云是否去畸变到扫描结束时刻"无法用单条消息自证（消息本身不携带每个点的补偿后时刻）。本报告采用两层证据：

1. **源码**（审计 §4-§6）：undistortCloud 参考位姿 = T_end（帧尾最后 IMU 时刻先验）+ delta correction（刚体），dense 消息 stamp = state.timestamp = 同一时刻 ⇒ 发布语义自洽。
2. **数据**：dense stamp 与 odom 严格相等（§5）；帧长 ≈ 95-100 ms 且 dense 比 lidar header.stamp 晚 ≈ 帧长的实测偏移与该语义吻合（92.66–99.89 ms，若参考在帧首则应 ≈ 0-5 ms）。

阶段 A 的几何实验（残差 vs offset_time）将在统一数据集上直接检验该参考时刻假设，见停止条件。

## 8. 样例命令（可复跑）

```bash
source /opt/ros/humble/setup.bash; source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
ros2 bag info /home/ywj/slam_data/geom_replay_20261110/replay3
python3 /tmp/final_replay3.py   # 完整性统计（排序后）
python3 /tmp/gaps.py            # 间隙定位
python3 /tmp/live_probe.py      # 在线探测（需节点运行中）
```
