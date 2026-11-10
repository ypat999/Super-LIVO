# 阶段 A/B 统一数据集清单（GEOMETRY_DATASET_MANIFEST）

- 日期：2026-11-10
- 数据集：`/home/ywj/slam_data/geom_replay_20261110/replay3/`（`metadata.yaml` + `replay3_0.db3`，rosbag2 sqlite3，CDR）
- 性质：同一原始 bag 重放、同一次 Super-LIO 运行产生的 4 话题同窗数据集。前置验证（任务 1/2/3）全部通过后录制。

## 1. 话题与覆盖

| Topic | 类型 | 帧数 | header.stamp 窗口（数据域） |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 1345 | 946685438.4996 – 946685572.8996 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 27242 | 同上（200 Hz，dt med 5.0 ms） |
| `/lio/cloud_body_dense` | `sensor_msgs/msg/PointCloud2` | 1307 | 946685438.4951 – 946685572.9982 |
| `/lio/odom` | `nav_msgs/msg/Odometry` | 1315 | 同上 |

- **评价窗口（阶段 B 留出帧 / 历史 sl 对齐窗口）946685507.0–531.5：lidar 245 = dense 245 = odom 245，1:1 完整、无 gap、无 dropout、无重复 stamp、无空帧**。
- 全 bag 完整窗口建议从 **946685459.0**（Map init done 后）起用；481.0–484.2 为录制内停摆段（§4）。

## 2. 时间戳语义（重要，引用审计结论）

1. 一切对齐、匹配、误差计算**以 header.stamp 为准**（Livox 本地时钟域 9.46685×10⁸ s）。rosbag 行级 sqlite timestamp 是重放时刻的墙钟，**不得**用作数据时间（与 header 差 ~8.5×10⁸ s，仅影响排序读取顺序，见 BODY_CLOUD_VALIDATION.md）。
2. `/livox/lidar`.header.stamp = 扫描开始；每点真实采样时刻 = `header.stamp + points[i].offset_time × 1e-9`（**原始逐点时间字段完整保留**，未经任何处理链修改；timebase 未被 Super-LIO 使用）。
3. `/lio/cloud_body_dense` 与 `/lio/odom`：header.stamp = **本帧最后一个 IMU 采样时刻**（≈ lidar.stamp + 95–100 ms），同帧两者逐位相等（实测 1307/1307）。
4. dense 点坐标 = 帧尾参考时刻的**校正后机体系**（frame_id 标签 "imu"，几何为雷达系；含 delta correction 刚体项，帧内形状 = 纯去畸变结果）。

## 3. 运行可复现凭据

| 项 | 值 |
|---|---|
| Super-LIVO commit | `1f37053260302f85f9014498fbee76216e59c5d5` |
| Super-LIO commit（工作树） | `67c0acd4676fa4e1ea0cd00d5486b3092c41317b` |
| 二进制 | `third_party/Super-LIO/build/super_lio/super_lio_node`，Nov 9 11:39 构建，md5 `00e97f6a067a8167113af60dcb2e245d` |
| 参数文件 | `r3live.yaml`，sha256 `632e5721bd41f74431de273009c4ddf92744e8ab27d436811d93b31d64e1d2a1` |
| 输入 bag | `/home/ywj/slam_data/Retail_Street`（原始未改） |
| 启动 | `ros2 run super_lio super_lio_node --ros-args --params-file .../r3live.yaml`（无 override；env `ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4`） |
| 录制 QoS 覆盖 | `/tmp/rec_qos.yaml`：4 话题 `reliability: best_effort, history: keep_all, depth: 10000`（与发布端匹配，消除 recorder 队列丢包） |
| 重放 | `ros2 bag play /home/ywj/slam_data/Retail_Street --rate 1.0 --disable-keyboard-controls` |

## 4. 已知限制（如实声明）

| # | 限制 | 影响 |
|---|---|---|
| 1 | 头部截断：录制比播放晚 ~1.0 s，缺原始 bag 前 10 帧 lidar / 205 条 imu（437.5–438.5 s，init 前） | 无（窗口外） |
| 2 | 停摆段 946685481.0–484.2：`[IMU Lag] 2975 ms` 导致该段 ~32 帧无 odom/dense（原始流完整）；紧随 5 帧 Trust LOST | 阶段 B 需排除该段（458–507 s 之间，本就不在历史窗口） |
| 3 | 6 个单帧 dropout（449.5/493.8/495.8/501.2/551.3/568.8 s，有 odom 无 dense）；评价窗口内为 0 | 无 |
| 4 | run-to-run 复现噪声 ~0.5 cm/s（窗口末端 13 cm/25 s），见 REPLAY_REPRODUCIBILITY.md | 阶段 B 位姿误差解释的下限参考 |
| 5 | 重放二进制 Nov 9 与历史 Nov 6 不同版本（差异分析见同报告 §2） | 与历史 sl 直接对比时的未排除变量 |

## 5. 数据使用规约（本阶段约定）

- 不抽取跨源不同步帧：dense/odom 用 stamp 逐位配对；原始点时间用 lidar header + offset_time 重建；与 IMU 对齐用 sync_measure 同口径（IMU 时刻 < lidar.end_time）。
- 不做颜色处理、不做体素融合、不做伪 GT（按任务边界）。
- 配套产物（同目录）：`live_probe_out.json`（在线 45 s 证据）、`cmp_curve3.csv` / `cmp_dp3.csv`（历史 vs 重放误差曲线）、`replay_full`（首次全窗 bag，lidar 完整但 IMU 有丢包，仅作对照不用于几何实验）。

## 6. 停止条件核对

| 停止条件 | 状态 | 证据 |
|---|---|---|
| 1. body 云持续发布，参考时刻/坐标系明确 | ✅ | BODY_CLOUD_VALIDATION.md；DESKEW_PIPELINE_AUDIT.md §6-§7 |
| 2. 历史 sl odom 可合理复现 | ✅ | REPLAY_REPRODUCIBILITY.md §5 |
| 3. 逐点时间与去畸变坐标链明确 | ✅ | DESKEW_PIPELINE_AUDIT.md §1-§10 |

三项全部满足，阶段 A/B 正式几何实验的数据前提成立。本报告交付后按约定等待审查。
