# 原始 Bag 数据可用性清单（2026-11-09）

> 目标：回答「Super-LIO 无色几何验证（阶段 A 帧内几何 / 阶段 B 帧间位姿）」能否从原始 bag 重放获取所需数据。
> 只陈述已实测核实的事实，未实测的项标注「待验证」。

---

## 1. 原始 bag 定位

| 项 | 值 |
|---|---|
| 路径 | `/home/ywj/slam_data/Retail_Street/Retail_Street.db3` |
| 大小 | 1.99 GB |
| 时长 | 135.47 s |
| 消息总数 | 30157 |
| storage | sqlite3（无压缩） |
| 配套配置 | `avia.yaml`、`camera_pinhole.yaml`（同目录） |

**结论**：这是 sl/e1 的上游原始 bag。sl（header 946685508.7→530.7）与 e1（946685507.99→531.7）都是该 135.47s 原始 bag 内的约 22~24s 子窗口（原始 bag LiDAR 首帧 header sec≈946685437，末帧≈946685572，覆盖该窗口）。

---

## 2. 三个 Topic 的可用性

| topic | 类型 | 消息数 | 频率 | 点级时间 | 状态 |
|---|---|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 1355 | ~10 Hz | ✅ 有 | ✅ 可用 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 27447 | ~200 Hz | — | ✅ 可用 |
| `/left_camera/image` | `sensor_msgs/msg/Image` | 1355 | ~10 Hz | — | ✅ 可用 |

三项均真实存在、非空、频率正常（LiDAR 与相机均 10Hz、IMU 200Hz）。

---

## 3. 逐项确认

### 3.1 LiDAR 点云格式与点级时间 ✅
- 格式为 **CustomMsg（`lidar_type: 1`）**，即 Livox Avia 原始报文，非 PointCloud2。
- 实测单帧 `point_num = 24000` = 6 条 scan line × 4000 点（`line ∈ {0..5}`，每线各 4000 点）。
- **点级时间已保留**（2026-11-10 重测，前 120 帧 / 288 万点样本）：
  - `offset_time` 范围 **0 → 99.92 ms**（对应 Avia ~100ms 扫描周期）；
  - 每帧**恰有 1 个点** `offset_time = 0`（帧首点），全样本共 120 个零值（0.0042%）——此前「100% 非零」的表述有误，正确说法是**字段完整填充、仅帧首点为 0**；
  - 每条 line 在存储序内 `offset_time` **严格单调不减**（120/120 帧），相邻点间隔 ≈ 25µs（最大跳变 25002ns），6 条线按 4167ns 相位交错（line k 起点 = k×4167ns）；
  - 帧间 header 间隔 99.82~100.7ms，无重复帧。
- ⚠️ **`timebase` 与 `header.stamp` 不在同一时刻域**：实测 `header.stamp − timebase ≡ 160.000000003 s`（恒定常数，120/120 帧）。即 `timebase` 是雷达内部时钟（上电起算），`header.stamp` 是驱动侧本地时钟，两者存在固定 160s 纪元差。**逐点绝对时刻 = `timebase + offset_time` 属于雷达内部时钟域，不能直接与 header/IMU 时间比较**。正确的帧内相对时间应取 `header.stamp + (offset_time − 99.9ms)` 或以 `offset_time` 为相对量、参考时刻锚定在 header 域（具体锚点取决于 Super-LIO 实现，见任务 3 源码审计）。
- 空间范围实测：x ∈ [0, 122]m，y ∈ [-8.6, 50]m，z ∈ [-0.8, 81]m（大尺度室内）。
- `tag` 仅两种取值 {0, 16}，`reflectivity` 0~150。

**结论**：原始 LiDAR 点 + 逐点时间戳 + 强度/线号全部可用，满足阶段 A「单帧去畸变前后几何对比」和阶段 B「重放不同配置重新去畸变」的前提。

### 3.2 IMU ✅
- `sensor_msgs/msg/Imu`，~200 Hz，与 LiDAR/相机同源同包。
- 可用于去畸变 / ESKF 状态估计重放。

### 3.3 相机图像 ✅
- `sensor_msgs/msg/Image`，10 Hz。
- 配套内参见 `camera_pinhole.yaml`（1280×1024，fx/fy≈1293.6，cx/cy≈626.9/522.8，含畸变 d0~d3）。阶段 A/B 若关闭颜色通道可不依赖图像；如需对齐仅作时间基准。

### 3.4 时间戳是否可用 ✅（含一个易踩的坑）
- **三个 topic 的 `header.stamp` 在同一时钟域**（Livox 设备本地时钟，~946685xxx 秒），彼此对齐、可用于同步与插值。
- **关键区别**：rosbag2 sqlite 的 `messages.timestamp` 列（= 1664265583，Unix 墙钟 2022）**不等于** `header.stamp`（= 946685437，本地时钟）。处理时必须以 `header.stamp` 为准，否则会出现 ~7.17 亿秒的错位。
- sl/e1 的 `colored_cloud`/`odom` header 也是 ~946685xxx 同一本地时钟域，与原始 bag 同域 → **时间对齐没有被破坏**。

### 3.5 能否重新生成 body-frame 点云 ✅（可生成，需重放）
- 原始 CustomMsg 本身就是 LiDAR 系（body）原始点 + 逐点时间；Super-LIO/FAST-LIVO2 用 `lidar_type: 1` 读入后自行去畸变。
- `avia.yaml` 的 `pcd_save` 支持 `type: 0=World / 1=Body`，可保存去畸变后的 body 系点云（当前配置 `pcd_save_en: false`，重放时改配置即可）。
- 已确认 Super-LIO 逐帧发布 body 系去畸变云，topic 为 **`lio/cloud_body_dense`**（visual_a_node 的 `body_cloud_topic` 配置默认值，注释为「全分辨率 + 异步去畸变输出」）。重放时直接订阅/录制该 topic 即可拿到逐帧 body 无色点云，无需改动生产代码。

---

## 4. 对阶段 A / B 的数据支撑结论

| 阶段需求 | 原始 bag 能否满足 | 说明 |
|---|---|---|
| 原始 LiDAR 点云 | ✅ | CustomMsg，24000 点/帧 |
| IMU 数据 | ✅ | 200 Hz |
| 每点时间信息 | ✅ | offset_time 100% 非零 |
| 去畸变后 body 系无色点云 | ✅ 可重放生成 | 重跑 Super-LIO，pcd_save.type=1 或订阅 body 云 |
| 对应帧状态估计与世界位姿 | ✅ 可重放生成 | 重跑后 /lio/odom |
| 统一帧采样 / 时间戳 / 空间范围 | ✅ | 原始 bag 单源，可统一 |
| 相机（阶段 B 可选，阶段 A 不需要） | ✅ | 10 Hz，内参齐全 |

**总体结论：原始 bag 数据完备，能够支撑阶段 A（单帧去畸变几何验证）与阶段 B（同源重放不同配置 + 留出帧位姿验证）。不需要再找其它数据源。**

---

## 5. 待验证 / 缺口

1. ~~Super-LIO 逐帧 body 云的确切 topic 名~~ ✅ 已确认：`lio/cloud_body_dense`。剩余需在重放时确认的是它是否**默认持续发布**（而非仅在 dense_map/pcd_save 开启时），这决定了阶段 A 能否免改代码录制。
2. **重放复现性**：需确认重跑 Super-LIO（`use_local_timestamp:=false`、`lidar_qos_reliable:=true`，见项目约束）能否复现 sl 的 odom；此前 sl/e1 的 header 处于本地时钟域，重放时应统一到同一设定，避免与历史产物错位。
3. **去畸变前后几何对比所需「未去畸变原始点」**：CustomMsg 的原始点即未去畸变点，但需确认 Super-LIO 是否对外暴露去畸变前的点序列（否则需在驱动/预处理层直接读 CustomMsg 自算）。
4. **点级时间去畸变的可重算性**：`offset_time` + IMU 已齐备，理论可独立重做去畸变；但 Super-LIO 的去畸变实现是否可直接复用需确认其入口。
5. **`timebase` 纪元差（新增，2026-11-10）**：`header.stamp − timebase ≡ 160s` 恒定。阶段 A 若自行重算逐点时间，必须先确认 Super-LIO 实际采用的参考域（header 域还是 timebase 域），否则重算的去畸变与官方输出存在系统性时间平移风险 → 归入任务 3（`DESKEW_PIPELINE_AUDIT.md`）核查项。

---

## 6. 附：与审计报告的关系

本清单呼应 [AUDIT_2026-11-09.md](file:///\\wsl.localhost\Ubuntu-22.04\home\ywj\git\Super-LIVO\docs\AUDIT_2026-11-09.md) §3 的「从原始 bag 重放」建议：确认了重放所需原始数据（LiDAR CustomMsg / IMU / 图像 / 逐点时间）齐全、时间戳可用、body 云可再生成，唯一前提是补齐 Super-LIO 逐帧 body 云的发布/保存确认（§5.1）。
