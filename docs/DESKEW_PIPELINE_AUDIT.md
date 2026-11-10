# Super-LIO 去畸变坐标链审计（DESKEW_PIPELINE_AUDIT）

- 日期：2026-11-10
- 审计对象：`third_party/Super-LIO/src/super_lio`（已安装二进制构建于 2026-11-06 21:02，源码 HEAD 67c0acd4 之后无影响链路的改动；本报告行号以当前源码为准，关键逻辑与二进制一致）
- 数据对象：`/home/ywj/slam_data/Retail_Street/Retail_Street.db3`
- 方法：逐行源码审计 + 历史运行日志 `/tmp/sf_slio.log` 交叉验证 + bag 实测统计（见 `DATA_INVENTORY_2026-11-09.md`）
- 范围：仅回答"原始 CustomMsg 点 → `/lio/cloud_body_dense`"的坐标与时间处理链，不含任何新实验设计。

## 0. 结论速览

| 问题 | 答案 |
|---|---|
| `timebase` 是否使用 | **完全未使用**（全仓库 grep 零引用），header.stamp 与 timebase 的 ~160 s 纪元差与 Super-LIO 无关 |
| `offset_time` 单位 | ns（Livox 官方定义，相对帧首），代码 `×1e-9` 转秒（ROSWrapper.cpp L1043） |
| 每点绝对采样时间 | `header.stamp + offset_time`（header 时钟域，本地时钟 ~9.46685e8 s） |
| IMU 插值 | 相邻 IMU 状态间线性索引；旋转用二阶 `Exp(ω·τ)` 小角近似；平移用 `v·τ + ½a·τ²` 泰勒展开 |
| 去畸变参考时刻 | **帧尾**：T_end = 本帧最后一个 IMU 采样时刻的位姿（≈ end_time − [0, 5ms)） |
| LiDAR/IMU 外参 | 仅旋转参与：IMU 的 acc/gyr 被旋到雷达系（L958）；**平移（杆臂）从未被使用**；r3live.yaml 旋转=单位阵 ⇒ 内部 body 系 ≡ 雷达系 |
| 去畸变后是否再滤波 | 无额外滤波/裁剪；仅入口过滤（blind/maxrange/tag）与帧尾超时无补偿点剔除 |
| `cloud_body_dense` 坐标系 | 配准校正后位姿的机体系（frame_id 标签为 `"imu"`，几何上即雷达系），帧尾参考时刻 |

## 1. 原始数据字段语义（CustomMsg 入口）

`livoxHandler`（[ROSWrapper.cpp](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/ros/ROSWrapper.cpp#L1019-L1068)）：

```
pt_time = pt.offset_time * 1e-9          // ns → s，相对帧首（L1043）
        + g_lidar_time_offset            // 参数 lio.ros.lidar_time_offset，默认 0
        + td_extra                       // 时延标定侧车，未启用恒 0（L1033）
lidar_data.start_time = stampToSec(msg->header.stamp)              // L1061，header 域
lidar_data.end_time   = start_time + 最后一个通过过滤点的 offset_time  // L1062
```

- `timebase`：全仓库无任何引用（grep `timebase` → 0 命中）。实测 `header.stamp − timebase ≡ 160 s` 恒定，属 Livox 驱动纪元差，**不进入 Super-LIO 任何计算**。
- `offset_time` 实测（Retail_Street，1355 帧）：范围 0 → 99.92 ms；每帧恰 1 个点为 0（帧首点）；同一 scan line 内单调；相邻点间隔 ≈ 25 µs。语义 = 相对帧首的采样时刻，与代码用法一致。
- `end_time` 由**最后一个通过入口过滤的点**定义（非最后一个原始点）；若帧尾点被 blind/maxrange 过滤掉，end_time 提前。

## 2. 入口过滤（L1036-1052）

逐点过滤条件（对 pc_full 与 pc 相同）：

| 条件 | 参数值（r3live.yaml） | 效果 |
|---|---|---|
| `pt.tag & 0x30 ∈ {0x00, 0x10}` | — | 只保留正常回波 |
| `dis > blind²` | blind = 2.0 m | 剔除 2 m 内自反射 |
| `dis < maxrange²` | maxrange = 100 m | 剔除 100 m 外 |
| 强度过滤 | 关闭 | — |

输出两份点云：
- `pc_full`：全部过滤后点（≈ 原始点数）→ **dense 话题唯一来源**；
- `pc`：每 `filter_rate=3` 点抽稀 1 点 + 0.5 m 体素降采样 → ESKF 观测/配准用。

## 3. 时间同步（sync_measure，[ROSWrapper.cpp L1516-L1562](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/ros/ROSWrapper.cpp#L1516-L1562)）

- 处理一帧 LiDAR 前，等待 `last_timestamp_imu_ >= lidar.end_time`（IMU 必须覆盖帧尾）；
- 本帧 IMU 窗口：`imu_time < end_time`（**严格小于**）。
- IMU 与 LiDAR header.stamp 同处本地时钟域（历史日志 `[IMU Delay] msg.stamp=9.46685e+08` 与 lidar 同域），无额外偏移参数。

## 4. IMU 传播（[super_lio.cpp L1264-L1295](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/lio/super_lio.cpp#L1264-L1295) + ESKF::Predict L219-295）

```
SetObsTime(end_time)
for imu in measures_.imu: kf_->Predict(imu, g_max_imu_dt)   // dt 截断到 current_obs_time_
                                                 // 断档 dt>0.05s 只对齐时间不积分
T_end = kf_->GetSE3()          // = 最后一个 IMU 采样时刻的位姿（先验/预测位姿）
propagate_states_ = 每个 IMU 时刻一个状态（含初始状态，M = IMU 数 + 1）
```

注意：`T_end` 的时刻是**最后一条 IMU 的采样时刻**（200 Hz ⇒ 距 end_time ≤ 5 ms），不是 end_time 本身。

## 5. 逐点去畸变（undistortCloud，[super_lio.cpp L1299-L1412](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/lio/super_lio.cpp#L1299-L1412)）

参考系 = T_end（帧尾先验位姿）。逐点：

```
query_time = start_time + pt.offset_time            // 点的绝对采样时刻（header 域）
if query_time > back_time: 剔除该点                  // back_time = 最后 IMU 时刻（L1371-1375）
区间索引 j：线性推进，tau = query_time − s_j.time
旋转：二阶 Exp(ω_j·τ)，ω_j = 该区间原始 gyr − bg_（与 ESKF 传播同源，L1388-1401）
平移：p' = R_end⁻¹(R_j·p_expanded + v_j·τ + ½·a·τ²)   // 泰勒展开（L1403-1409）
```

关键行为：
1. **帧尾点剔除**：`query_time > back_time` 的点被丢弃（不是透传）。剔除窗口 = 最后 IMU 时刻与 end_time 之差，∈ [0, 5 ms)，均值 ≈ 2.5 ms ⇒ 每帧尾部最多 ~2.5% 的点（~500 点）不进入 dense 云。阶段 A 做"残差随 offset_time 分布"时，帧尾区间天然无样本。
2. `propagate_states < 2`（无 IMU 区间）时原样透传，不去畸变。
3. 角速度取 `gyr − bg_`（逐区间常数），与 ESKF 积分同一套 ω，避免常值偏置累积（L1276-1285 注释明确此设计）。
4. 平移项用区间起点状态 `s_j` 的 `v_j` 与区间终点加速度 `s_{j+1}.a`；旋转与平移都只补偿到 T_end，**帧内不做位姿插值以外的修正**。

## 6. dense 链路与 delta correction（[super_lio.cpp L648, L793-819, L822-864](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/lio/super_lio.cpp#L822-L864)）

- `enqueueDense()` 在每个 **trusted 帧**配准后无条件调用（L648），**不受 `pcd_save_en` / `dense_map` / `lio.output.dense` 门控**（`g_visual_dense` 只影响 reloc 路径）。Guard 跳帧（lidar_gap>0.2 s 或 IMU 覆盖异常）时该帧既无 odom 也无 dense。
- DenseThread 异步执行：
  1. `undistortCloud(pc_full, T_pred)` —— 与实时链路同一函数、同一参考位姿（帧尾先验）；
  2. **delta correction**（L854-861）：`dR = T_corr.Rᵀ·T_pred.R`，`dp = T_corr.Rᵀ·(T_pred.t − T_corr.t)`，逐点刚体应用 ⇒ 最终点落在**配准校正后位姿**的机体系；
  3. `pub_cloud_body_dense(dense_body, data.timestamp)`。

**对阶段 A 的含义**：delta correction 是整帧刚体变换，**不改变帧内几何形状**（点间距离、局部平面厚度、残差分布均不变）。因此 `/lio/cloud_body_dense` 的单帧几何 ≡ `undistortCloud(pc_full, T_pred)` 的几何；但它不是"纯去畸变"输出——其位姿标签与 `/lio/odom` 一致（含配准修正）。

## 7. 发布语义（[ROSWrapper.cpp L1795-L1801](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/ros/ROSWrapper.cpp#L1795-L1801)）

```
frame_id    = g_imu_frame = "imu"      // 参数 lio.output.imu_frame 默认值；r3live.yaml 未设置
header.stamp = toRosTime(data.timestamp)
             = kf_->GetNavState().timestamp = 本帧最后 IMU 采样时刻
QoS: best_effort, KeepLast(2)          // publisher 无条件创建（L910-916）
```

- **stamp 不是 LiDAR header.stamp**：比对应帧 LiDAR header.stamp 晚 ≈ 帧尾偏移（~95-100 ms）。
- `/lio/odom` 同样使用 `state.timestamp`（最后 IMU 时刻，[super_lio.cpp L2337-L2348](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/lio/super_lio.cpp#L2337-L2348)）⇒ **同帧的 body_dense 与 odom 的 header.stamp 相等**（实测逐位一致，见 BODY_CLOUD_VALIDATION.md），这是任务 1 匹配检查的正确口径；与 LiDAR header.stamp 差 ~+95 ms 属正常，不是错位。
- `/lio/odom` 消息（[ROSWrapper.cpp L1586-L1625](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/ros/ROSWrapper.cpp#L1586-L1625)）：`header.frame_id=world`，`child_frame_id="base_link"`，位姿 = IMU/雷达系后验再乘 `lio.extrinsic.odom_robo`（r3live.yaml 全 0 ⇒ 数值上与机体系位姿相同）。高频路 `/lio/imu/odom`（200Hz，child=imu）与 `/lio/robo/odom` 另存。

## 8. 外参的真实用法（重要发现）

`g_lidar_imu`（SE3，来自 `lio.extrinsic.lidar_imu = [0.04165, 0.02326, -0.0284, 单位旋转]`）全仓库仅 4 处引用（params.h/cpp 声明、构造、L958 使用）：

```cpp
M3 R_IMU_to_Lidar = g_lidar_imu.R_.transpose();   // ROSWrapper.cpp L958
data.acc = R_IMU_to_Lidar * acc_raw;               // IMU 向量旋到雷达系
data.gyr = R_IMU_to_Lidar * gyr_raw;
```

- **点云从不乘以外参**；外参的平移（杆臂）**从未参与任何计算**。
- 因此内部状态机体系 = 雷达系；`frame_id="imu"` 只是标签。当前配置旋转为单位阵 ⇒ IMU 向量数值不变，"imu 系"与"雷达系"完全重合（原点差 = 未补偿的杆臂 4.2/2.3/−2.8 cm，且因杆臂未用而不可区分）。
- 含义：body_dense 的几何原点就是 LiDAR 光学中心；若未来做 LiDAR-相机外参联合评价，不能把 `"imu"` 标签当作物理 IMU 原点。

## 9. 坐标与时间处理链（总图）

```
CustomMsg{header.stamp(H域), points{x,y,z,reflectivity,tag,offset_time[ns]}, timebase[未用]}
 │
 ├─ livoxHandler (ROSWrapper L1019-1068)
 │    入口过滤: tag∈{0x00,0x10}, 2m<dis<100m
 │    pt_time = offset_time×1e-9 (+0)                    [相对帧首, s]
 │    start_time = header.stamp; end_time = start + 最后存活点 offset
 │    pc_full = 全量过滤点;  pc = 1/3抽稀 + 0.5m体素
 ▼
 sync_measure (L1516-1562): 等 last_IMU_time ≥ end_time; IMU 窗 [·, end_time)
 ▼
 Propagation_Undistort (super_lio L1264-1295)
 │    逐 IMU kf_->Predict(dt≤0.05s 截断/断档冻结)
 │    T_end = 最后 IMU 时刻位姿(先验); propagate_states 每 IMU 一个
 ▼
 实时链路: undistortCloud(pc, T_end) → ESKF Observe → T_corrected
 ▼
 enqueueDense (L648, 每 trusted 帧无条件)
 ▼
 DenseThread (L822-864)
 │    ① undistortCloud(pc_full, T_pred=T_end)
 │       · query_time = start + offset_time
 │       · query_time > 最后IMU时刻 → 剔除(帧尾≤5ms)
 │       · 旋转 Exp(ωτ) 二阶, ω=gyr−bg; 平移 v·τ+½a·τ² 泰勒
 │    ② delta correction: 整帧刚体 T_pred→T_corrected (不改帧内形状)
 │    ③ pub_cloud_body_dense(dense_body, stamp=最后IMU时刻)
 ▼
/lio/cloud_body_dense: frame_id="imu"(=雷达系), 帧尾参考, 配准校正后位姿机体系
/lio/odom: 同 stamp, world→"imu"
```

## 10. 逐条对照任务 3 问题清单

| # | 问题 | 结论（出处） |
|---|---|---|
| 1 | timebase / offset_time 单位与用法 | timebase 未用（grep 0 命中）；offset_time ns→s 相对帧首（L1043） |
| 2 | 每点真实采样时间 | `header.stamp + offset_time + g_lidar_time_offset(0) + td_extra(0)`（L1044, L1361） |
| 3 | IMU 插值/外推 | 区间线性索引 + 二阶小角旋转 + 泰勒平移；不外推超 back_time，超界点剔除（L1371-1375）；断档 dt>0.05 s 冻结（Predict L219-295） |
| 4 | 去畸变参考时刻 | 帧尾：最后 IMU 采样时刻的先验位姿 T_end（L1274） |
| 5 | LiDAR/IMU/body 外参 | 仅 R 用于旋转 IMU 向量到雷达系（L958）；t 未用；R=I ⇒ body ≡ 雷达系 |
| 6 | 去畸变后是否再滤波 | 无；仅入口过滤 + 帧尾时间剔除（§5/§6） |
| 7 | cloud_body_dense 属于哪个 frame | 配准校正后位姿的机体系（雷达系），帧尾参考时刻；stamp=最后 IMU 时刻=同帧 odom stamp |

## 11. 对阶段 A/B 的约束（仅陈述事实，不构成实验设计）

1. **阶段 A（帧内几何）**：body_dense 的帧内形状完全由 `undistortCloud(T_pred)` 决定（delta correction 为刚体，不影响形状），可直接用于单帧平面残差；但残差随 offset_time 的分析在帧尾 [back_time, end_time] 区间无样本（点已剔除）。
2. **阶段 B（帧间位姿）**：body_dense 与 odom 同 stamp、同位姿（校正后），二者配对无时间歧义；与 LiDAR header.stamp 的 ~+95 ms 差是参考时刻定义所致，不是数据问题。
3. 原始 CustomMsg **不可**当作统一时刻刚性云直接比较（雷达系逐点采样时刻不同）——与用户 §2 判断一致。
4. `frame_id="imu"` 为标签性命名，几何上是雷达系；杆臂未参与计算，若需 IMU 原点系需另行变换。

## 12. 遗留不确定项

- 二进制（2026-11-06 21:02 构建）与当前源码间若存在未提交改动，行号会漂移；关键逻辑（L1043 ×1e-9、L958 外参、L1371 剔除、stamp 语义）已由历史运行日志行为佐证（IMU Delay、Guard skip、odom 频率）。
- `g_lidar_time_offset` / `td_extra` 在历史运行中均为默认 0（日志无相关参数覆盖记录）。
