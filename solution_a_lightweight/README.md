# 方案 A：极致轻量化（Super-LIO + KLT + Colorizer）

> 当前状态：**已实现并接入 ROS2 节点**（视觉测量注入含噪声模型与信息量上限，彩色点云含 1cm 体素时间中位抗椒盐），已在 NTU VIRAL `eee_03` 评测（ATE 0.221 m，见 `docs/HANDOVER_2026-11-08.md`）。退化对抗实验（走廊裁剪、`Y_HALF=1.2`）结论：状态回灌链路已通，但当前对位姿**无纠正力**（同 bag |A−LIO|≤0.7 m），瓶颈在约束质量而非机制，详见 §8。下文架构与接口约定仍然有效。

## 1. 目标与边界

- 定位主干仍是 Super-LIO（ESIKF + OctVox），不改滤波器结构、不改地图结构。
- 新增两件事：**彩色点云**（与定位解耦）与**轻量视觉约束**（退化时参与更新）。
- 不引入完整 VIO、不引入滑窗优化器、不移植 FAST-LIVO2 视觉地图。

## 2. 系统架构

```
MID360 + IMU ──► Super-LIO（传播 / LiDAR 更新 / OctVox）
                        │
                        │ 当前位姿、局部地图
                        ▼
             轻量视觉前端（FAST + PyrLK）
                        │ 特征轨迹 + 跟踪质量
                        ▼
             深度关联（LiDAR 局部地图）
                        │ 带深度的 2D-3D 观测
                        ▼
             重投影约束 → 退化门控 → 视觉更新（可选）
                        │ 增量 dx（交由宿主 ESIKF 注入）
                        ▼
                  Super-LIO 原有 ESIKF

图像 + 点云 + 位姿 ──► Colorizer ──► 彩色点云（与视觉定位完全独立）
```

## 3. 目录结构与模块职责

```
solution_a_lightweight/
├── super_livo_visual_a/                  # 视觉前端 + 约束 + 更新适配
│   ├── include/super_livo_visual_a/
│   │   ├── frontend/klt_frontend.hpp     # KltFrontend：灰度金字塔 / FAST / PyrLK / 前后向一致性
│   │   ├── constraint/depth_association.hpp      # LidarDepthAssociation：特征 ↔ LiDAR 深度
│   │   ├── constraint/reprojection_constraint.hpp # ReprojectionConstraint：残差与 H/b
│   │   ├── constraint/degeneracy_gate.hpp         # DegeneracyGate：退化等级 → 视觉权重
│   │   └── update/visual_update.hpp               # VisualUpdate：仅适配，不持有滤波器
│   ├── src/                              # 与头文件一一对应的实现
│   └── src/node/visual_a_node.cpp        # ROS2 接线节点（含状态回灌，见 §8）
└── pointcloud_colorizer/                 # 彩色点云（方案 B 共用此包）
    ├── include/pointcloud_colorizer/
    │   ├── image_sync.hpp                # 图像 ↔ 扫描窗口时间对齐（曝光中点优先）
    │   ├── pose_interpolation.hpp        # 每点时间戳的位姿插值（运动补偿）
    │   ├── projection.hpp                # T_CL 变换 + 针孔投影 + 视野检查
    │   ├── occlusion_check.hpp           # 深度缓冲遮挡 / 深度边缘检查
    │   ├── rgb_assignment.hpp            # RGB 采样 + 颜色质量维护
    │   └── colorizer_pipeline.hpp        # 上述步骤的编排
    └── src/
```

## 4. 关键接口约定

| 接口 | 输入 | 输出 | 说明 |
| --- | --- | --- | --- |
| `KltFrontend::process` | 去畸变灰度图 + 时间戳 | `TrackResult` | 只跟踪，不碰位姿 |
| `LidarDepthAssociation::associate` | 特征 + `T_wc` | `FeatureDepth` | 深度来自 LiDAR，视觉不恢复尺度 |
| `ReprojectionConstraint::build/accumulate` | 特征 + 深度 + `T_wc` | 残差块 / `H,b,cost` | `r = z - π(K·T_CW·P_W)` |
| `DegeneracyGate::evaluate` | 退化度、跟踪数、分布、残差、条件数、可观测性 | `GateDecision` | 决定视觉是否/以多大权重参与 |
| `VisualUpdate::apply` | 法方程 + 门控 | `dx` | 由 Super-LIO ESIKF 注入并更新协方差 |
| `ColorizerPipeline::process` | 点云 + 每点时间戳 + 图像缓存 + 位姿缓存 | `ColoredPoint[]` | 视觉定位关闭时仍可用 |

## 5. 与 Super-LIO 的接入点

- 传播 / LiDAR 更新 / 地图：`third_party/Super-LIO/src/super_lio/include/lio/ESKF.h`、`super_lio.h`、`include/OctVoxMap/OctVoxMap.hpp`，**保持不改**。
- 视觉更新：在 LiDAR 更新之后、地图插入之前插入一次可选更新；`VisualUpdate` 只返回增量，由宿主注入。
- 地图插入顺序按原逻辑执行，方案 A 不引入延迟提交（该问题在方案 B 处理）。

## 6. ROS2 接口（实际接入，NTU 配置 `config/visual_a_ntu.yaml`）

| Topic | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/left/image_raw`（`image_topic`） | `sensor_msgs/Image` | 订阅 | 去畸变后驱动主流程 |
| `lio/odom` | `nav_msgs/Odometry` | 订阅 | Super-LIO 位姿 T_wl（child_frame_id 标 "imu" 但状态实为 LiDAR 位姿） |
| `lio/cloud_world` | `sensor_msgs/PointCloud2` | 订阅 | 世界系点云（深度关联） |
| `lio/cloud_body_dense` | `sensor_msgs/PointCloud2` | 订阅 | 当帧去畸变扫描（着色用） |
| `/imu/imu` | `sensor_msgs/Imu` | 订阅 | |
| `/lio/degeneracy` | `std_msgs/Float64MultiArray` | 订阅 | Super-LIO 退化签名 `[cond6, axes, active, cond_t, evt_min, ux, uy, uz]`（§8.1） |
| `super_livo_visual_a/odom` | `nav_msgs/Odometry` | 发布 | 修正后轨迹 |
| `super_livo_visual_a/path` | `nav_msgs/Path` | 发布 | |
| `super_livo_visual_a/colored_cloud` | `sensor_msgs/PointCloud2` | 发布 | Colorizer 输出 |

## 7. 实施阶段与验收

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| A1 | Colorizer 独立实现 | 静态颜色对齐；飞行不拖影；边缘无明显错色 |
| A2 | FAST + KLT 前端 | 输出轨迹与质量；楼梯间数据可离线回放 |
| A3 | LiDAR 深度辅助 2D-3D 更新 | 普通场景不劣化；退化场景航向漂移可观测地改善 |
| A4 | 退化门控与权重调优 | 视觉失效时能自动回退到纯 LIO 行为 |

> 视觉更新库内默认关闭（`VisualUpdate::Options::enable = false`），NTU 实验配置
> `visual_a_ntu.yaml` 中已显式打开（`update.enable: true`）。

## 8. 退化对抗（当前实现与结论）

纪律：不自造退化检测，只消费 Super-LIO（ros2 分支 `exp/degeneracy-adaptive`）发布的
`/lio/degeneracy`；Super-LIO 在裁剪输入下不可复现（B 侧 8 次重复跑实测 LIO 终点
14~794 m），一切对照只看同 bag 内逐点 |VIS−LIO| 与多跑分布，不引用单次绝对值。

与官方 FAST-LIVO2 的逐维度差异、以及 RK3588 实机上怎么绑核/比指标，见根
[README「与官方 FAST-LIVO2 的差异」与「RK3588 实机对比测试」两节](../README.md)。

### 8.1 退化门控真度量

- 严重度对数映射：`sev = log(cond/gate.cond_lo) / log(gate.cond_hi/gate.cond_lo)`，默认
  `cond_lo=35`、`cond_hi=10000`。本数据集健康段 cond_t≈25~40 → sev≈0；旧默认 10/100 会把
  cond_t≈33 误映射成 sev≈0.5，导致健康段无端松开先验（已修）。
- 完全退化（`n_effect=0`，cond=1e12）由 `axes>0 || active` 硬触发直接置 sev=1，不走映射。
- 签名超时 `gate.degen_timeout`（1.0 s）内未收到则视为无签名，按健康处理。

### 8.2 状态回灌 `update.reinject`（默认 true）

取代旧的"持久偏移 `pose_offset_` 左乘原始轨迹"模式：

- `onOdom` 里把 LIO 左增量 `dw = T_lio(k)·T_lio(k-1)⁻¹` 挂到修正轨迹缓存尾
  （`own.T_wl = dw · own_pose_cache_.back().T_wl`，首条以原始 LIO 位姿种子）。
- 视觉基准位姿只查修正缓存（`poseAtStampCorrected`），更新接受后写回缓存尾
  （时间差 ≤0.15 s 才写回）；发布与诊断均基于 `own_pose_cache_`。
- 诊断行"持久偏移"即 `off_disp = own.back()·lio.back()⁻¹`，沿退化法向分解 mm 值。
- 设 false 退回旧模式（`pose_offset_` + 各向异性衰减 `decayPoseOffset`）。

### 8.3 实验结论（NTU eee_03 走廊裁剪，Y_HALF=1.2）

- A 终点误差 98.4 m vs 同 bag Super-LIO 102.9 m，全程 |A−LIO|≤0.7 m：**回灌链路通了，
  但没有产生纠正力**。
- 根因在约束质量：走廊段大量帧 `mode=0`（判健康不更新）或 `mode=3`（重投影 rmse 2~4 px
  超 `gate.max_reprojection_rmse=3.0` 拒绝、cond 达 1e5），可用约束 n=0~33 太少。
- 日志判据：视觉帧行 `tracked=x/y n= spread= rmse= bias= cond= deg= mode= w= <原因> | 耗时 ms:`；
  退化状态行 `退化状态: sev= dir= 修正=mm(沿u) | 持久偏移=mm(沿u) °`。
- 候选改进（未实施）：放宽 rmse 拒绝门槛、增加退化段特征配额、或借用 B 的历史锚点式
  绝对约束替代逐帧 LiDAR 深度关联。
