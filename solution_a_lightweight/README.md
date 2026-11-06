# 方案 A：极致轻量化（Super-LIO + KLT + Colorizer）

> 当前状态：**仅架构骨架**。所有类只有接口声明与空实现，`TODO(阶段 N)` 标注了后续实现位置，不含任何算法逻辑。

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
│   └── src/                              # 与头文件一一对应的空实现
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

## 6. ROS2 接口规划

| Topic | 类型 | 方向 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | 订阅 |
| `/livox/lidar`、`/livox/imu` | 点云 / IMU | 订阅（Super-LIO 已订阅） |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 订阅（Super-LIO 输出） |
| `/cloud_colored` | `sensor_msgs/PointCloud2` | 发布（Colorizer） |
| `/visual/features` | 自定义 | 发布（调试） |
| `/visual/status` | 自定义 | 发布（退化等级、门控结果） |

## 7. 实施阶段与验收

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| A1 | Colorizer 独立实现 | 静态颜色对齐；飞行不拖影；边缘无明显错色 |
| A2 | FAST + KLT 前端 | 输出轨迹与质量；楼梯间数据可离线回放 |
| A3 | LiDAR 深度辅助 2D-3D 更新 | 普通场景不劣化；退化场景航向漂移可观测地改善 |
| A4 | 退化门控与权重调优 | 视觉失效时能自动回退到纯 LIO 行为 |

> 视觉更新默认关闭（`VisualUpdate::Options::enable = false`），先保证 A1/A2 可独立验证。
