# 方案 B：紧耦合 LiDAR-Visual-Inertial（相对稳定可靠）

> 当前状态：**仅架构骨架**。所有类只有接口声明与空实现，`TODO(阶段 Bn)` 标注了后续实现位置，不含任何算法逻辑。

## 1. 目标与边界

- 视觉具备**独立短期运动估计能力**，再与 Super-LIO 紧耦合，而不是只做图像上色。
- 统一状态估计器：仍以 Super-LIO 的 ESIKF 为中心，不引入第二套 EKF/滑窗优化器。
- 借鉴 Basalt / SVO Pro / FAST-LIVO2 的前端与特征管理机制，不直接移植其完整后端。

## 2. 系统架构

```
MID360 + IMU ─┐
              ├─► Super-LIO（LiDAR 匹配 / OctVox 几何地图）
全局快门相机 ─┘            │
                           │ 当前位姿、几何地图
                           ▼
                 视觉前端（KLT/LSSD Patch + 特征生命周期）
                           │ 多帧特征轨迹
              ┌────────────┴────────────┐
              ▼                         ▼
   LiDAR 地图辅助 2D-3D 约束      MSCKF 多帧轨迹约束
              │                         │
              └────────────┬────────────┘
                           ▼
                 统一 ESIKF（传播 + LiDAR + Visual）
                           │
                   延迟地图提交 → OctVox / Visual Map
                           ▼
                   统一轨迹 + 彩色点云
```

## 3. 目录结构与模块职责

```
solution_b_tight_coupled/
├── super_livo_visual_b/
│   ├── include/super_livo_visual_b/
│   │   ├── state/state_layout.hpp              # 状态定义与块索引（R,p,v,bg,ba,g,T_IC,T_IL）
│   │   ├── frontend/patch_tracker.hpp          # PatchTracker：KLT/LSSD 图像块跟踪
│   │   ├── frontend/feature_lifecycle.hpp      # FeatureLifecycle：new/tracked/mature/lost
│   │   ├── feature_map/feature_track.hpp       # FeatureTrack：多帧观测轨迹
│   │   ├── feature_map/keyframe_manager.hpp    # 视觉关键帧与轨迹地图（独立于 OctVox）
│   │   ├── constraint/lidar_visual_constraint.hpp  # 2D-3D 重投影（地图辅助）
│   │   ├── constraint/msckf_constraint.hpp         # 多帧重投影 + 零空间投影
│   │   ├── constraint/photometric_constraint.hpp   # 可选 Patch 光度残差
│   │   ├── estimator/unified_estimator.hpp         # 统一 ESIKF 封装
│   │   └── estimator/map_commit_scheduler.hpp      # 延迟地图提交
│   └── src/                                     # 与头文件一一对应的空实现
└── README.md
```

彩色点云模块**直接复用** [`../solution_a_lightweight/pointcloud_colorizer`](../solution_a_lightweight/pointcloud_colorizer)，不做重复实现：它与定位主干解耦，两套方案共用同一份代码。

## 4. 视觉约束的两类来源

| 类型 | 实现文件 | 数据来源 | 作用 |
| --- | --- | --- | --- |
| LiDAR 地图辅助 2D-3D | `lidar_visual_constraint.hpp` | OctVox 三维锚点 | 尺度/深度明确，但地图扭曲时会继承误差 |
| MSCKF 多帧轨迹 | `msckf_constraint.hpp` | 连续图像同一特征 | 不依赖全局地图，提供独立航向信息 |
| （可选）光度 Patch | `photometric_constraint.hpp` | 图像纹理 | KLT 几何约束不足时补充 |

## 5. 滤波与地图提交顺序

```
IMU 传播 → LiDAR 更新 → 更新几何地图 → 视觉跟踪
        → 视觉测量更新 → 状态/协方差统一更新
        → 维护关键帧与特征轨迹 → 延迟提交地图
```

注意：视觉更新会改变当前位姿，**不能用更新前位姿插点**。`MapCommitScheduler` 负责把提交推迟到更新完成后重新计算变换。

## 6. ROS2 接口规划

| Topic | 类型 | 方向 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | 订阅 |
| `/livox/lidar`、`/livox/imu` | 点云 / IMU | 订阅（Super-LIO 已订阅） |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 订阅（Super-LIO 输出） |
| `/cloud_colored` | `sensor_msgs/PointCloud2` | 发布（复用 Colorizer） |
| `/visual/tracks` | 自定义 | 发布（特征轨迹） |
| `/visual/keyframes` | 自定义 | 发布（关键帧） |
| `/visual/diagnostics` | 自定义 | 发布（约束数量、条件数、更新量） |

## 7. 实施阶段与验收

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| B1 | 统一状态与延迟地图提交 | 状态/协方差维度一致；插入顺序无位姿错配 |
| B2 | Patch 前端 + 特征生命周期 | 长序列跟踪稳定；特征轨迹可离线检查 |
| B3 | 2D-3D + MSCKF 多帧约束 | 楼梯井累计航向扭曲显著减小；普通场景不劣化 |
| B4 | 可选光度约束 | 纹理弱/几何弱场景的补充收益可量化 |

## 8. 已知限制（需在实现阶段验证）

- 单目多帧视觉存在尺度与特定运动下的可观测性限制，必须与 IMU/LiDAR 联合使用。
- 视觉约束的质量必须以协方差 + 质量指标输出，供融合层按传感器可信度决策。
- 高频 Patch 与金字塔会提升算力开销，需通过特征数、关键帧频率、更新频率控制。
