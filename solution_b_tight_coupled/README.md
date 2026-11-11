# 方案 B：紧耦合 LiDAR-Visual-Inertial（相对稳定可靠）

> 当前状态：**已实现并接入 ROS2 节点**（统一 ESIKF、测量噪声模型与信息量上限已补齐，估计器修过 IMU 初始化），已在 NTU VIRAL `eee_03` 做过早期评测（ATE 0.681 m，IMU 初始化修复后未重测，见 `docs/HANDOVER_2026-11-08.md`）。退化对抗实验（走廊裁剪、`Y_HALF=1.2`）已用 **8 次重复单跑**验证：B 8/8 次不劣于同 bag LIO，并把终点误差的跨跑散布从 LIO 的 14~794 m 压到 8~128 m；抑制幅度按口径为均值 −73% / 中位数 −50%（单次数字不可复现，勿引用）。机制与结论见 §9，选轨耗时修复见 §9.6。下文架构与接口约定仍然有效。

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
│   ├── src/                                     # 与头文件一一对应的实现
│   └── src/node/visual_b_node.cpp               # ROS2 接线节点（退化感知与守卫见 §9）
└── README.md
```

彩色点云模块**直接复用**独立包 [`../pointcloud_colorizer`](../pointcloud_colorizer/README.md)，不做重复实现：它与定位主干解耦，两套方案共用同一份代码。

A/B 共用的纯算法（退化签名映射、各向异性先验、重投影方差模型）在
[`../common/super_livo_common`](../common/super_livo_common/README.md)；
改这些口径时**只需改一处**，两端同步生效。

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

## 6. ROS2 接口（实际接入，NTU 配置 `config/visual_b_ntu.yaml`）

| Topic | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/left/image_raw`（`image_topic`） | `sensor_msgs/Image` | 订阅 | Patch 前端与着色共用 |
| `lio/odom` | `nav_msgs/Odometry` | 订阅 | Super-LIO 位姿 T_wl（状态实为 LiDAR 位姿） |
| `lio/cloud_world` | `sensor_msgs/PointCloud2` | 订阅 | 世界系点云（2D-3D 锚点来源） |
| `lio/cloud_body_dense` | `sensor_msgs/PointCloud2` | 订阅 | 当帧去畸变扫描（着色用） |
| `/imu/imu` | `sensor_msgs/Imu` | 订阅 | 统一 ESIKF 传播 |
| `/lio/degeneracy` | `std_msgs/Float64MultiArray` | 订阅 | Super-LIO 退化签名 `[cond6, axes, active, cond_t, evt_min, ux, uy, uz]`（§9.2） |
| `super_livo_visual_b/odom` | `nav_msgs/Odometry` | 发布 | 视觉修正后轨迹 |
| `super_livo_visual_b/path` | `nav_msgs/Path` | 发布 | |
| `super_livo_visual_b/colored_cloud` | `sensor_msgs/PointCloud2` | 发布 | 复用 Colorizer |

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

## 9. 退化对抗（当前实现与验证结论）

纪律：不自造退化检测，只消费 Super-LIO 的 `/lio/degeneracy`；Super-LIO 在裁剪输入下
不可复现（同输入终值随机，8 次重复跑实测 LIO 终点 14~794 m），一切对照只看同 bag 内
逐点 |VIS−LIO| 与多跑分布，不引用单次绝对值。

与官方 FAST-LIVO2 的逐维度差异、以及 RK3588 实机上怎么绑核/比指标，见根
[README「与官方 FAST-LIVO2 的差异」与「RK3588 实机对比测试」两节](../README.md)。

### 9.1 LiDAR 地图绝对锚定 + 历史锚点保持

实现在 `constraint/lidar_visual_constraint.*`。2D-3D 约束的三维锚点不再只信"当帧彩色
地图"（漂移地图会把误差原样注入），而是引入跨帧持久的 `anchors_`（feature_id →
`AnchorEntry{xyz, stamp, last_seen}`）：

- 新建特征两级关联：先把**全部持久锚点**投影建栅格（hgrid，8 px cell），命中则优先继承
  历史锚点（`anchored=true`，走 60 px 宽门限）；未命中再退回当前帧漂移地图栅格。
- 关联门限通过才提交锚点并推进 `last_seen`（健康期建立的锚点不会被退化段的漂移观测污染）。
- 过期与容量：`lv.anchor_keep_sec`（默认 20 s，按 `frame_stamp − last_seen`）清除；
  超过 `max_anchors`（20000）用 `nth_element` 按最久未观测淘汰。
- 效果：退化段观测仍能锚回健康期建立的世界坐标，产生真实纠正力（这是此前缺失的那一环）。

### 9.2 退化感知先验与 sev 映射

`buildLidarPrior`：`Hp = (1−sev)·Hp + sev·Hp_dir`——退化方向松开对 LIO 的信任、垂直方向
收紧。sev 为对数映射 `log(cond/gate.cond_lo)/log(gate.cond_hi/gate.cond_lo)`，默认
`cond_lo=35`、`cond_hi=10000`（本数据集健康段 cond_t≈25~40 → sev≈0；旧默认 10/100 会把
cond_t≈33 误映射成 0.5，健康段就无端松开先验，已修）。完全退化（`n_effect=0`，cond=1e12）
由 `axes>0 || active` 硬触发置 1。注入强度上限：`est.visual_info_ratio=3.0`
（退化时 10.0）+ σ 地板（5 mm / 退化 3 mm），防协方差塌缩。

### 9.3 退化段守卫

`guard.enable=true`：先验否决（LIO 帧间跳变超 `lio_jump_pos=1.0 m` / `lio_jump_rot=0.2618 rad`
且 sev≥`0.9` 时拒绝跟随）+ 速度限幅（`vel_ratio=1.5`、`vel_fallback=3.0`、`vel_abs_cap=3.0`），
日志尾段 `守卫: 先验否决= 速度限幅=` 计数。

### 9.4 验证结论（NTU eee_03 走廊裁剪，Y_HALF=1.2，8 次重复单跑 tag=y12fix2~y12fix9）

单次数字不可复现（LIO 侧尤甚），结论只用**同 bag 配对**与**跨跑分布**表述。

| TAG | B 终点 (m) | 同 bag LIO 终点 (m) | 抑制 | \|B−LIO\| p50 (m) |
| --- | --- | --- | --- | --- |
| y12fix2 | 37.00 | 146.33 | 75% | 0.029 |
| y12fix3 | 106.79 | 184.47 | 42% | 0.025 |
| y12fix4 | 83.31 | 623.83 | 87% | 0.030 |
| y12fix5 | 14.19 | 14.12 | ≈0% | 0.019 |
| y12fix6 | 86.37 | 156.09 | 45% | 0.029 |
| y12fix7 | 8.41 | 14.32 | 41% | 5.521 |
| y12fix8 | 127.81 | 208.09 | 39% | 0.053 |
| y12fix9 | 107.04 | 794.37 | 87% | 0.034 |

1. **方向稳定**：8/8 次 B 不劣于同 bag LIO。唯一没有抑制空间的 y12fix5 是 LIO 本身几乎
   没漂（终点 14 m）的健康跑，B 与它在 0.5% 内持平——视觉介入不破坏健康解。
2. **幅度取决于口径，禁止引用单次数字**：均值 71.4 vs 267.7 m（−73%）；中位数
   84.9 vs 170.3 m（−50%）；逐次比值中位 41.5%、区间 0%~87%。
3. **B 压缩了不可复现性本身**（最硬的一条）：LIO 终点跨跑 14.1~794.4 m（55×），B 只有
   8.4~127.8 m（15×）。随机漂移主要来自 LIO/裁剪链路，绝对锚定把它吸掉大半。

`|B−LIO| p50` 除 y12fix7（5.52 m，B 与 LIO 系统性分开且 B 更准）外均落在 0.02~0.05 m：
健康段 B 紧贴 LIO，纠正力集中在漂移段；介入自 60~70 s 开始，残余误差主要是介入起点偏晚
（退化早期 sev/cond 尚未越门槛）。

待办/疑点：

- 介入偏晚与少量 `lv=0` 帧的原因分析（未排任务）。

### 9.5 日志判据

视觉帧行：

```
视觉帧: obs= tracks= lv= mscfk= used= quality= | 退化 sev= dir= | 注入 λ= cap= scale= dxθ= dxp= σp= σθ= |v|= | 耗时 ms: … | 约束细分 ms: 拷贝地图= 投影建栅格= 法方程= 选轨= mscfk= | 轨迹池= 观测= 最长= 配对= | 地图点数= | 守卫: 先验否决= 速度限幅=
```

注意 `tracks=` 是 MSCKF 选轨输入数、`lv=` 是 2D-3D 约束数；`选轨=` 仅计
`keyframe_manager_->constraintTracks` 一段；`轨迹池/观测/最长/配对` 来自
`KeyframeManager::trackPoolStats()`（配对=Σn(n−1)/2，即视差扫描的 acos 次数上限）。

### 9.6 选轨耗时：视差扫描 O(n²)（已定位并修复）

- **CPU 挤占假设不成立**：B 单跑（tag=y12hist2）与双套并跑的分布几乎逐项相同
  （p90 258 vs 264 ms、max 422 vs 410 ms、>50 ms 帧数 43/179 vs 43/181）→ 是真算得慢。
- **根因**：低视差段没有任何轨迹达到 `msckf.min_parallax_deg`，`parallaxDeg` 的"达标即退出"
  永不触发，每条轨迹跑满 O(n²)；叠加 §9.1 的历史锚点使长期轨迹观测数涨到 99~275，单帧配对数
  达 0.9M~7.2M（406 ms / 7.2M ≈ 56 ns/对，成本模型闭合）。
- **修复**（`feature_track.*`）：`kMaxObservations=16` 截断（保留首帧 + 最近 15 帧——首帧必须
  留住，否则 LV 约束 `observations.front()` 的历史锚点路径失效）+ 视差结果缓存：追加观测只把
  缓存置为失效，查询时按上限规模（最多 120 对）全量重建。
  **不要**在 `addObservation` 里增量算配对：实测段错误，gdb 栈停在 `pairAngleDeg`；截断后每帧
  最坏 200 轨迹 × 120 对 ≈ 1.5 ms，增量缓存属多余优化。
- **效果**：8 次重复单跑 `>50 ms 帧数 = 0/176~178`，选轨 p50 0.3~0.5 ms、max ≤3.0 ms。
- **语义变化**：参与 MSCKF 的轨迹观测史被限制在 16 帧（约束强度略降），基线一律按
  tag=y12fix* 重跑，修复前的 y12hist* 数字不再可比。
