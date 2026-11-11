# pointcloud_colorizer

LiDAR 点云着色（彩色点云）独立包。与定位主干完全解耦：**视觉定位关掉也能单独跑**，
方案 A（轻量）与方案 B（紧耦合）共用同一份实现，不再各自复制。

## 1. 输入 / 输出

| 方向 | 内容 |
| --- | --- |
| 输入 | LiDAR 点云 + 每点时间戳、图像缓存（含曝光中点时间戳）、位姿缓存 `T_WL(t)`、相机内参、外参 `T_CL` |
| 输出 | `ColoredPoint[]`：`xyz` + `rgb(0~255)` + `quality(0~1)` |

## 2. 处理链

```
图像时间对齐 → 每点位姿插值（运动补偿）→ T_CL 变换 + 针孔投影
             → 遮挡 / 深度边缘剔除 → RGB 采样 → 体素时间中位着色
```

| 模块 | 文件 | 职责 |
| --- | --- | --- |
| `ImageSync` | `image_sync.hpp` | 在图像缓存中选与扫描窗口最匹配的一帧；**优先曝光中点**，到达时间仅兜底，`max_offset` 内找不到则放弃本帧着色 |
| `LidarPoseInterpolator` | `pose_interpolation.hpp` | 按每个点的时间戳插值 `T_WL`，把点搬到图像曝光时刻的世界系（不做这一步会拖影） |
| `Projector` | `projection.hpp` | `P_C = T_CL·P_W`，针孔投影 + 正深度 / 视野余量检查 |
| `OcclusionChecker` | `occlusion_check.hpp` | 像素级 z-buffer：被前景遮挡的点、深度不连续边缘的点不染背景色 |
| `RgbAssigner` | `rgb_assignment.hpp` | 双线性采样 BGR；按 `quality` 决定是否覆盖旧颜色（保留最佳观测） |
| `VoxelColorMedian` | `voxel_color_median.hpp` | 1 cm 体素内保留最近若干帧观测，取 RGB 分量中位数，抗椒盐 |
| `ColorizerPipeline` | `colorizer_pipeline.hpp` | 上述步骤的编排入口 |

## 3. 关键设计

- **按点时间戳做运动补偿**：Livox 帧内点时间跨度可达 100 ms，用帧末位姿统一投影会在
  运动时产生数像素到数十像素的错色；必须逐点插值。
- **遮挡与深度边缘剔除**：只做投影不做遮挡检查，前景轮廓会染上背景色；深度边缘处同理。
- **质量保留而非覆盖**：同一地图点被多帧观测时，用 `quality` 比较决定是否覆盖，
  避免一张过曝/过暗的图把此前的好颜色洗掉。
- **体素时间中位**：同一 1 cm 体素内的多次颜色观测取分量中位数，比"最后一帧覆盖"
  稳定，能明显压掉椒盐噪点（对照实验见根 [README](../README.md) 与
  [docs/](../docs/) 的点云质量指标）。

## 4. 与其他包的关系

```
pointcloud_colorizer   ←  super_livo_visual_a（方案 A）
                       ←  super_livo_visual_b（方案 B）
```

两个方案都通过 `ament_target_dependencies(... pointcloud_colorizer)` 链接本包，
节点里只负责把「点云 + 每点时间戳 + 图像缓存 + 位姿缓存」喂给 `ColorizerPipeline`。
着色与视觉约束互不影响：视觉门控关闭时彩色点云照常输出。

## 5. 编译

```bash
cd <workspace>
colcon build --packages-select pointcloud_colorizer
```

依赖：`rclcpp`（包导出用）、`Eigen3`、`OpenCV`（`libopencv-dev`）。
