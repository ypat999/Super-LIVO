# Super-LIVO
Integrate Super-LIO and SVO under FAST-LIVO2 framework

## 子模块（third_party）

依赖代码以 git submodule 方式固定在 `third_party/` 下，克隆时需递归拉取：

```bash
git clone --recursive https://github.com/ypat999/Super-LIVO.git
# 已克隆过的主仓库：
git submodule update --init --recursive
```

| 路径 | 上游仓库 | 分支 | 固定提交 |
| --- | --- | --- | --- |
| `third_party/FAST-LIVO2-ROS2` | https://github.com/U-AMC/FAST-LIVO2-ROS2.git | main | `a09599a` |
| `third_party/Super-LIO` | https://github.com/ypat999/Super-LIO.git | ros2 → no-de-check → exp/degeneracy-adaptive | `326ed21`（工作区检出 `67c0acd`） |
| `third_party/basalt` | https://github.com/VladyslavUsenko/basalt.git | master | `0f3b2b5` |
| `third_party/rpg_svo_pro_open` | https://github.com/uzh-rpg/rpg_svo_pro_open.git | master | `ca371f3` |
| `third_party/direct_visual_lidar_calibration` | https://github.com/koide3/direct_visual_lidar_calibration.git | main | `02a0dc0` |
| `third_party/livox_camera_calib` | https://github.com/yeyiru/livox_camera_calib_ros2.git | ros2 | `fb0dd4c` |
| `third_party/livox_camera_lidar_calibration` | https://github.com/DGUCCL7103/Livox_Camera_LiDAR_Calibration_ROS2.git | main | `cdb7f73` |

说明：后三个为相机-雷达外参标定工具（ROS2 版）。Super-LIO 为自有 fork，实验改动在
`no-de-check` / `exp/degeneracy-adaptive` 分支上推进（beam_err/dept_err 距离自适应方差、
K=8 拟合邻域、平面残差软降权等）；其余子模块保持上游原始代码。

## 相机-雷达外参标定流程

三个标定子模块按"板子初标 + 大场景精修"两步使用（标定板轻微弯曲不影响最终精度，
精修环节不依赖板子）：

1. **初标**：`livox_camera_lidar_calibration`（棋盘角点法）获取大致外参；
2. **精修**（二选一）：
   - `livox_camera_calib`：Livox 非重复扫描专用，场景边缘像素级点对线对齐，
     支持多场景联合优化，带 rough calibration 兜底坏初值；
   - `direct_visual_lidar_calibration`：全自动直接视觉-雷达配准，**无需初值**，
     板子弯曲严重时可直接用它跳过初标；
3. 精修结果写回 SLAM 配置外参，用彩色点云与图像边缘对齐验证（根目录 preview 图流程）。

注意：`direct_visual_lidar_calibration` 自带子模块（iridescence 等），需
`git submodule update --init --recursive` 递归拉取后才能编译。

## 方案目录

设计说明见 [gpt双方案设计.md](gpt双方案设计.md)。两套方案**均已实现算法并接入 ROS2 节点**，
在 NTU VIRAL `eee_03` 上完成评测：

| 目录 | 方案 | 内容 |
| --- | --- | --- |
| [solution_a_lightweight](solution_a_lightweight/README.md) | A：极致轻量化 | Super-LIO + FAST/PyrLK + 2D-3D 重投影 + 退化门控；独立 `pointcloud_colorizer`（A/B 共用，含 1cm 体素时间中位着色抗椒盐） |
| [solution_b_tight_coupled](solution_b_tight_coupled/README.md) | B：紧耦合 | Patch 前端 + 特征生命周期/关键帧地图 + MSCKF 多帧约束 + 统一 ESIKF + 延迟地图提交 |

## 仓库结构与主要文档

| 路径 | 内容 |
| --- | --- |
| [docs/HANDOVER_2026-11-08.md](docs/HANDOVER_2026-11-08.md) | **工作主线交接文档**：§16 总纲、§17 按帧染色诊断链、§18/§20 官方图像消融、§19 RTK-SLAM 真值评测 |
| [docs/](docs/) | 专项审计：地图更新机制（MAP_UPDATE_*）、OctVox 审计、去畸变管线、点云质量指标、数据清单（DATA_INVENTORY）等 |
| [tools/](tools/) | 评测与取数脚本：`odom_vs_gt.py`（官方协议 ATE）、`traj_compare.py`、着色一致性/耗时/平面厚度等 |
| [experiments/](experiments/) | 成对消融实验：`fl2_img_ablation`（官方 FAST-LIVO2 有/无图像 ×3 轮）、`octvox_ablation`（地图表示消融） |
| [speed_test.md](speed_test.md) | RK3588 实机测速报告（Super-LIO / FAST-LIVO2 / 方案 A/B，绑核公平对比） |
| 根目录 `*.png` / `*.html` | 各数据集点云预览图与诊断报告（如 `fastlivo2_bag_yaw_drift_report.html`） |

## 关键实验结论（摘要）

- **NTU eee_03 ATE**：官方 LIVO 0.028 m / 官方纯 LIO 0.043 m；Super-LIO 0.138→0.130 m
  （观测模型对齐后）。剩余差距主因是地图表示（0.25 m 子体素均值压缩）的内存/速度取舍，不建议改。
- **RTK-SLAM Dataset**（全站仪 <1cm 真值，不做 SE(3) 对齐）：Super-LIO 绝对 ATE 0.347 m
  vs 官方 FAST-LIO-SAM 0.374 m；无累积漂移，误差主要是子图刚体偏置而非逐帧抖动。
- **图像消融**（§20）：官方 `img_en=1/0` ×3 轮判决——图像确实参与位姿（厘米级），稳态精度增益≈0，
  有效作用是"锚定/防跑飞"。
- **自采 2026-10-30 bag**：三种跑法（Super-LIO / FAST-LIVO2 纯 LIO / LIVO）均存在约 17° yaw
  漂移重影，源于 IMU 陀螺零偏（gz≈−0.42°/s）+ 回环未纠正，属数据侧问题；诊断见
  [fastlivo2_bag_yaw_drift_report.html](fastlivo2_bag_yaw_drift_report.html)。
- **Retail_Street（HKU 公开数据集）**：必须用官方标定 `camera_pinhole.yaml`（fx≈1293.6）+
  官方 `avia.yaml`（`img_time_offset: 0.1`）；bag 自带 `avia_lvi.yaml`（fx≈906、
  `img_time_offset: 1200.25`）是 DeepRobotics 车载配置，误用会导致 RGB 严重错位。
  标定文件与来源注释已归档在 bag 目录 `/home/ywj/slam_data/Retail_Street/`。
