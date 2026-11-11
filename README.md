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
| `third_party/FAST-LIVO2-ROS2` | https://github.com/ypat999/FAST-LIVO2-ROS2.git（fork 自 U-AMC/FAST-LIVO2-ROS2） | main | `b9f931f` |
| `third_party/Super-LIO` | https://github.com/ypat999/Super-LIO.git | ros2 → no-de-check → exp/degeneracy-adaptive | `326ed21`（工作区检出 `67c0acd`） |
| `third_party/basalt` | https://github.com/VladyslavUsenko/basalt.git | master | `0f3b2b5` |
| `third_party/rpg_svo_pro_open` | https://github.com/uzh-rpg/rpg_svo_pro_open.git | master | `ca371f3` |
| `third_party/direct_visual_lidar_calibration` | https://github.com/koide3/direct_visual_lidar_calibration.git | main | `02a0dc0` |
| `third_party/livox_camera_calib` | https://github.com/yeyiru/livox_camera_calib_ros2.git | ros2 | `fb0dd4c` |
| `third_party/livox_camera_lidar_calibration` | https://github.com/DGUCCL7103/Livox_Camera_LiDAR_Calibration_ROS2.git | main | `cdb7f73` |

说明：后三个为相机-雷达外参标定工具（ROS2 版）。Super-LIO 与 FAST-LIVO2-ROS2 为自有 fork：
Super-LIO 的实验改动在 `no-de-check` / `exp/degeneracy-adaptive` 分支上推进（beam_err/dept_err
距离自适应方差、K=8 拟合邻域、平面残差软降权等）；FAST-LIVO2-ROS2 的 fork 在上游 `a09599a`
之上有两批改动：
① `ae28b0c` 修三处功能缺陷——`blind_sqr` 分支未赋值、MID360 话题订阅缺失、发布 odom 的时间戳
改用 lidar 帧时刻（原为系统时间，评测时会被 evo 判成时间轴错位）；
② `b9f931f` 加**滚动淘汰视觉地图点**（`pruneVisualMap`），解决视觉地图随时间线性增长导致长序列
OOM 的问题，详见「关键实验结论」。其余子模块保持上游原始代码。

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
| [solution_a_lightweight](solution_a_lightweight/README.md) | A：极致轻量化 | Super-LIO + FAST/PyrLK + 2D-3D 重投影 + 退化门控（消费 `/lio/degeneracy` 真度量）+ 状态回灌（`update.reinject`） |
| [solution_b_tight_coupled](solution_b_tight_coupled/README.md) | B：紧耦合 | Patch 前端 + 特征生命周期/关键帧地图 + MSCKF 多帧约束 + 统一 ESIKF + 延迟地图提交；LiDAR 地图绝对锚定 + 历史锚点保持 + 退化感知先验与守卫 |
| [pointcloud_colorizer](pointcloud_colorizer/README.md) | 共用组件 | 彩色点云独立包（A/B 共用，与定位解耦；逐点运动补偿 + 遮挡剔除 + 1cm 体素时间中位抗椒盐） |
| [common/super_livo_common](common/super_livo_common/README.md) | 共用组件 | A/B 共用的纯算法头文件库：退化签名映射、各向异性先验、重投影测量方差（口径只改一处） |

## 与官方 FAST-LIVO2 的差异

先说定位：A/B **不是从 FAST-LIVO2 改出来的**，而是挂在 Super-LIO 外部的视觉节点；官方
FAST-LIVO2 在这里是"同问题、不同解法"的对照系。三者的核心分歧不在估计器（都跑 ESIKF），
而在**视觉观测怎么构造、以谁为锚点**：

| 维度 | 官方 FAST-LIVO2 | 方案 A（轻量） | 方案 B（紧耦合） |
| --- | --- | --- | --- |
| 基座 | 自带 LIO+VIO 一体的 ESIKF | Super-LIO（复用其 ESIKF） | Super-LIO（复用其 ESIKF） |
| 视觉观测形式 | LiDAR 地图点投影到图像，取 **patch 光度误差**（直接法） | 角点（FAST + PyrLK）+ LiDAR 深度关联，**2D-3D 重投影误差**（几何法） | 同 A 的几何法，另加 **MSCKF 多帧零空间约束** |
| 视觉锚点来源 | 全局 LiDAR 地图点（地图漂了锚点就漂） | 全局 LiDAR 地图点（同 FL2，深度来自地图） | 地图点 **+ 健康期建立的持久锚点**（跨代绝对锚定，`anchors_` 只在成功观测时刷新 last_seen） |
| 是否依赖全局地图 | 是 | 是 | 部分否（MSCKF 用多帧位姿三角化，不依赖地图） |
| 退化处理 | 无显式退化检测，持续融合 | 消费 `/lio/degeneracy`，退化时才注入 + 状态回灌（`update.reinject`） | 消费 `/lio/degeneracy`，cond/sev 映射成先验注入强度，另加先验否决/速度限幅守卫 |
| 先验信息上限 | 无显式上限 | 有 | 有（`visual_info_ratio` 常态 3× / 退化 10×）+ σ 地板，防止一帧压塌协方差 |
| 视觉计算量（RK3588 单大核） | LIO 71 + VIO 28 ≈ 99 ms/帧 | 视觉合计 21.9 ms（std 4.7） | 视觉合计 41.8 ms（std 27.2，修复前口径，见下） |
| 真值精度（NTU eee_03 ATE） | **0.028 m**（最强） | 未单列（对位姿几乎无纠正力） | 见 §9.4，退化段方向稳定 |

一句话概括：

- **FL2 把视觉当"地图的稠密光度校正"**——精度天花板最高，代价是计算最重、且对 LiDAR 地图
  退化没有专门的对抗手段（地图一漂，光度残差就跟着错）。
- **A 把视觉当"退化时的稀疏几何补充"**——代价最小，但走廊段可用重投影约束太少
  （实测 n=0~33），对位姿几乎无纠正力。
- **B 把视觉当"不依赖地图的多帧几何约束 + 跨代绝对锚点"**——纠正力最强且方向稳定，
  代价是计算最重、实时性临界。

## RK3588 实机对比测试（怎么测才算公平）

平台：RK3588S（A76×4 = 核 4-7，A55×4 = 核 0-3），Ubuntu 22.04 + ROS2 Humble，Release
（`-O3 -march=native`）。完整报告见 [speed_test.md](speed_test.md)。

**1. 绑核（必须与实机 launch 一致，否则数字没有可比性）**

| 进程 | 绑定 |
| --- | --- |
| `ros2 bag play`（顶替 Livox 驱动） | `taskset -c 4` |
| `super_lio_node` | `taskset -c 7` + `lio.single_core:=true` |
| `fastlivo_mapping` | `taskset -c 7` + `OMP_NUM_THREADS=1` |
| `visual_a_node` / `visual_b_node` | `taskset -c 7`（**与 LIO 同挤一个大核**） |
| `parameter_blackboard`（FL2 相机参数托管） | `taskset -c 0` |

组间留 ≥15 s 间隔降温；每组**至少重复 3 次**（本仓库已实测：裁剪输入下 Super-LIO 本身
不可复现，单跑数字会骗人）。

**2. 四档对照**

```bash
bash /tmp/bench/run1_slio.sh    # ① Super-LIO 单跑（基线）
bash /tmp/bench/run2_fl2.sh     # ② 官方 FAST-LIVO2（+ parameter_blackboard）
bash /tmp/bench/run_ab.sh a     # ③ Super-LIO + 方案A（同核）
bash /tmp/bench/run_ab.sh b     # ④ Super-LIO + 方案B（同核）
python3 /tmp/bench/parse_bench.py slio.log fl2.log vis_a.log vis_b.log <bag时长>
```

**3. 指标口径**（三者必须同口径，否则不可比）

| 指标 | 定义 |
| --- | --- |
| odom 发出延迟 | 雷达消息到达回调（steady 墙钟）→ 该帧 odom publish 完成的墙钟差；含排队与线程调度 |
| 整帧处理时间 | LIO 帧全流程 CPU 耗时：Super-LIO `[FrameProcess]`、FL2 `[LIOTotal]` / `[VIOTotal]`、A/B 视觉 `合计` |
| 波动 | 全体样本 std + P95/max（A/B 视觉是 ~1 s 一条采样，不是全帧） |

**4. 三个必须注意的坑**

- **B 的实时性数字已过期**：[speed_test.md](speed_test.md) 里 B 视觉 41.8 ms、约束
  22.7±26.6 ms 是**修复选轨 O(n²) 之前**测的（当时单帧最坏 422 ms）。修复后选轨 max 已降到
  2.1 ms，B 是否仍"单大核实时临界"必须重测才能下结论。
- **FL2 在单大核上跑不满实时**：处理需求 ≈0.99 帧周期，长序列必然积压，odom 延迟退化到
  秒级（均值 5.3 s、峰值 20 s）。所以"在 RK3588 上测 FL2"要么放开绑核/降帧频，要么明确
  标注"该组为积压态，延迟指标不代表算法耗时"，否则对比不公平。另外其视觉地图原本常驻累积、
  长序列会 OOM，现已在 fork `b9f931f` 用滚动淘汰修掉（详见"关键实验结论"）。
- **不要只看延迟**：A/B 的价值在退化段，健康段应与 LIO 逐点贴合（`|B−LIO|` p50 ≈ 2~5 cm）。
  建议的判据组合是「延迟分布 + 同 bag 逐点差 + 退化段终点误差多跑分布」三者一起看，
  任何单一指标都能给出误导性结论。

## 仓库结构与主要文档

| 路径 | 内容 |
| --- | --- |
| [docs/HANDOVER_2026-11-08.md](docs/HANDOVER_2026-11-08.md) | **工作主线交接文档**：§16 总纲、§17 按帧染色诊断链、§18/§20 官方图像消融、§19 RTK-SLAM 真值评测、§22 滚动淘汰视觉地图点（99.3% 覆盖 / ATE 0.561 m）与三层点云偏差结果 |
| [docs/](docs/) | 专项审计：地图更新机制（MAP_UPDATE_*）、OctVox 审计、去畸变管线、点云质量指标、数据清单（DATA_INVENTORY）等 |
| [tools/](tools/) | 评测与取数脚本：`odom_vs_gt.py`（官方协议 ATE）、`traj_compare.py`、着色一致性/耗时/平面厚度等；RTK-SLAM 参考跑批链路 `run_fl2_rtk.sh` / `run_rtk.sh`（经 `ros_batch_guard.sh` 做互斥与"只杀同批"收尾）+ `fl2_pcd_merge.py`（合并 pcd 分片）+ `rtk_traj_prep.py` / `rtk_pcdev.py`（轨迹导出与点云偏差） |
| [experiments/](experiments/) | 成对消融实验：`fl2_img_ablation`（官方 FAST-LIVO2 有/无图像 ×3 轮）、`octvox_ablation`（地图表示消融；`10~13_*.sh` 为 LIO 核 + 方案A 上色对比跑批） |
| [freqcmd.txt](freqcmd.txt) | 换环境/换机常用命令清单：依赖、构建、主链路跑批、测速方法与已知坑 |
| [speed_test.md](speed_test.md) | RK3588 实机测速报告（Super-LIO / FAST-LIVO2 / 方案 A/B，绑核公平对比） |
| 根目录 `*.png` / `*.html` | 各数据集点云预览图与诊断报告（如 `fastlivo2_bag_yaw_drift_report.html`） |

## 关键实验结论（摘要）

- **NTU eee_03 ATE**：官方 LIVO 0.028 m / 官方纯 LIO 0.043 m；Super-LIO 0.138→0.130 m
  （观测模型对齐后）。剩余差距主因是地图表示（0.25 m 子体素均值压缩）的内存/速度取舍，不建议改。
- **RTK-SLAM Dataset**（全站仪 <1cm 真值，不做 SE(3) 对齐，`construction_seq2` 整轮 16 个 GT 点全覆盖）：
  `super-lio` 0.347 m / `fast-lio-sam` 0.374 m / `super-lio-beam3` 0.508 m / **官方 FAST-LIVO2 0.561 m** /
  `super-lio-beam` 0.656 m。fl2 的 SE(3) 误差 0.091 m 是最低的一档——"形状准、绝对位置不准"；
  其余系统无累积漂移，误差主要是子图刚体偏置而非逐帧抖动。
- **三层点云偏差（`tools/rtk_pcdev.py`）**：以 fl2 的稠密建图为参考面网（39457 面 / 覆盖 26.7%），
  量各系统点的 L1 距离分层散布、L2 跨帧重影、L3 绝对刚体偏差。L1 出斜率（如 fl2 σ0=0.96 cm、
  σθ=0.134°，beam3 σθ=0.282°）；**L3 目前分辨率只到 ~10 cm**（参考面厚度上限 8 cm 主导），
  各系统法向 MAD 7.7–10.6 cm 全落在自比对刻度 9.53 cm 的 0.81–1.11 倍内，**不能据此排序**；
  L2 样本稀疏且非单调，暂不结论。要让 L3 可排序需换更薄的面或改点到最近点的法向距离。
- **官方 FAST-LIVO2 的内存坑已解决（`b9f931f`）**：开图像时原实现会让每个视觉地图点钉住一帧
  完整原图（`FEATURE::img_`），1.92 MB/帧 × 16.5 Hz ≈ 32 MB/s，加上 VisualPoint 自身增长 ≈
  16 MB/s，合计 ≈48 MB/s 无界涨；`img_en=1` 51.8 MB/数据秒 vs `img_en=0` 4.3 MB/数据秒，
  12 倍差全在视觉侧。`pcd_save.interval` 分片只保住已 flush 的地图，**对这部分内存无效**
  （旧轮 46 分片仍在 480 s / 80% 被 OOM）。现在 `pruneVisualMap` 在每次分片落盘后以当前帧时刻
  为 cutoff 删掉已落盘点（连带释放图像）并清空 `warp_map`，当前帧点留到下一轮匹配：本机 23 GB
  可跑满 **595.5/599.5 s = 99.3%**，58 分片对应 58 次 prune，合并 24.88M → 8.92M 点 @5 cm。
  代价是视觉地图只保留最近一片（约 200 帧 / 12 s），**长时视觉重观测能力被放弃**；
  若要做长序列回环需改成按空间距离淘汰。
- **图像消融**（§20）：官方 `img_en=1/0` ×3 轮判决——图像确实参与位姿（厘米级），稳态精度增益≈0，
  有效作用是"锚定/防跑飞"。
- **LiDAR 退化对抗**（NTU eee_03 走廊裁剪，`Y_HALF=1.2` 温和退化，B 侧 8 次重复单跑
  tag=y12fix2~y12fix9）：B 方案（绝对锚定 + 历史锚点保持 + sev 映射 + 守卫）**8/8 次不劣于
  同 bag LIO**，且把终点误差的跨跑散布从 LIO 的 14.1~794.4 m（55×）压到 B 的 8.4~127.8 m
  （15×）——视觉介入不仅降低误差，还压缩了不可复现性本身。幅度只按分布口径给：均值
  71.4 vs 267.7 m（−73%）、中位数 84.9 vs 170.3 m（−50%）、逐次比值 0%~87%（中位 41.5%）；
  **单次数字不可复现，禁止再引用"437→123 m / 72%"这类单跑结论**。选轨耗时已修复
  （视差扫描 O(n²)：`kMaxObservations=16` 截断 + 视差缓存），8 次跑 `>50 ms 帧数 = 0/176~178`。
  A 方案状态回灌链路已通但对位姿无纠正力（同 bag |A−LIO|≤0.7 m），瓶颈在约束质量（走廊段
  大量帧判"健康不更新"/rmse 拒绝，可用重投影约束 n=0~33 太少）。纪律：不自造退化检测，只
  消费 Super-LIO 的 `/lio/degeneracy`；Super-LIO 在裁剪输入下不可复现（同输入终值随机），
  一切对照只看同 bag 内逐点 |VIS−LIO| 或多跑分布。细节见两份方案 README 的退化章节。
- **自采 2026-10-30 bag**：三种跑法（Super-LIO / FAST-LIVO2 纯 LIO / LIVO）均存在约 17° yaw
  漂移重影，源于 IMU 陀螺零偏（gz≈−0.42°/s）+ 回环未纠正，属数据侧问题；诊断见
  [fastlivo2_bag_yaw_drift_report.html](fastlivo2_bag_yaw_drift_report.html)。
- **Retail_Street（HKU 公开数据集）**：必须用官方标定 `camera_pinhole.yaml`（fx≈1293.6）+
  官方 `avia.yaml`（`img_time_offset: 0.1`）；bag 自带 `avia_lvi.yaml`（fx≈906、
  `img_time_offset: 1200.25`）是 DeepRobotics 车载配置，误用会导致 RGB 严重错位。
  标定文件与来源注释已归档在 bag 目录 `/home/ywj/slam_data/Retail_Street/`。
