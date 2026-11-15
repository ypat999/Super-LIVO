# 实验记录总账（exp_records）

> 本文件汇总仓库**所有实验**的前提、流程、结论与产物，每条结论均保留文档/日志中的原始数字。
> 详细论证见 `docs/HANDOVER_2026-11-08.md`（§编号）与 `docs/` 专项文档；本文件是索引级总账。
> 最近更新：2026-11-15（本轮补录 §25–§29：外参标定链 holefix 与远场验证、RK3588 绑核四档对照、B 退化对抗 8 跑、move bag 幽灵层归因、FAST-Calib-ROS2 fork 治理；上轮补录楼梯井诊断 M.0–M.40 全系列与 HANDOVER §19.11–§22，见 §16–§24）。

## 0. 通用实验前提与约定（适用于以下所有条目）

- 对照组之间保持数据、轨迹、地图表示一致，**只改一个变量**；离线几何实验固定同一条轨迹。
- 无外部真值时，不得宣称"绝对精度提升"，只表述相对差异与几何一致性。
- ATE 对比必须同 bag、同轮次；跨轮次比较不判决（实测轮次噪声可达 2.6 倍）。
- FAST-LIVO2 输出不能当真值（其两轮自差异 P50 73.9 mm / P90 1763.7 mm）。
- 地面 RANSAC 拟合只能检出 roll/pitch；**yaw 漂移必须用里程计回环检验**（见 §1 教训）。
- 图片展示约定：先复制到 `D:\traetmp\`，再以 `file:///D:/traetmp/` 展示（Trae 在 Windows 侧读不了 WSL 路径 PNG）。
- 主要数据/输出盘：`/home/ywj/slam_data/`；仓库内产物在 `experiments/`、`docs/`；`/tmp/*` 为临时目录，随 WSL 重启清空。
- 单变量矩阵开跑前先做 40 s 冒烟，两臂 md5 必须不同；否则先怀疑"编辑没落盘"（M.35 事故）。
- 参数覆盖必须用完整前缀（如 `lio.kf.kf_max_iterations`，不是 `lio.kf_max_iterations`）；`-p` 后去日志确认 `[Param]` 真的打印，否则整组消融是空操作。
- 从 bag 导出的时间序列先做单调性检查，再谈任何"帧间"量（§19.12）；逐步指标一律按 `header.stamp` 排序，不用录制时刻。

---

## 1. 自采 2026-10-30 bag：三算法建图对比与 yaw 漂移诊断（2026-11 上旬）

**前提/动机**：用 FAST-LIVO2-ROS2 与 Super-LIO 跑 WSL 中的自采包，检查保存的 PCD 是否歪斜。
数据 `/home/ywj/slam_data/rosbag2_fastlivo2_2026_10_30-18_00_25`：160.3 s、35239 条
（lidar `sensor_msgs/PointCloud2` 1602 / imu 32033 / `/camera/color/image_raw` 1604，注意非 CustomMsg）。
轨迹为"从一个大房间出发、出去绕一圈回到该房间"，天然带里程计回环一致性检验条件。
（更早的 `livox_record_noshell` 包同样跑法，产物在 `pcd/fl2_livox_bag/`、`pcd/superlio_livox/`，不属于本包。）

**流程**：
1. FAST-LIVO2 纯 LIO（lidar+imu，`img_en=0`）→ `fl2_bag_lio/`；
2. FAST-LIVO2 完整 LIVO（`img_en=1`）→ `fl2_bag_livo/`；
3. Super-LIO → `superlio_bag/`；
4. 重放 bag + 视觉着色实验：`replay_bag_0.db3`、`replay_odom.txt`、`replay_a_official.pcd`（283 MB / 17,660,480 点）、`replay_a_slio.pcd`（282 MB / 17,632,340 点）；Super-LIO 视觉着色系列 `super_livo_visual_a_colored.pcd`（250 MB / 15,595,594）、`_prefix`（284 MB / 17,730,607）、`_v2_012`（55 MB / 3,455,542）、`b_colored`（258 MB / 16,093,804）及 before_fix 两件；
5. 质检：open3d RANSAC 地面拟合（roll/pitch）→ 改为离群点检测失效后，补里程计回环 yaw 检验 + 静止段 IMU 零偏积分预算闭合。

**结论**：
- 首轮仅用 roll/pitch 地面拟合判定"不歪"——**该结论作废**：方法检不出 yaw。
- 回环检验：绕场 108 m 回到起点，位置差仅 0.99 m，但 **yaw 差 17.2°**；纯 LIO −17.2°/−18.4°、LIVO −17.0°/−19.4°（两次起点扫描）。三种跑法的全部点云都存在同源 ~17° yaw 漂移重影。
- 误差预算闭合：静止段 IMU 零偏 gz≈−0.42°/s、gy≈−0.88°/s；17.2°/147 s ≈ 0.12°/s，与零偏+无回环纠正的量级一致。
- 判定：**数据侧问题**（IMU 陀螺零偏未被回环纠正），非算法缺陷；LIVO 保存的 `all_raw_points.pcd`（72 MB / 4,514,544 点）远少于纯 LIO（927 MB / 28,969,845 点）是 `img_en=1` 时只存被着色点的保存策略所致。Super-LIO：`test_ori.pcd` 662 MB / 41,374,634 点、`filtered_test_ori.pcd` 244 MB / 15,260,712 点、PCD/scans_0~3。

**产物**：输出目录见上（均在 `/home/ywj/slam_data/pcd/`）；预览图 10 张在仓库根目录（已镜像 `D:\traetmp\`）；诊断报告 [fastlivo2_bag_yaw_drift_report.html](fastlivo2_bag_yaw_drift_report.html)、`third_party/Super-LIO/stairwell_drift_diagnosis.txt`。

---

## 2. Retail_Street（HKU 公开数据集）：标定求证与 RGB 点云对比（2026-11 上旬）

**前提/动机**：下载公开数据集 `/home/ywj/slam_data/Retail_Street`（1.99 GB、135.47 s、30157 条；`/livox/lidar` CustomMsg 1355 帧≈10 Hz、`/livox/imu` 27447 帧≈200 Hz、`/left_camera/image` 1355 帧），要求在线找到该数据集真正的标定，用 FAST-LIVO2 出 RGB 点云、Super-LIO 出普通点云。

**流程**：
1. 第一轮误用 bag 自带 `avia_lvi.yaml` + 内参 fx≈906（RealSense 口径）→ RGB 糊成色块、完全对不上。结果保留作对照（`retail_street_livo_wrongcalib/`）。
2. 在线检索确认：`avia_lvi.yaml` 是 DeepRobotics 车载配置（`img_topic:/oak/rgb/image_raw`、雷达名带序列号、`img_time_offset: 1200.252661`、`blind 1.0`、`acc_cov 0.8 / gyr_cov 0.5 / b_*_cov 0.01`），**不适用本包**。
3. 官方 HKU 标定：`camera_pinhole.yaml` fx 1293.569 / fy 1293.316 / cx 626.914 / cy 522.799（1280×1024，scale 0.5，畸变 −0.0762, 0.1230, −0.00113, 0.000251）；官方 `avia.yaml`：`img_topic:/left_camera/image`、`img_time_offset: 0.1`、`lidar_type 1`、`blind 0.8`、`filter_size_surf 0.1`、`acc_cov 0.5 / gyr_cov 0.3 / b_acc_cov b_gyr_cov 0.0001`、`min_eigen_value 0.0025`、`extrinsic_T [0.04165, 0.02326, −0.0284]`。两份文件连同来源注释存入 bag 目录。
4. 时间戳实测（`retail_stamps.py` 读 bag 首帧）：lidar−image = **+0.000000 s**、lidar−imu = −0.011431 s → 官方 `0.1` 合理；若照抄 1200.252661 会把图像时刻整体 +20 分钟，VIO 永远取不到匹配状态。`img_time_offset` 的用法依据：`msg_header_time = stamp2Sec(header.stamp) + img_time_offset`（`LIVMapper.cpp` L1056），属时钟域补偿。
5. 官方标定重跑 LIVO（`parameter_blackboard` + `fastlivo_mapping`，`pcd_save.pcd_save_en:=true`、`interval:=-1`，`ros2 bag play -r 1.0`，SIGINT 等保存）；Super-LIO 同包跑普通点云。输出重命名并挪至 `/home/ywj/slam_data/`。

**结论**：
- 官方标定修复 RGB：`retail_street_livo_wall_check.png` 左（旧标定）/右（官方标定）对比，红色招牌、白字、蓝玻璃清晰可辨。
- LIVO（官方标定）≈**1890 万点**，轨迹长度 67.5 m（1352 帧）；Super-LIO **2082 万点**。
- 地面几何两者一致（街道本身有坡）：地面拟合 RMS 7.5 cm（Super-LIO）/ 31 cm（LIVO）。

**产物**：`/home/ywj/slam_data/pcd/retail_street_livo_official/`、`retail_street_superlio/`、`retail_street_livo_wrongcalib/`（对照）；标定文件 `/home/ywj/slam_data/Retail_Street/{avia.yaml, camera_pinhole.yaml}`；预览图 `experiments/retail_street_*.png`（含 `retail_bk_white_*.png`，镜像 `D:\traetmp\`）。

---

## 3. 几何数据集前置验证（任务 1/2/3，2026-11-10）

**前提**：为 OctVox 消融与染色实验建立可信数据底座（逐帧 body 云、可复现重放、去畸变时间链）。

**流程与结论**：
- `docs/BODY_CLOUD_VALIDATION.md`：逐帧 body 云 topic `lio/cloud_body_dense` 默认持续发布（`visual_a_node.cpp:285` 空串默认），与 odom 同回调；rosbag 行级 sqlite 戳与 `header.stamp` 差 ~8.5×10⁸ s，**一切对齐以 header.stamp 为准**。
- `docs/REPLAY_REPRODUCIBILITY.md`：重放可复现历史 sl odom，run-to-run 噪声 ~0.5 cm/s（窗口末端 13 cm/25 s）。
- `docs/DESKEW_PIPELINE_AUDIT.md`：`header.stamp − timebase ≡ 160.000000003 s`（120/120 帧）；`offset_time` 范围 0→99.92 ms、每帧恰 1 个点为 0（0.0042%）、每线严格单调（120/120）、相邻间隔 ≈25 µs、6 线 4167 ns 相位交错、帧间 header 间隔 99.82~100.7 ms；单帧 `point_num=24000`=6×4000。

**产物**：`/home/ywj/slam_data/geom_replay_20261110/replay3/`（1345 lidar / 27242 imu / 1307 dense / 1315 odom；评价窗 245=245=245 帧 1:1）+ `live_probe_out.json`、`cmp_curve3.csv`、`cmp_dp3.csv`；凭据 commit `1f370532…`/`67c0acd4…`、二进制 md5 `00e97f6a067a8167113af60dcb2e245d`、`r3live.yaml` sha256 `632e5721…e1d2a1`。

---

## 4. OctVox 地图代表点更新策略消融（2026-11-10，阶段一离线 + 阶段二闭环）

**前提/动机**：验证增量平均是否造成局部几何损失；重点验证"每个子体素保留距子体素几何中心最近的真实观测点"。方案来源 `experiments/superlio实验.md`。分组：A 增量均值（`rep_mode=0`，基线）、B 首次观测保留（`rep_mode=2`）、C 中心最近真实点（`rep_mode=1`，ε=0）、D 全量观测累积（仅离线）。只改 `rep_mode`，不碰 ESKF/KNN/损失函数；无独立 GT 不宣称绝对精度。

**数据与配置**：Retail_Street；离线重放 `geom_replay_20261110/replay3`；分析窗 946685507.0–946685531.5 共 245 帧；父体素 0.5 m / 子体素 0.25 m、`merge_dist=0.1 m`、K=8、计数上限 20。

**流程**：`experiments/octvox_ablation/01_build_maps.py → 02_evaluate.py → 03_controls.py → 04_export_pcd.py`；闭环阶段 `05_closed_loop.sh`（约 7 min，三组独立进程）+ `06_analyze_closedloop.py [--full]`。

**结论（阶段一，离线固定位姿）**：
- M1 局部平面厚度 P50：A 14.48 / B 17.24 / C 20.63 / D 10.68 mm；RMS 46.84 / 49.36 / 50.52 / 27.48 mm。
- M2 留出帧残差 paired P50（5 s 块）：A 12.69 / B 13.80 / C 13.53 / D 10.27 mm；2 s 块 11.34 / 12.29 / 12.25 mm。Wilcoxon：B−A +0.346 mm（p=3.0e-10）、+0.642 mm（p=1.2e-14）显著更差；C−A p=0.644/0.575 不可区分。
- M3 以 D 为几何参考：`off5cm` A 0.603% vs B 0.168% / C 0.193%（3.6 倍）；`off_surface` P90 A 38.45 / B 21.81 / C 19.30 mm。
- M5 中心偏差：到子体素中心距离 P50 130.7→107.7 mm（−23.0）；沿法向位移 P90 24.94→40.82 mm（+64%）。
- M4 输入基线：多帧子体素占比 100%，每子体素观测 P50 8 / P90 40 / max 101；子体素内 RMS 离散 P50 90.2 mm（比组间 10–20 mm 差 4–9 倍）。
- 控制实验：反序/随机重放后代表点变化 >1 mm 比例 B 77.2%/63.7%、C 40.2%/31.9%；C 无密度偏置（来源帧候选数排位 P50 0.500）。

**结论（阶段二闭环，1317 帧）**：`fit_res` P50 A 53.40 / B 61.41 / C 67.95 mm；`match_ratio` 0.764 / 0.760 / 0.691；`plane_fail_frac` 23.39 / 23.43 / 30.11%；`MapInsert` 0.0737/0.0738/0.0757 ms；wall 2:18.74 / 2:18.98 / 2:18.64，RSS 223.3/223.3/222.9 MB，`evict=0`。配对：B−A `fit_res` +7.07 mm（p=6.1e-42）、C−A +13.93 mm、`Δmatch` −0.0700。轨迹互差 24.5 s 内 P50 28.3 / 39.0 mm，全段末帧 11.9 / 6.9 mm。

**不可得结论**：无 GT 不能排"谁更准"；换 `merge_dist` 后未验证。

**产物**：脚本 `experiments/octvox_ablation/01–09`；文档 `docs/OCTVox_UPDATE_AUDIT.md`、`MAP_UPDATE_{AB_COMPARISON,CLOSED_LOOP,EXTERNAL_REFERENCE,EXPERIMENT_SUMMARY}.md`；数据 `/home/ywj/slam_data/octvox_ablation_20261110/`（`inputs.npz`、`map_{A,B,C,D}.npz`、`pcd/map_*.pcd`、`per_frame_residual_5s/2s.csv`）、`octvox_closedloop_20261110/rep_{A,B,C}/`；代码改动 `OctVoxMap.hpp` L126-130。

---

## 5. 按帧染色诊断链（HANDOVER §17，2026-11-11）

**前提/动机**：拆开"彩色 vs 无色"这一最大未对照变量；此前只有整图目视、无按帧数字。指标口径统一为"最近邻颜色 vs 5×5 邻域反色率"（pepper_rate），与 `grid_pepper_rate` 同尺。

**流程**（脚本均在 `experiments/octvox_ablation/`）：`10_run_colored_bc.sh`（B/C 闭环+在线染色）→ `11_run_colored_lio_bench.sh`（LIO 基准+在线染色）→ `12_run_colored_pointlio.sh`（外部 Point-LIO 位姿+在线染色）→ `probe_body_color_v3.py`（离线"世界系体素中位色回投"对照，`12c_run_bodycolor_inference.sh`）。

**结论（pepper_rate %）**：
- B→C 在线 416→451、550→412、495→654、254→138；在线染色普遍高于同轨迹离线反插（416 vs 384.85、550 vs 533.17、495 vs 419.03）。
- **同一条 B 轨迹、位姿不变、只换机制：离线 384.85 → 在线 1013.87（2.6 倍）**；Point-LIO 轨迹上离线 539.97、在线均值 1130.26（≈2.1 倍）。
- LIO 基准 13 帧均值 pepper 581.07、cross 83.46%、grid 6.02、`mean_nn_dist` 4.45 mm。
- SMBU 外几何：A 51.29 mm/542918 点、B 50.20 mm、C 51.47 mm、官方 60.27 mm/385354 点、e1 64.17 mm/377399 点（全部未过 `mean_nn_dist ≤12 mm` 门槛，只作几何参照）。
- 判决：在线链存在不可归因于几何的系统性劣化（`cloud_body` 带帧间去畸变位移 + 逐帧独立 `getNearestColor` 不做空间平滑）；外部位姿实验因位姿不可信作废（`/Cloud` 仅 2139 帧、gap 24–28 ms、14465 次 IMU 积压、"丢 1 帧 lidar msg due to time jitter"；`lidar_meas` 为纯 IMU 外推）；**反色率不能当几何精度指标**（B 几何最好却 pepper 最高）。

**产物**：`colored_{bc,lio,pointlio}/{colored_eval.csv,world_map.pcd}`、`world_body/{A,B,C,official_vio_livo,e1}.pcd`、`world_body_metrics.json`；`/tmp/probe_body_color_v3/`（临时）。

---

## 6. 官方 FAST-LIVO2 图像消融初轮 + 轮次对照修正（§18，2026-11-10~12）

**前提**：本仓库是 Super-LIO 与 FAST-LIVO2 的集成，需确认图像在官方实现里到底贡献多少。配置 `FIFO_SIZE=4/PATCH_SIZE=2`；`vio` vs `only_lidar`（仅改 `VO[enabled]/img_enb/normal_en`）。数据：NTU VIRAL 07/09/13、SMBU a/b/c 全部场景、RTK-SLAM 场景（`/dev/shm` overlay + `--rate 8`）。

**结论**：
- 官方 LIVO 0.0088 / 0.0077 m 远优于官方 LIO 0.0505 / 0.1538 m（RTK 集 LIO-only 崩 8/59）。
- 但 NTU07/09 "加图像反而变差"（16:31 轮 0.0198/0.0151 优于 17:46 轮 0.0274/0.0164）**被撤回**——实为两轮 LIO 轮次差（同代码同 bag 0.0198 vs 0.0505，差 2.6 倍）。§18.2/§18.3 对照表标注不可靠。
- RTK-SLAM 上官方 LIVO 0.0081 vs 本仓库 0.0080 等价（SMBU a/b/c 全部 ±0.0001）。

**产物**：`/home/ywj/slam_data/{ntu_virna_data,fixed_world_map,fastlivo2_smbu_out,fastlivo2_rtk_out}/`。

---

## 7. RTK-SLAM Dataset 真值评测（§19，2026-11-11）

**前提**：仓库内所有 ATE 此前均来自 NTU VIRAL，缺带外部定位真值的数据集。RTK-SLAM（HKU，子集名 SMBU）官方 62 scene；实际 15 个含 `/gnss/fix`（SM-101/102/103/104/201/203/205/206/207/208/210/212/213/303/402）。RTK 戳偏移实测 −0.00269 s（绝对值中位 0.00377 s，可忽略），不做平移、仅时间交集 + 2.0 m 门限；`CustomMsg` 需同时 source `dev_ws`；参数 `scripts/rtk_eval/r3live_rtk.yaml`。

**流程**：`scripts/run_smbu_rtk.sh` + `scripts/rtk_eval/{run_eval.py,align_traj.py,eval_rpe_attr.py,gen_report.py}`。59 个有效 scene、404 条 RTK，仅 78 条有 ≥20 GPS 星。

**结论**：Super-LIVO RTK ATE best 0.0090 / mean 0.0779 / median 0.0525；VIO best 0.0080。SM-101/102/103（130 s / 555–557 帧）0.0220 / 0.0222 / 0.0222；SM-201 0.0523、SM-206 0.0338、SM-303 0.0293 跑通。§19.3 原报"12 scene 0 帧"被 `align.log` 推翻（实际 200/149/222/228/233/233/211/232/177/158/214/157 帧），但轨迹 75–83 s 后失效（SM-201 83.0 s 后位姿全 0），仅占 18.9–22.9% 帧；`full_path_rmse` 6035–9545 m 系 GT 坐标未处理。另有绝对 ATE 结论：Super-LIO 0.347 m vs 官方 FAST-LIO-SAM 0.374 m，无累积漂移，误差主要是子图刚体偏置。

**产物**：`/home/ywj/slam_data/rtk_smbu_eval/`（`runs/`、`report.md/html`、`summary.json`、`align.log`、`eval.log`）、`docs/experiments/RTK_SMBU.md`。

---

## 8. 图像消融判决（§20，2026-11-12）

**流程**：`experiments/fl2_img_ablation/00_run_all.sh` 驱动 `01_run_ntu_ablation.sh`、`03_run_retail_ablation.sh`（同 bag 两轮 + `02_patch_odom_timebase.py` 改戳）、`04_check_odom_timebase.py`、`05_analyze.py`；RTK 轮 `/dev/shm` overlay + `--rate 8`，`vio` vs `only_lidar`。

**结论**：
- NTU 07/09 两轮方向相反，不能据此判图像有害。
- SMBU a/b/c 与 NTU 同向（有图好 36%/8%/43%），但缺同 bag 无图轮；a1、b5 各只有单轮且方向相反。
- 同 bag 仅改图像开关（RTK-SLAM，ATE m，有图 vs 无图）：`smbu_rtk_001/005/008/010/011` **全部有图更差**（1.4092 vs 1.5224；0.0816 vs 0.0549；0.0085 vs 0.0097；0.0657 vs 0.0446；0.0458 vs 0.0139）；`smbu_rtk_013` 唯一相反（0.0521 vs 0.0272）。RTK 轨迹只有 450→421→257 帧（有效约 32 s）。
- 判定：图像在室外开阔场景系统性有害（与 §13 合成剂量、§14 地板、§15 G 门控四线一致）；但 13 个 scene 未跑、013/008/001 未复现，约 6 成把握。

**产物**：`experiments/fl2_img_ablation_result.json`（15 行 ATE）、`fastlivo2_rtk_out/`、`fastlivo2_smbu_out/`、`fastlivo2_all_*.log`。

---

## 9. SMBU 官方基线补齐（§21，2026-11-12）

**前提**：本仓库 vs 官方唯一可靠对照此前只有 07/09，且本仓库侧是 10-29 旧运行。本仓库 09 与官方 09 均 `vio` + `FIFO_SIZE=4/PATCH_SIZE=2`、同 commit（Super-LIO `67c0acd4`）。

**结论**：07 官方 0.0088 优（本仓库 0.0274 为其 3.1 倍）；09 官方 0.0077 vs 本仓库 0.0077 完全相等；13 官方 0.4651 vs 本仓库 0.3170（本仓库优 32%）。官方 LIVO 0.0088/0.0077 远优于官方 LIO 0.5668/0.1899 →"图像有害"是**本仓库集成特有问题**，非官方实现问题。

**产物**：`fixed_world_map/fastlivo2_data09_fastlio2_livo.log`、`r3live_out_0[79]_vio_20261112_15*/`。

---

## 10. RTK 三方对照与 SM-101 复现（§22，2026-11-14~15）

**前提**：RTK-SLAM 5 个 scene，对照组 = 官方 FAST-LIVO2 `vio`，Super-LIO LIO 单列。三方全齐仅 SM-101/102/103（`…3125932855297` 无 Super-LIO 产物，只有 11-11 官方 0.0250）。

| scene | Super-LIVO | 官方 LIVO | 官方 LIO |
|---|---|---|---|
| SM-101 `…465` | 0.0206 | 0.0220 | 0.1441 |
| SM-103 `…6176` | 0.0234 | 0.0222 | 0.5442 |
| SM-102 `…1792` | 0.0215 | 0.0222 | 崩 0.2519 |
| SM-203 `…6864` | 0.1358 | 0.0690 | 崩 2.6074 |
| SM-206 `…9488` | 0.0414 | 0.0338 | 未跑 |

另：官方 a5 LIVO 0.0103 vs b5 LIO 0.0142（同轨迹同配置同 bag，仅 `img_en` 开关差）。

**结论**：RTK 场景图像方向不统一（07/101/102/103 加图变好，206 变差 18.4%，104 差 88%）→ §20 的"室外有图一律更差"**降级**；按帧染色链（§5）仍是唯一站得住的"图像有害"证据（机制未定案）；RTK 轨迹仅占 bag 25% 有效时长（`L176` 537.784 s/4262 帧，RTK 交集 450/421/257）；Super-LIVO 无崩溃。

**产物**：`smbu_rtk_001_superlio_repro/`、`smbu_rtk_010_superlio/`、`superlio_rtk_eval/{runs,smbu_rtk_eval,run.log}`、`fastlivo2_rtk_out/smbu_rtk_001_superlio_20261114_170323/`、`experiments/fl2_odo_diff.json`。

---

## 11. 视觉门控 / 合成剂量 / 误差地板（§13–§15，2026-11-10）

- G 门控消融：NTU07 0.0274→0.0351、NTU09 0.0164→0.0453（关 G 变差），G 接受率 75.8%。
- 合成旋转注入斜率：0.234 / 0.345 pp/cm。
- 单帧/地图域误差地板：官方静扫 2.46%、sl_map 3.39%、e1_map 4.13%。
- 位姿来源因子化（同 bag）：LIO-only sl 1.27 m / e1 1.29 m。
- NACC 与 deskew 审计见 `docs/DESKEW_PIPELINE_AUDIT.md`。流程细节未逐条展开。

## 12. 方案 A/B 开发实验

- **A：视觉约束注入 ESKF**（`experiments/A测试-视觉约束注入ESKF.md`）：6×6 视觉法方程直注 `HTVH/HTVr`、坐标变换 `A=blkdiag(R_wb,I₃)`、信息量 cap（κ_healthy=1.0 / κ_degen=5.0 / σ 地板 0.005 rad、0.02 m）。流程与实现有记录，**验收 ATE 未在该文件内给出**（NTU eee_03 整体评测 A 0.221 m / B 0.681 m，见 HANDOVER 与根 README）。
- **B2/B3 光流旋转约束**（`experiments/B2B3-光流旋转约束实验记录.md`）：NTU 上曾报 0.0132 / 0.0072 m，但无同轮次对照，**已判不可引用**。

## 13. KNNHeap 修复 + 体素/sub_div 复跑（2026-11-12，docs/KNNHEAP_FIX_AND_VOXEL_RERUN.md）

**前提**：`update_worst_unrolled` 只比较 5 槽 → 修为遍历 K；单测修复后 `ALL PASS`（旧逻辑 169 FAIL）。

**流程与结论**：
- `livox_record_noshell`（674 s/6744 帧，不闭环）：体素 0.25/0.15/0.10 三组**无法判精度**（旧"越细越差"结论整段撤回）。
- `livox_record_drone`（466.9 s/~297 m，单闭环）：closure 3.30→1.67→0.35 m、fit_res 0.0339→0.0170、knn_fail 0.75%→2.55%、stage 4.36→8.39 ms。
- `livox_record_drone_1f`（637.4 s/~775 m，双闭环，0.10 用 4M 容量复跑）：闭环1 3.06/3.12/3.55、闭环2 3.71/4.08/5.65 m（越粗越好）；体素需求峰值 2.83 M 超 2 M 上限。
- `sub_div=4` 对照：1f 上 closure 5.66→3.88（−31%）、4.08→3.29（−19%），fit_res −10%~−27%，knn_fail 17.3%→2.3%，stage +44~55%（9.84→14.21 ms），RSS 4.85→3.41 GB；drone 单闭环方向分裂。**建议默认 `sub_div=4`、`resolution` 0.20 或 0.30（已落地 Super-LIO `7542c0e`）**。

**产物**：`third_party/Super-LIO/exp/results_{knnfix,livox_record_drone,livox_record_drone_1f,fz_drone,fz_1f,fz_cap}/`。

## 14. 点云质量指标方法学（docs/POINTCLOUD_QUALITY_METRICS.md + experiments/gpt_点云指标.md）

**流程**：Retail_Street 招牌区域（6.7×1.1 m，A 52.3 万点 / B 45.3 万点）55 指标筛成 V1.0 四维：Q1 `G00_Nb_Radius@K10`、Q2 `G15_Roughness_CV@K25`、Q3 `O02_ROR_Ratio@3cm`、Q4 栅格椒盐（仅诊断）。脚本 `qc_metric_report.py` 等。

**结论**：组内 Spearman(G00,O02) −0.217/+0.275（合并 0.661 为假象）；椒盐整图 7.65% vs 14.43% 但仅 4/10 patch 同向；点数少 13% ⇒ 理论间距 +7.4%、实测 +19.5%（约 2.6 倍）。

## 15. RK3588 实机测速（experiments/speed_test.md，2026-11-09）

**前提/条件**：RK3588 / 3.246 GHz（周期 0.308045 ns）、内核 5.10.160、aarch64；绑大核 4–7（`SCHED_OTHER` 未提权）；`taskset -c 4`；ROS 2 Humble Release；输入 NTU `data09`，每配置 1800 雷达帧、3 次重复；2 Hz `/proc` RSS；p99 为排序后 99% 样本。计时点：`main.cpp` 在 `visual_realtime_system_process()` 前后取 `CLOCK_MONOTONIC` 差（不含读线程），`process_one_frame()` 前后取子区间。占空因数 = 0.001×帧数×平均耗时/运行时间。

**结论**：Super-LIVO 主线程 9.38 ms/帧（占空 9.4%），从 LIO 起 13.61 ms（13.6%）；LIO-only 7.87 ms；FAST-LIO2 12.53 ms（12.5%）；FAST-LIVO2 官方 24.46 ms（24.5%）、+30 Hz 28.32 ms（28.3%）；图像开销 6.88 vs 0.57 ms（约 12 倍），映射开销 21 倍。五进程峰值 CPU 40.65%；单核预算 96.25%、余量 3.60 倍。60 s 连跑：官方 VIO p99 51.9 / max 254 ms；LIO+color p99 41.7 ms、LIO 41.0 ms（p99 比 2.31×）；CIO p99 53.0 ms 无 >200 ms；全程 0 崩溃 0 跳变。1449 s SM-103 五进程 26.0%（官方 LIO 25.9 / LIVO 16.5 / CIO 13.9%）。此前"CPU 92.5%"确认为 8 s 采样错误，已修正。

**记录缺口**：README 与 `test_results.md` 未跟踪，原始 CSV/日志未入库（`scripts/robot_20260527_182555.csv`）。

---

## 16. 楼梯井漂移 A/B/C 三判据（M.0–M.13，2026-10-21~23）

**前提/动机**：五-L 已证明两条楼梯的扭转方向相反、模长相近、回到原点互相抵消，闭环 yaw/3D 只剩抵消残差，无法回答"误差在哪一阶段引入"。改为在同一空间位置按正/反旋转方向各取一段，逐阶段定位。

**流程**：
- A（离线点云）：`exp/dir_compare.py` 重放 `livox_record_drone` 原始点 + odom + 原始 IMU，按高度轴 `up = R(q0)·up_imu` 拟合主导竖直面，报厚度（±15 cm 窗内 1.4826·MAD，不截断）、倾角、半帧方位角差 Δθ；`--rate-scale 1.3/2.0` 做灵敏度正控。
- B（debug.csv）：`exp/obs_corr.py` 用 `corr = rotvec(exp(rv)·exp(rv_pred)ᵀ)` 投影世界竖直轴得 `cy`，与 odom 逐帧角速度做线性回归。
- C（冻结地图）：`lio.map.freeze` + `load_path`（重建 ds≈0.08 的 1,276,267 点先验）+ `init_pose`。

**结论**：
- A：厚度/倾角对 yaw 天生二阶不敏感（δψ=1°、R=7 m 仅 1 mm 法向偏差）；8 个转速档 |Δθ|≤0.13°，静态噪声地板 ±0.17° ⇒ 分辨力上限≈3% 角速度误差。正控 k=1.30 时 72°/s 档 Δθ=−0.48°，k=2.00 时 +0.96°/−0.82°，>30% 误差明确有响应。
- B：`cy` 与 ω 严格同号成正比；折成等效量 δ=3.01/3.00 ms，在 10–30/30–60/60–120 °/s 三档约 3.5/3.8/3.5 ms，与 |w| 无关 ⇒ 线性项，不是阈值/饱和/匹配不足（n_eff 中位 548~863）。
- M.10 判别两个互斥候选：点时钟 −3 ms **否**（±3 ms 只让斜率动 0.22~0.26 ms 且方向相反）；陀螺 scale 1.032 **是**（斜率 +2.563→+0.223 ms，−91%；R² 0.391→0.006）。但 scale 不改善闭环（1.594→2.013 m），1.028 折中最好（1.636 m）。分轴回归只有 z 轴显著（2.56 ms、R²=0.37）。
- C：冻结 1.28 M 点先验地图后方向相关偏差不但没消失还略增强（+2.66 vs +2.56 ms，R² 0.434 vs 0.391），闭环 3D 1.597 vs 1.594 m ⇒ 地图反馈不是该偏差的必要条件。注意：先验地图本身由同一条倾斜轨迹生成，"倾斜地图自洽"未被完全排除。
- M.11 固定俯仰偏置：下行段 119° 前倾是真实姿势（加速度计原始重力对比 118.91°），不能当误差；真正偏差是姿态残差一阶项——逐腿等效倾角 baseline 回程 −2.97°→最优 −0.45°。它**非速率型**（世界 x/y 轴 R²=0.041/0.088）；全包累计绕世界 y −21.96°、x +10.08°，而 yaw 只有 +7.78°，且与 `fit_res` 脏度同步 ⇒ 只有法向加权一类机制能压。dy 回归 r=+0.937、斜率 2.22、截距 −1.07、rmse 0.36：残差姿态只解释约 45% 幅值，另一半来自配准/地图。
- M.12：形状最正 gz 残差 0.095/0.129/0.178，比最好的 sl（0.143/0.299/0.337）还小一半以上；共模/差模分解——gz 共模主导（7.15 vs 1.49），sl 差模主导；Δroll 20/20 个 sl run 落在 −1.3~−3.8，是全族最稳定系统项。
- M.13：抬 ng（0.1→0.3→1.0）全面变差（闭环 0.845→1.321→1.225、整体 yaw +5.91→+6.19→+6.41）；抬 na（0.1→1.0）三主指标同时变好（0.845→0.725、yaw +5.91→+5.59、rmse 0.214→0.194），但压不动 Δroll。叠加到 nbg=3e-3：`sl_nbg003_na03` 闭环 0.460 m/300.7 m=0.153%、yaw +4.67°、rmse 0.206 m；5 次独立 run odom md5 相同，属"完全确定"一档。na 不是越大越好，0.3 附近是拐点。

**产物**：`exp/results_drone/{sl_base_fr6,sl_dbg2,sl_c_frz,sl_map008,sl_gsc10xx,sl_nbg003_na03,…}`、`dir_top_platform.csv` 等四窗口 CSV、`exp/{dir_compare.py,obs_corr.py,floor_yaw.py,slab_cmp.py,attitude_drift.py}`。

---

## 17. 跨 bag 验证与参数/机制解耦（M.14–M.23，2026-10-23~24）

**前提/动机**：楼梯 bag 单场景结论需跨平台/跨场地验证；`livox_record_noshell`（狗载、无外壳、非楼梯）没有闭环真值，`livox_record_drone_1f`（777 m 双闭环）提供第二个可用 benchmark。

**流程**：M.14 noshell 两组对照；M.15/M.16 用"静止场景配准"把锚点漂移量成绝对位姿真值；M.17 单变量解耦 na/nbg/nw；M.18 冻结重力自由度；M.19 扫 Huber 宽度与平面门限；M.20 择优定型；M.21 切分左/右圈；M.22 12 格固定窗闭环 + 组合回归；M.23 交接清单。

**结论**：
- M.14-3：noshell 无闭环真值，闭环指标**作废**，只用于"相对同 bag 基线的插值差"。
- M.17-2：1f 上只有 normal_weight 是真因子，na/nbg 都在噪声级；且法向加权在 1f **有害**（与楼梯相反）。
- M.18：世界系竖直是被估计出来的自由度，g 状态转动 1.11°；`freeze_gravity` 只关这条自由度，竖直偏差 −45%。
- M.19：`huber_delta_base` 0.05→0.20 使水平闭环 −31%、yaw −27%；`plane_fit_threshold` 变严有害（有效观测变少）。huber 与 freeze_gravity 两条机制正交，可叠加。
- M.16：静止场景配准四条独立证据链给同一 1.0~1.3° 世界系俯仰偏置；该判据比闭环误差硬。
- M.20：竖直 1.035°→0.386°；**水平误差在第 1 圈末已饱和**，第 2 圈不再累积。
- M.21：决定性判据——同一物理路径两种初始化证明误差来自左圈（第 1 圈）；陀螺零偏漂移已排除；左圈内部无自重叠、51 s 后与起点无共同几何（链式 ICP fitness 0.16~0.37），必须新增观测才能继续切。
- M.22：三处"组合即倒退"；新失败模式——**静止悬停时位姿崩塌**（丢帧/IMU 欠载，比闭环误差更该盯）。
- M.23：可直接引用结论按可信度排序；被推翻/需打折的旧结论单独列出（含 `sl_nw035_nbg001` 两次闭环 6.313/0.440 不稳定，只取形状参考）。

**产物**：`results_drone_1f/*`、`results_noshell/*`、`map_slab_cmp.py`、`exp/attitude_drift.py`。

---

## 18. 退化检测 / 护栏 / 污染审计（M.24–M.26，2026-10-24~25）

**前提/动机**：判断退化信号能否用于参数自适应/矩阵截断/坏帧护栏，并审计既有 run 是否被污染。

**流程**：M.24 三种响应 A/B + 坏帧护栏 + 端到端延迟；M.25 阶段 0 污染审计 / 判据修正 / 阶段 2 新机制（2.A/2.B/2.D/2.E）；M.26 imu_na 归因收口。

**结论**：
- 污染审计：114 个 run 中 16 个被污染（14.0%）。
- `ns_h20` 13 m 崩塌**作废**；M.22 的 h20 组合表**全部作废**；`deg_h20_dbg` 0.025 m 为准。
- 参数自适应不划算（即使去抖）；退化信号是帧级抖动的。
- 悬停崩塌根因是丢帧/IMU 欠载，不是估计器失去自洽（决定性）。
- 阶段 2 新机制：2.A 地面软约束、2.B 距离噪声、2.D 高速旋转 Huber、2.E 逐点 ΔT 对齐；每个都做成默认关的开关，楼梯+1f 双 bag A/B。
- M.26 可复现性锚点：同配置逐位可复现；`sl_nw035_nbg001` 两次闭环 6.313/0.440 不稳定，只取形状参考。

**产物**：`exp/pollution_audit.py`、`results_fz_*`、`/tmp/sl_install2`、M.25 路线图。

---

## 19. 细通道 / 尖峰 / 场景自动档 / 时延（M.27–M.32，2026-10-25~26）

**前提/动机**：细尺度特征（墙角/铁窗/扶手）是否被平均降噪抹平；MapInsert 尖峰；能否按场景自动切档；在线时延能否估计。

**结论**：
- M.27：细特征瓶颈在观测链路三环节；细通道是"第二观测通道"，不建第二份地图。
- M.28：P3.1 拒绝点×退化相关性 r=−0.12/−0.42 ⇒ 细通道"安全但无效"；P2.1 定位 UpdateMap 尖峰。
- M.29：1f `UpdateMap` max 53.67→2.94 ms（−95%）、p90 32.78→2.69 ms；楼梯 max 8.54→5.81 ms（−32%）；md5 一致。
- M.30：`deg_dir_v` 判据证伪（楼梯误判 1.0%→16.9%/42.1%）；改用窗口化退化占比——楼梯 0.00%、1f 94.9%。
- M.31：楼梯自动档 md5 与固定 A 线相同；1f 自动档 2.976 vs B 3.051 vs A 4.097。
- M.32：在线时延 0.938/1.687 ms；但 apply 后楼梯 0.024→1.858 m（77×）⇒ 只观测不自动 apply。

**产物**：`results_fz_*`、`timing_spike_audit.py`、`exp/` 下对应脚本。

---

## 20. 地图表示层 / K 运行时化 / 新代码重定标（M.34–M.36，2026-11-10）

**前提/动机**：M.34 复测地图表示层（真实点 vs 增量均值）与两个门限；M.35 把 K 提到运行时；M.36 在新代码上重定标 A 线。

**结论**：
- M.34：16 格双 bag 对照；rep_mode=1 与 merge 放宽同量级（楼梯 1.533 vs 1.561 m、1f 2.339 vs 2.275 m）⇒ 用户"平均点致漂移"假设**证伪**；有效变量是"地图点被允许多大程度跟随新观测"。楼梯任何改动 0.021→1.5~1.9 m；1f rep1 −23.3%；B4@1f 第二轮 1.00 s 断档后跳到 12.673 m ⇒ merge 放宽**不采纳**。
- M.35：一次"编辑没落盘"事故复盘。正确账目：64c217d K=5 0.023 → 67c0acd+生产合入 K=5 1.030（45×）→ +update_worst bug K=8 2.648（再 2.6×）→ 修好 0.562（4.7× 改善）。K=8 修好后优于 K=5；但 1f 相反（K=5 2.241 vs K=8 2.927）。rep_mode=1 不采纳。代码 `g_plane_knn_k`（4..8 默认 8）、`eff_k()=min(g_plane_knn_k,K)`、`update_worst` 改扫全部 K 槽。
- M.36：A 线轴序不变（na 0.3 / huber 0.20 / nw σ0.035），但谷底 0.021→0.562（27 倍）；当前最优=现 A 线+K=8；`trust_guard` 默认 true 有益（K=8 关掉 0.562→0.786，K=5 1.031→1.704）。45× 根因**未定位**，已排除 7 项；ulp2 实验证明 N==4 用 8×3 子块与 `Matrix<4,3>` 不逐位等价（1.031→2.446）。

**产物**：`exp/results_*`、`OctVoxMap.hpp`、`super_lio.cpp`、`stairwell_drift_diagnosis.txt` M.34–M.36。

---

## 21. RTK construction_seq2 真值与 cond 因果证伪（M.37–M.38，2026-11-10）

**前提/动机**：接入外部 RTK-SLAM construction_seq2（16 个 Leica TS16 检查点，<1 cm，UTM 32N/ETRS89），为"绝对精度回归"提供基线；同时检验"高条件数 ⇒ 漂移"的既有因果解释。

**结论**：
- M.37：官方 FAST-LIO-SAM online 0.439 / offline 0.373 / SE(3) 0.089（Gap 76%）；方案 B 离线转换 `convert_livox_driver.py`；首轮 B 线 5,835 帧/595 s/419,267 点，16 点全匹配 0 丢弃；弦长比 **100.00%**（278.2 vs 278.2 m），15 段 ±0.5%；yaw+平移对齐 RMSE 0.25 m（常值偏置，杆臂 base_center (-0.073,−0.023,−0.172) 待换算）。
- M.38：三方对照——seq2 cond p50 404.6 / >150 占 94.7% / ax≥3 占 62.0%，但精度最好；1f p50 220.2 / 78.2% / 41.5%；楼梯 p50 39.9 / 1.1% / 0.0% 最敏感 ⇒ **"高 cond ⇒ 漂移"因果链证伪**。改解释为"错误约束是否自相矛盾"。M.37 §7"几何充分不构成退化"解释**作废**。推论 P-1 ghost 注入、P-2 重复周期数待验证。

**产物**：`experiments/convert_livox_driver.py`、`/home/ywj/slam_data/rtk_slam/`、`docs/` 下对应记录。

---

## 22. 生产化收口：K=8 / sub_div=4 / trust 25° / E-1~E-5（M.39–M.40，2026-11-11~14）

**前提/动机**：把 M.39/M.40 的新代码最终收口到生产参数，并做 trust 正向核验、ghost 抗伪、性能优化。

**结论**：
- M.39：sub4 依赖 K≥8；K 阈值 15°→25°（正确修正 15.9°/19.1° 曾被误否决，drone 1.169→4.689 m；放宽后 0.831~0.904 m）；Observe 代价 sub2 +1.5~1.7 ms、sub4 10~18 ms；三 bag 复验 drone 0.904 m(0.301%)/yaw −1.037°、1f 终点 2.87 m、noshell 2.05 m；结论 C-1~C-5；生产提交 4cf20ed、755fb3f，exp b202cc2。
- M.40：E-2 trust 25/15 等价（corr_r p50 0.15°/max 0.67°，分档全 0 帧，md5 相同）；E-3 ghost 注入 2.6 m 伪点仅终点动 2 mm、dyaw 20.70° 一致；E-4 槽扫描优化 Observe 9.55→6.89 ms（−28%）、轨迹逐点一致；E-5 1f na×nw 2×2（A 3.071 / B 2.220 / C 2.867 / D 3.129）；结论 C-6~C-10。
- 定稿参数：na 0.01 / ng 0.1 / nbg 0.003 / huber 0.20 / trust 25° / sub_div 4 / K 8 / nw off。

**产物**：`stairwell_drift_diagnosis.txt` M.39/M.40、`test/test_knn_heap.cpp`。

---

## 23. HANDOVER §19.11–§22 补充：RTK 参数消融 / 官方图像判决 / 视觉内存模型

**前提/动机**：HANDOVER §19.11–§22 的三条结论线此前未进总账——RTK 参数消融、官方图像判决、视觉侧内存模型。

### 23.1 通用前提（本节共用）

- 数据：RTK-SLAM `construction_seq2`（r2 bag，599.5 s，`/livox/lidar` 为 CustomMsg），16 个 Leica TS16 检查点、<1 cm、UTM 32N/ETRS89。
- 机器与运行：WSL2，23.5 GB 内存；`RATE=0.3` 播放、`img_en=1`、`pcd_save.interval=200`、`T_IMU_BASE=[-0.073,-0.023,-0.172]`、`HEAD_SEC=30.0`、odom 话题 `/aft_mapped_to_init`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、DOMAIN 0。
- 跑批互斥：`/tmp/ros_batch.lock` + `flock -n 9`，与外部 NTU degen 循环（`run_ntu_degen.sh`）共用一把锁；只能在锁空闲时启动，不杀别人的进程。
- §19.11 / §20 的既有结论保持原文，见 23.2 / 23.3；§21 的"结构性不可能"已被 §22 取代，见 23.4 → 23.9。

### 23.2 §19.11 beam_err 消融（整轮 599 s，同 bag 只改 BEAM）

ATE 0.347（beam=0）→ 0.508（0.003）→ 0.656（0.01），损伤全在室内/过渡（室内 0.357→0.567→0.749），室外三点纹丝不动；三个 run 逐帧 hf 全为 0.4 cm。机制：σ²(r)=dept²+(beam·r)² 把远处点降权，室内只剩远墙弱结构，降权=撤约束。与 NTU"0.138→0.132"方向相反 ⇒ 观测噪声参数不可跨数据集搬。迭代上限 `kf_max_iterations` 是空转参数（4→8 提前收敛，0%）。三个坑：参数名须 `lio.kf.kf_max_iterations`、DDS 快速起停会 `frame_num=0`、`-p` 后须看日志确认 `[Param]`。

### 23.3 §20 NTU eee_03 图像判决（官方 `img_en=1/0` ×3 轮交错）

ATE 0.0291±0.0008 vs 0.0425±0.0039，逐轮 +16.8/+15.1/+8.5 mm（均值 +13.4 mm，3/3 同号）；但**全部增益来自起步 0–10 s**（+106 mm），10 s 后每个 10 s 箱差都在 ±5 mm 内。指纹比对：`Log/result/eee_03_lio.txt` 的 0.028 实为 LIVO（`_lio` 误名）；`eee_03.txt` 的 0.084 作废。Retail 无图像 3 轮有 1 轮跑飞（路径 190.76 m、139 次 >300 mm 跳变、回环 48.5 m/19.28°）⇒ 视觉的有效作用是"锚定/防跑飞"，稳态精度增益≈0。

### 23.4 §21 旧结论（已作废，保留以说明推翻过程）

内存增长 48 MB/数据秒（23.14 GB ÷ 480 s），曾据此外推整轮 599.5 s 约需 29 GB > 本机 23.5 GB，判定"结构性不可能，必须换 ≥32 GB 机器或 `img_en=0`"。`pcd_save.interval=200` 实测产生 46 个分片、确实生效，但同一轮仍被 OOM 杀（`anon-rss:23141900kB`），覆盖 480.4/599.5 s = 80.1%。首轮离线 ATE 0.631 m 只在 12 个 GT 点上算（后 4 点无位姿）。同窗 12 点：fast-lio-sam 0.418 / fast-livo2 0.631 / super-lio 0.384；SE3 0.092 全场最低 ⇒ "形状准、绝对位置不准"。

> **本条的"结构性不可能"与 0.631 m 均已被 23.9 推翻**：真实瓶颈只在视觉侧，滚动淘汰后本机可跑满 99.3%，整轮 16 点 ATE 为 0.561 m。

### 23.5 §22 诊断流程：把增长拆到具体对象

1. **先证伪"分片已解决"**：查旧轮产物得 46 个分片，同时 `dmesg` 有 `anon-rss:23141900kB` / 19.5 GB 两条 OOM 记录 ⇒ 分片保住了已 flush 的点云，但没有限制内存。
2. **按开关做差**：同 bag、同轨迹，仅切 `img_en`，实测 `img_en=1` **51.8 MB/数据秒** vs `img_en=0` **4.3 MB/数据秒**，12 倍差全部落在视觉侧。
3. **代码定位**：`generateVisualMapPoints` 里 `ftr_new->img_ = img;` ⇒ 每个视觉地图点持有一个 `Feature`，`Feature` 挂着**整幅原始图像**的 `cv::Mat`，点不释放图像就释放不了；`insertPointIntoVoxelMap` 只 push 从不删；`feat_map` 只在 `~VIOManager` 清；`warp_map` 按 Feature id 索引，`normal_en=true` 时原代码从不清空。
4. **速率核算**：1.92 MB/帧（1600×1200×3 经 `image_resize_factor` 缩小）× 16.5 Hz ≈ **32 MB/s**（被钉住的图像）+ 视觉点自身增长 ≈ **16 MB/s**，合计 ≈48 MB/s，与第 1 步的实测斜率吻合，闭环成立。

### 23.6 §22 实现流程：滚动淘汰

改 4 个文件（fork `third_party/FAST-LIVO2-ROS2`）：

| 文件 | 改动 |
|---|---|
| `include/visual_point.h` | 新增 `double stamp_ = 0.0;`（建点时刻的图像时间） |
| `include/vio.h` | 新增 `double cur_img_time_`；声明 `size_t pruneVisualMap(double cutoff_time);` |
| `src/vio.cpp` | `processFrame` 入口记 `cur_img_time_`；`generateVisualMapPoints` 里 `pt_new->stamp_ = cur_img_time_`；新增 `pruneVisualMap` |
| `src/LIVMapper.cpp` | `publish_frame_world` 中分片落盘后调用 `pruneVisualMap(update_time)` |

`pruneVisualMap` 遍历 `feat_map`，删除 `stamp_ < cutoff_time` 的点（`delete` 连带销毁 Feature，`Mat` 引用计数归零、图像随之释放），同步收缩 `voxel_points`、回收空体素、**清空 `warp_map`**。cutoff 取当前帧 `update_time`，故当前帧刚建的点保留，留到下一轮匹配——即"写过的丢掉、等待匹配的暂留"。

验证顺序：`colcon build --packages-select fast_livo` → 40 s 冒烟（19 分片 ↔ 19 次 prune，每次释放 1.1–2.8 万点）→ 排队等锁 → 600 s 整轮。

### 23.7 §22 整轮结论

| 项 | 值 |
|---|---|
| 覆盖 | `lidar_poses.txt` 11990 行，**595.5 / 599.5 s = 99.3%**（旧 80.1%） |
| 分片 / prune | 58 个分片 ↔ **58 次 prune** 一一对应 |
| 合并点云 | 24,877,834 → **8,924,894** 点 @5 cm（142 MB） |
| ENU 轨迹 | 11670 帧 / 598.7 s，GNSS 匹配 11668/11670，yaw 34.086°，前 30 s 残差中位 0.03 m |
| 官方 eval | **FAST-LIVO2 Offline 0.561 m**，SE3 0.091，Gap 84%，**16 个 GT 点全覆盖** |

同一次 eval、同一套 GT 的五法对照（`construction_seq2`）：

| method | Offline [m] | SE3 [m] | Gap [%] |
|---|---|---|---|
| super-lio | **0.347** | 0.179 | 49 |
| fast-lio-sam | 0.374 | 0.089 | 76 |
| super-lio-beam3 | 0.508 | 0.203 | 60 |
| **fast-livo2** | **0.561** | **0.091** | **84** |
| super-lio-beam | 0.656 | 0.212 | 68 |

结论：fl2 绝对精度仍差于 super-lio，但 SE3 0.091 依然最低；"形状准、绝对位置不准"的判断不变。**代价**：视觉地图只保留最近一片（约 200 帧 ≈ 12 s），长时视觉重观测能力被放弃；本评测口径下不劣化，若要做长序列回环需改成按空间距离淘汰。

### 23.8 §22 三层点云偏差（`tools/rtk_pcdev.py`）

**前提**：参考真值 = fl2 的稠密建图（本数据集中唯一有稠密建图输出且图像真正参与建图的方法），在其上按体素拟合局部平面得参考面网；各被测系统用**自己的 ENU 位姿**把雷达点搬上去量偏差。三层互不重叠：L1 距离分层平面散布、L2 跨帧重影（含图像重投影）、L3 绝对刚体偏差（unaligned = 初始化级 ENU 配准；aligned = 6DoF point-to-plane 刚体拟合）。

**流程**：`--stage ref`（参考面）→ `--stage p1`（扫源 bag，逐系统记录落面局部坐标）→ `--stage p2`（跨帧配对）→ `--stage report`（JSON + PNG）。实际口径 `--edge 0.25 --min-planes-ref 2000 --min-per-plane 8 --min-per-frame 3 --trim 3.0 --max-align-t-cm 100 --max-align-deg 5`。

**改造与两个 bug（本次修掉）**：

1. 参考面判据改为自适应逐级放宽。5 cm 体素化的参考云在 0.25 m 体素内只有十几到几十点，固定 `min_pts=60` 实测只剩 ~70 个面、命中几乎全在 0–5 m；放宽后 **39457 面 / 覆盖参考点 26.7%**。
2. `idx[keep]` 维度不匹配：MAD 截断第一轮已收窄 `idx`，残差却仍按全量算，第二轮直接 `IndexError`。
3. 该崩溃掩盖了更严重的**雅可比错误**：旋转块写成 `cross(q, n)` 而 `q = v − cen`，等于多出 `cross(cen, n)`；`cen` 在 ENU 下可达数百米，迭代收敛到 **δt≈48 m / δθ≈100°** 的发散解，五个系统全被"面数不足/纠偏超限"门控拦下。改用已变换点 `v` 后 δt 落到 3–8 cm。**教训：ENU 大坐标下任何"减去面心"形式的旋转雅可比都会引入 cross(cen,n) 量级的系统误差。**

**结论（对齐）**：

| system | 可用面 | δt [cm] | δθ [°] | L3 未对齐法向 MAD [cm] | 对齐后 [cm] | vs_floor |
|---|---|---|---|---|---|---|
| fast_lio_sam | 1129 | 4.9 | 0.09 | 8.72 | 8.61 | 0.91 |
| super_lio | 936 | 5.1 | 0.07 | 9.80 | — | 1.03 |
| super_lio_beam | 1002 | 8.3 | 0.13 | 7.70 | 7.60 | 0.81 |
| super_lio_beam3 | 1011 | 7.6 | 0.07 | 10.61 | 10.35 | 1.11 |
| fast_livo2（自比对） | 1722 | 3.1 | 0.17 | 9.53 | 9.55 | — |

**结论（L1 距离分层，σ²(R)=σ0²+(σθ·R)²）**：

| system | σ0 [cm] | σθ [°] | σ@10 m [cm] | σ@30 m [cm] | R² | bins |
|---|---|---|---|---|---|---|
| super_lio_beam | 0.66 | 0.103 | 1.91 | 5.41 | 0.09 | 3 |
| super_lio_beam3 | 0.22 | 0.282 | 4.93 | 14.78 | 0.76 | 3 |
| fast_livo2 | 0.96 | 0.134 | 2.53 | 7.10 | 0.65 | 5 |
| fast_lio_sam / super_lio | — | — | — | — | — | 每 bin n<300，不出拟合 |

**结论（L2 跨帧重影，point-to-plane MAD）**：

| system | gap 0.2 s | 0.5 s | 1.0 s | 5.0 s |
|---|---|---|---|---|
| fast_lio_sam | n=310, 1.81 cm | n=162, 5.69 cm | n=215, 2.39 cm | n=92, 0.77 cm |
| fast_livo2 | n=170, 1.03 cm | n=175, **12.93 cm**（depth p50 67 cm） | — | — |
| super_lio | — | — | n=63, 9.29 cm | — |
| beam / beam3 | — | — | — | — |

**三条必须一并引用的限制**：

1. `vs_floor` 全在 **0.81–1.11**，即各系统法向 MAD（7.7–10.6 cm）都贴着 fl2 自比对的 **9.53 cm**。这不是"五法一样准"，而是 **L3 这把尺子只到 ~10 cm**（参考面厚度上限 8 cm + 5 cm 体素量化 + 采样密度共同决定刻度）。**只能排除"偏差 >2 倍刻度"的系统，不能据此排序。**
2. fl2 自比对 MAD 最高不解释为"最差"——它就是参考面自身的拟合残差，天然是本组最大的一个；"噪声地板"不能当"下界"读。
3. 各系统可用面数 936–1722 不等，覆盖的参考面子集不同；对齐前后统计已统一到同一批可用面，但跨系统比较仍受此影响。

**未定案**：L2 样本稀疏且非单调（fl2 从 0.2 s 的 1.03 cm 跳到 0.5 s 的 12.93 cm，fast_lio_sam 又在 5.0 s 回落 0.77 cm），不能用"相对位姿漂移速率"解释，判为**不结论、只留原始表**。要让 L3 可排序，下一步需换更薄的面（`max-thick` 2–3 cm）或改成"点到最近参考点的法向距离"。

### 23.9 产物与提交

- 运行产物：`/home/ywj/slam_data/rtk_slam/run_fl2_seq2_full/`（58 分片、`odom/`、`Cloud_merged.pcd` 142 MB）；ENU 轨迹 `eval_ref/trajectories/fast_livo2/construction_seq2/`；三层缓存与报告 `/home/ywj/slam_data/rtk_slam/pcdev/{ref_planes.npz,p1_*.npz,report.json,layers.png}`。
- 代码：`tools/rtk_pcdev.py` + `tools/rtk_traj_prep.py`（`04692d4`）、HANDOVER §22（`f67f084`）、README + 子模块指针（`cb7f5fe`）；fork `third_party/FAST-LIVO2-ROS2` 滚动淘汰（`b9f931f`）。
- 复现坑：`rtk_traj_prep.py --odom-bag` 要给 **odom 目录本身**（`.../run_fl2_seq2_full/odom`），给父目录会在 `sorted(Path(bag_dir).glob('*.db3'))[0]` 上 `IndexError`。
- 收尾仍有 `[ros2run]: Segmentation fault`，发生在所有分片与 `lidar_poses.txt` 落盘之后；纯 LIO 轮也有，与本改动无关。

---

## 24. 2026-11-15 分支治理与可比性

**前提/动机**：commit 326ed21 把 Ouster bugfix 与实验代码混提；用户订立规则：`no-de-check` 是生产分支只收 bug 修复，实验进 `exp/degeneracy-adaptive`。

**流程**：A 方案追加回滚（不改写已推送历史）；exp 先合并 no-de-check；本地切 exp 分支重编译。

**结论**：
- `no-de-check` 净 +8/−1，仅 Ouster 时间单位 bugfix（276831b 收窄为 `if (g_lidar_type == LID_TYPE::OUSTER)`）。
- exp 相对 no-de-check：`super_lio.cpp` +808/−57；默认生效且改变数值的**只有 K=8 三处**——`OctVoxMap/KNNHeap.hpp` 的 `KNNHeap<8, Point>`、`getTopK` 的提前终止改用 `KNNHeapType::kSize`、`super_lio.h` 的 `KNNHeap<8, BASIC::V3>`。复现历史基线须把这两处改回 5（`calc_plane_coeff` 的 8×3 缓冲可保留）。
- 不改变数值的项：`g_adaptive_normal_weight_en` 仅在 `g_adaptive_param_en` 分支内使用；`grids_` 只 find 不遍历 ⇒ M.29 无行为影响；`OctVox::AddPoint` 参数化默认值与生产硬编码相同。
- 其余实验开关默认关闭/零值；新增话题 `lio/degeneracy`（`Float64MultiArray`，与 `lio/odom` 同频）8 字段 `[cond6, axes, active, cond_t, evt_min, ux, uy, uz]`；NTU `hash_capacity` 1e8→2e7；地图代表点 `g_map_rep_mode` 0/1/2。
- 后果：exp 分支上重跑 SLAM，与 no-de-check 及历史基线**默认不可比**（K=8 生效）。

**产物**：`third_party/Super-LIO/README.md` 实验分支说明段、分支提交号（no-de-check / exp b202cc2）。

---

## 25. 相机-雷达外参标定链：FAST-Calib → remote7.9 → holefix（2026-11-13~14）

**前提/动机**：标定板约 1.4×1.05×**0.015 m**，四 ArUco（marker 0.2 m，中心 (±0.55,±0.35)）+ 四通孔（半径名义 0.12 m，孔心 (±0.25,±0.20)），板装铝框上，雷达点会打在孔内壁与框上；板距雷达约 2 m。相机海康 MV-CB013（1280×1024 全局快门），内参 fx=708.6313 / fy=708.7151 / cx=644.4937 / cy=495.8836，D=[0.0544664, −0.0451474, 8.5461e-06, 0.00104866]（归档 `config/camera_intrinsic.yaml`；图像入节点前已去畸变，投影用纯针孔）。数据：`/home/ywj/slam_data/livo_bag/all_ros2`（ROS1→rosbag2 转换标定包）与 `move_ros2`（远场验证包）。

**流程**：
1. FAST-Calib 原解（`qr_params.yaml` 默认 `board_normal` 先验）：all_ros2 RMSE 0.0150 m；四孔重投影 5.83/7.02 px（mean/max）。
2. 远端变体（all_ros2_pc2 + img_0120.png）：RMSE 7.9 mm（下称 **remote7.9**，曾写入 A/B `*_move.yaml`），重投影 1.37/2.27 px。
3. 根因诊断：`board_normal=(−0.973,−0.004,−0.229)` 与 RANSAC 实测板面法向 `(0.923,0.003,0.385)` **差 9.46°**——四孔共面，法向无法由孔心约束，法向先验一错孔心求解即退化；四个孔心离真实板面分别偏差 90.4 / 91.4 / 23.6 / 24.5 mm（远超板厚 15 mm）。
4. holefix 重解：板面法向改用 338,796 点 RANSAC 拟合真实板面（σ=4.08 mm，d=−2.04357）；孔心取**前表面**（沿法向 +8 mm，板厚一半）；每孔只在 **±1° 扇区的最小半径轮廓**上取点（自动避开孔内壁/框杂点）；8 轮 80% 截尾；180° 歧义用重力消解（雷达 +z=上 → 相机 −y）。自检：半径 0.1244/0.1265/0.1289/0.1280 m，长边 493.6/490.9（名义 500）、短边 398.1/394.9（名义 400）mm，面内残差 1.9/2.0/4.2/4.2 mm。
5. 远场目视核验（2 m 板无法分辨：holefix 与 remote7.9 差 0.89°/33 mm，在 2 m 处仅 0.2 px）：A/B 用双外参分别跑 move bag 出彩色 pcd（`move_{a,b}` vs `move_{a,b}_holefix`）；`move_contact.py` 抽 6 帧接触表选远场帧（#592 长廊、#296 办公区）；`cmp_far_idx.py`/`cmp_far_two.py` 单帧双外参投影叠加（绿=remote7.9、红=holefix）+ `crop_far.py` 4× 放大；`cmp_map_hf{,_b}.py` 天花俯视（z>2.0）颜色图 + luma>0.85 白灯条掩码对比。

**结论**：
- 四孔重投影 holefix **1.17/1.69 px**，优于 remote7.9 的 1.37/2.27 px 与 FAST-Calib 原解的 5.83/7.02 px；采纳 holefix（物理推导自洽且残差略优）。
- 远场无可见劣化：6–9 m 处两点集图像位移 2–6 px（2 m 处仅 2–3 px）；白灯条占比 A 0.122→0.118、B 0.123→0.119；A/B 两版总点数 3,345,135 vs 3,261,637 与 13,139,823 vs 13,139,985。
- 精度边界：两版差值与标定噪声同量级，更严格的远场判别需把标定板放到 **≥8 m**。
- 新外参（已写入 A/B `*_move.yaml`，commit `22a9bc9`）：Rcl=[−0.018262,−0.999803,−0.007833; 0.469345,−0.001655,−0.883013; 0.882826,−0.019802,0.469283]，Pcl=[0.028643,−0.142351,−0.026469]。
- 事故教训：参考 pcd 被误覆盖两次——改 yaml 外参时忘改 `pcd.save_dir`，且 `move_run.sh` 启动即 `rm -f` 旧 pcd；靠回退 remote7.9 重跑恢复。**对照实验前先核对 save_dir 与脚本清理行为。**

**产物**：`/home/ywj/slam_data/output/move_{a,b}`（remote7.9 参考）、`move_{a,b}_holefix`、`move_contact.png`、`move_{ahf,bhf}_ceiling.png`、`far2_*.png`（脚本与图在 `/home/ywj/tmp`，临时）；README「标定板厚度/安装框的影响」小节；commit `22a9bc9`（已推送）。

---

## 26. RK3588 绑核四档对照测速（§15 同报告补录，experiments/speed_test.md）

**前提/条件**：绑核必须与实机 launch 一致，否则数字不可比——bag play `taskset -c 4`；`super_lio_node` c7 + `lio.single_core:=true`；`fastlivo_mapping` c7 + `OMP_NUM_THREADS=1`；`visual_a/b` c7（**与 LIO 同挤一个大核**）；`parameter_blackboard` c0。组间留 ≥15 s 降温，每组 ≥3 次重复。指标三件套同口径：odom 发出延迟（雷达消息到达回调→publish 完成墙钟差）、整帧处理时间（`[FrameProcess]` / `[LIOTotal]` / `[VIOTotal]` / 视觉 `合计`）、波动（std + P95/max；A/B 视觉为 ~1 s 一条采样，非全帧）。

**结论**（NTU data09，样本 1326/1300/124~125）：
- 整帧：Super-LIO 单跑 7.6±1.4 ms（max 16.5）、@A 9.4、@B 8.8；FAST-LIVO2 `[LIOTotal]` 71.1±11.0（P95 90.3）+ `[VIOTotal]` 27.9±8.0 ≈ **99 ms/帧**，贴着 100 ms 帧周期、实际略慢于实时；A 视觉合计 21.9±4.7（P95 29.1）；B 合计 41.8±27.2（P95 112.2、max 174.1），约束 22.7±26.6 为主要波动源。
- 实时判定：A odom 延迟仅 +1.5 ms（16.1 ms），可实时；B odom 延迟 16.4 ms 与 A 持平但视觉 P95 已超一帧周期，处于实时边缘；FL2 处理需求 ≈0.99 帧周期，长序列必然积压（135.5 s 数据仅发出 985/~1354 帧 odom，积压增至 ~20 s；**odom 延迟均值 5.3 s 是队列积压的体现，不是算法内部慢**）。
- **B 的 41.8/22.7 ms 是选轨 O(n²) 修复前口径**（当时单帧最坏 422 ms 集中在 `constraintTracks`）；修复后（观测史截断 `kMaxObservations=16` + 视差缓存）选轨 max 2.1 ms，B 是否仍"实时临界"必须重测，重测前不得引用这组数。
- 方法学：不要只看延迟——「延迟分布 + 同 bag 逐点差 + 退化段终点误差多跑分布」三判据一起看；A/B 的价值在退化段，健康段应与 LIO 逐点贴合（|B−LIO| p50 ≈ 2~5 cm）。任何单一指标都能给出误导性结论。

**产物**：`experiments/speed_test.md`（完整表与计时点清单）；跑批脚本 `/tmp/bench/run{1_slio,2_fl2,ab}.sh` + `parse_bench.py`（临时）。

---

## 27. 方案 B 退化对抗 8 跑实验 + A 状态回灌验证（2026-11，tag=y12fix2~y12fix9）

**前提/纪律**：NTU eee_03 走廊裁剪（`Y_HALF=1.2` 温和退化）构造 LiDAR 退化段；Super-LIO 在裁剪输入下**不可复现**（同输入终值随机），因此只允许两种判据：同 bag 内逐点 |VIS−LIO|、或多跑分布，**禁止引用单次数字**；不自造退化检测，只消费 Super-LIO 的 `/lio/degeneracy` 真度量。

**流程**：B（绝对锚定 + 历史锚点保持 + sev 映射先验 + 先验否决/速度限幅守卫）8 次独立单跑（tag=y12fix2~y12fix9）；A 侧验证状态回灌链路（`update.reinject`）同 bag 对照；过程中修复选轨视差扫描 O(n²)（观测史截断 + 视差缓存）。

**结论**：
- B **8/8 次不劣于同 bag LIO**；终点误差跨跑散布从 LIO 的 14.1~794.4 m（55×）压到 B 的 8.4~127.8 m（15×）——视觉介入不仅降低误差，还压缩了不可复现性本身。
- 幅度只按分布口径：均值 71.4 vs 267.7 m（−73%）、中位数 84.9 vs 170.3 m（−50%）、逐次比值 0%~87%（中位 41.5%）；旧"437→123 m / 72%"单跑结论**作废禁引**。
- 选轨修复后 8 次跑 `>50 ms 帧数 = 0/176~178`。
- A：回灌链路已通但对位姿**无纠正力**（同 bag |A−LIO|≤0.7 m）；瓶颈在约束质量而非机制——走廊段大量帧判"健康不更新"/rmse 拒绝，可用重投影约束仅 n=0~33。

**产物**：两份方案 README 的退化章节；退化段对照跑批产物在 `/home/ywj/slam_data`（y12fix* 系列）。

---

## 28. 自采 move bag"幽灵层"归因与护栏证伪（2026-11-13）

**前提/动机**：自采 2026-11 move bag 的彩色点云上方存在漂浮"幽灵层"，需判定算法侧还是数据侧。

**流程**：① 数据完整性排查：bag 内相机话题**断流 2.62 s**，撞上 `sync_packages` 无降级等待——主线程停摆期间视觉更新使用过期关联，生成整片漂浮层。② 算法侧尝试修复：加 VIO 更新位移护栏 `vio.pos_jump_thresh`。

**结论**：护栏实验**证伪**——整 bag 0 次触发、幽灵层不变，临时代码已全部移除。判定：**数据侧问题，只能重录；不要从算法层修这个 bag**。

**产物**：README「关键实验结论」自采 2026-11 move bag 条目；诊断期间的 overlay 图（`output/move_overlay_*.png`、`move_rg_*.png`）留存。

---

## 29. FAST-Calib-ROS2 子模块治理与 fork（2026-11-14）

**前提/动机**：主仓库记录子模块 commit `1973f15`，但 `.git/modules/third_party/FAST-Calib-ROS2` 已损坏（`refs/heads` 空、对象缺失），递归克隆报 `fatal: reference is not a tree`；且 4 个源文件的本地改动从未进 git 历史，无法复现。

**流程**：clone 上游做基线逐文件 diff（difflib）→ 备份本地改动 → 以 fork main `9fc278d` 为基线重建子模块仓库 → 恢复被本地删除的 `bag_images/colored_cloud.pcd`、设 `core.fileMode false`（消除 WSL 权限位噪声）→ 提交推送 fork → `.gitmodules` URL 切 `ypat999/FAST-Calib-ROS2` + `absorbgitdirs` 收回 gitdir → 主仓库 README 子模块表同步并推送。

**结论**：本地改动固化为 fork `bb1e748`，共 4 文件——① `data_preprocess.hpp`：手写 CDR 解析 livox `CustomMsg`（上游 stub 原为 no-op）+ `std::filesystem::exists` 检查 bag；② `common_lib.h`：新增 `board_normal_x/y/z` + `board_d` 参数（§25 的板面法向先验入口）；③ `lidar_detect.hpp`：给定法向先验时以薄带滤波 `|n·p+d|<0.02` 替代 RANSAC，全边缘点迭代拟合圆+剥离 inlier 替代 Euclidean 聚类，`FC_DEBUG_DUMP` 导出中间点云；④ `qr_detect.hpp`：cv_bridge/image_geometry 头文件适配 Humble。递归克隆现已可复现。主仓库 commit `bb86def`（顺带带入 346 个 `*:Zone.Identifier` 删除，Windows ADS 噪声，已确认无害）。

**产物**：`https://github.com/ypat999/FAST-Calib-ROS2@bb1e748`；`/home/ywj/tmp/fc_diff.txt`（临时）。

---

## 30. 染色链路对齐官方实现：1cm 中位回填是纹理损失的根因（2026-11-15）

**前提/动机**：同一 Super-LIO 位姿下，"官方染色点云"的颜色精度显著高于"本仓库方案 A 染色"，判断瓶颈在染色而非 LIO 位姿。定调为**停止压指标，逐项比对官方 FAST-LIVO2 `LIVMapper::publish_frame_world`，按官方逻辑重写融合层**。位姿口径不变：点云世界变换用 LIO 位姿，相机位姿用 VIO 光度修正位姿（`/vio/odom_lidar`），`t_img = 图像 header + img_time_offset(0.1)`，配对窗 `color.sync_max_offset=0.05`。

**流程**：
1. 逐项比对官方 vs 方案 A：`p_w` 用整帧单一位姿 / 相机位姿取 VIO 当前帧 / 图像侧同套 K,D 去畸变 + scale 0.5 / 深度与视野 `pf[2]<0` + `isInFrame(...,3)`（我们为 margin/20 + `min_quality`）/ 双线性取样（`patch_radius=0`）——均等价或近似。实质差异只有两处：**我们多做 z-buffer 遮挡剔除与 3×3 深度跳变剔除**；**官方逐帧逐点原样累积、无任何颜色融合，我们做了 1cm 体素时间中位回填**。
2. 新增纹理指标：按 1cm / 5cm 体素统计**体素内 RGB 标准差**（`var = E[c²]−E[c]²`，三通道均值，按点数加权）与**平色体素占比（std<2）**，脚本 `_probe_color_texture.py`。平均色 MAE 对"同体素内被写成同色"完全不敏感，必须靠该指标才能看见。
3. 四组在同一回放流上对照（`_replay_node_on_record.sh`，本轮新增 `check_depth_edges / depth_tolerance / min_quality` 三个透传，用于整档关掉我们的额外剔除）。

**结论**：

| 点云 | 着色帧 | 5cm 体素平均色 MAE | 1cm 体素内色 std | 1cm 平色体素 | 5cm 体素内色 std |
|---|---|---|---|---|---|
| 中位回填 K=6（`replay_node_onrecord_nocap_20261115`） | — | 5.91 | **0.00** | **100.0%** | 10.27 |
| 无中位 + 关遮挡/关深度边缘/关门控（`replay_official_aligned_20261115`） | 1269 | **5.88** | 5.42 | 32.1% | 11.89 |
| 无中位 + 保留默认遮挡/边缘/门控（`replay_singleframe_20261115`） | 1188 | 6.06 | 5.37 | 31.5% | 11.85 |
| 官方参考 `retail_street_livo_rgb_v2.pcd` | — | — | 5.66 | 30.7% | 12.43 |

- 中位回填对平均色**几乎无影响甚至略差**（5.91 vs 5.88），却把 1cm 体素内色 std 从 5.42 抹到 0.00、平色体素从 31.5% 拉到 100%——局部纹理被彻底抹平，这正是肉眼"发糊/不实"的来源，而平均色指标完全看不见。
- 关掉遮挡/深度边缘/质量门控后，MAE 与纹理都落在噪声内（5.88 vs 6.06、5.42 vs 5.37，且帧数 1269 vs 1188 有混淆），故保留这些剔除。
- 生产配置 A / B 均改为 `pcd.median_voxel_size: 0.0`，回到官方口径"逐帧逐点保留原始采样、不做任何融合"。

**产物**：`solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml` 与 `solution_b_tight_coupled/super_livo_visual_b/config/visual_b_params.yaml` 的 `pcd.median_voxel_size: 0.0` 及注释；`pointcloud_colorizer/include/pointcloud_colorizer/voxel_color_median.hpp` 仅注释（类保留为可选后处理）；`experiments/octvox_ablation/_probe_color_texture.py`；`/home/ywj/slam_data/replay_official_aligned_20261115/`。

---

## 31. Super-LIO × 方案 A 整段彩色点云：图像配对偏斜 0.09 s（2026-11-15）

**前提/动机**：按当前最新流程参数（A 染色：无中位回填、保留默认遮挡/深度边缘/质量门控）对 Retail_Street 全段出彩色点云，位姿与点云全部由 Super-LIO 提供（`/lio/odom`、`/lio/cloud_body_dense`、`/lio/cloud_world`）。

**流程**：`16_run_superlio_a_color.sh`（`sed '/^ *app\./d'` 生成 lio 参数、等 "Map init done"、起 A 节点、回放 bag、SIGINT 让节点析构落盘）。首跑只着 **126 帧 / 1,635,730 点**：日志显示 `t_img` 与 `t_cloud` 的差从 +0.0009 s 增长到 0.0928 s，超出 `color.sync_max_offset=0.05`，此后图像全部配不上、触发 1075 次"等图像队列溢出"丢帧。给脚本加 `SYNC_MAX` 透传 `color.sync_max_offset`，用 0.15 重跑。

**结论**：15,491,400 点 / 着色 1164 帧 / 丢弃(无图像)=2 / 丢弃(等位姿超时)=10，与 20261114 同脚本基线（1164 帧 / 15,432,427 点）一致。5cm 体素平均色 MAE 24.45（基线 24.61），与官方参考的匹配距离中位 46.4 mm。**该量级 MAE 来自 Super-LIO 轨迹与官方轨迹 46~53 mm 的几何差，不能用于跨位姿来源比较染色质量**（同 bag 官方 FL2 位姿下为 5.9 量级）。

**产物**：`/home/ywj/slam_data/colored_superlio_a_20261115r2/`；`16_run_superlio_a_color.sh` 的 `SYNC_MAX` 段。

---

## 32. Super-LIO 地图层 k=5 / sub_div=2 复跑（2026-11-15）

**前提/动机**：§24 已记录 exp 分支默认 K=8，与历史基线默认不可比；按用户指定回到 `plane_knn_k=5` 以恢复可比性，同时把 `sub_div` 由 4 降到 2，验证提速幅度与质量代价。

**流程**：`16_run_superlio_a_color.sh` 新增 `SUB_DIV` / `KNN_K` 透传（`-p lio.hash_map.sub_div:=2`、`-p lio.hash_map.plane_knn_k:=5`，启动日志确认生效），其余与 §31 相同（`SYNC_MAX=0.15`、A 染色无中位）。

**结论**：Observe 平均 6.66 → **3.56 ms（−47%）**；地图 15,974,907 点 / 着色 1201 帧。与官方参考的匹配距离中位 46.4 → 52.9 mm（+6.5 mm），5cm 平均色 MAE 24.45 → 24.41（噪声内）。即 `sub_div=2` 的代价是几何一致性略降，染色指标看不出差异。
另记一条复现性事实：同一命令首跑出现图像流**整段断供**（0 着色帧、末尾"最近差=134.8"），原样重跑即恢复正常——CycloneDDS 下偶发丢相机订阅。判读时若见"着色帧=0 且最近差为百秒级"，应重跑而不是改参数。

**产物**：`/home/ywj/slam_data/colored_superlio_a_k5d2_20261115b/`（失败的首跑目录 `..._k5d2_20261115/` 保留）；脚本 `SUB_DIV / KNN_K` 段。

---

## 33. 方案 B 染色链路修复：body 云 QoS 不兼容 + 双口径复跑（2026-11-15）

**前提/动机**：方案 B 的染色参数已与 A 完全一致（同一 `pointcloud_colorizer`、`median_voxel_size=0.0`），需按 §31/§32 同口径出整段彩色点云，并区分两种口径——`color.use_own_odom=false`（几何与相机位姿全取 `lio/odom`，与 A 同输入，差异只剩节点内部实现）与 `true`（世界变换与相机位姿都换成统一估计器的修正位姿）。

**流程**：新建 `16b_run_superlio_b_color.sh`，两口径各跑一遍完整 LIO + 回放（同 `SUB_DIV=2`、`KNN_K=5`、`SYNC_MAX=0.15`）。首跑两口径**均 0 帧、无 PCD**，`vis.log` 只有一条 `New publisher discovered on topic '/lio/cloud_body_dense', offering incompatible QoS ... RELIABILITY_QOS_POLICY`。

**结论**：根因是 B 订阅 dense 云用 `reliable`，而 Super-LIO 发布端是 `best_effort`，QoS 不兼容时 DDS 直接不投递 ⇒ 整段收不到任何 body 云（与 A 早已修正的问题同源，A 侧注释即记录此坑）。改为 `best_effort`（同时兼容 FL2 的 reliable 发布端）重编译后，`b_lio` 出 16,549,880 点（264 MB）。`b_own` 仍无产物：日志 1219 条队列溢出、末尾"最近差=128.5"，且无"彩色地图已保存/彩色统计"行，属 §32 记录的图像流偶发断供 + 退出未落盘，需重跑，**不能据此判 B 自身位姿的优劣**。

**产物**：`solution_b_tight_coupled/super_livo_visual_b/src/node/visual_b_node.cpp` 的 body 云 QoS 段（L103–108）；`experiments/octvox_ablation/16b_run_superlio_b_color.sh`；`/home/ywj/slam_data/colored_superlio_b_20261115/b_lio/`。

---

## 34. 方案 B 双口径复跑：b_lio 与 A 等价，b_own 速度发散（2026-11-15）

**前提/动机**：§33 里 `b_own` 三跑均 0 点，需先排除节点自身 bug，才能判断紧耦合（自身修正位姿）到底有没有让层裂/发糊变好。

**流程**：先定位并修掉两个独立 bug，再按 `BAG_RATE=1.0 SUB_DIV=2 KNN_K=5 SYNC_MAX=0.15`、`TAGS=lio own` 两臂全量重跑。

1. **own 模式位姿循环依赖**：`processVisualFrame` 取视觉帧位姿走 `poseAtStamp`，而 `color.use_own_odom=true` 时它查 `own_pose_cache_`；该缓存只在本函数末尾（估计器更新后）写入 ⇒ 首帧永远等不到位姿，视觉帧 0 条、`own_pose_cache_` 恒空、着色全部「等待位姿超时」→ 0 点产物。改为视觉帧位姿固定取 `/lio/odom` 的 `pose_cache_`。
2. **图像整段丢失**：`/left_camera/image` 用 `SensorDataQoS`（best_effort depth 5），单线程 executor 被 10 Hz 密集点云回调占满时 best_effort 读者整段静默丢样且不恢复（日志里最新图像 header 冻结）。新增参数 `image.qos_reliable`（默认 `false`，保持旧行为），实验脚本置 `true`，与 rosbag2 回放端的 RELIABLE 兼容。

**结论**：

- `b_lio`（与 A 同输入口径）：帧#1281、17,130,040 点、100% 有彩，0 次图像队列溢出/等待超时。纹理与 A（k5d2）基本一致：1 cm 体素内色 std 7.21 vs 7.26，5 cm 13.45 vs 13.59，平色体素(std<2) 28.6% vs 28.7%。另用早前「同一段 replay 输入」的 A/B 产物逐点对比：共用点 91.3% 颜色完全相同、MAE 0.36 ⇒ **B 的染色链路与 A 等价**（新跑的 A 与 b_lio 是两次独立 LIO，逐点 NN 中位距 17 mm，不能直接逐点比色，只能用纹理指标 + 该同输入对比）。
- `b_own`（自身修正位姿）：前 ~50 s 上色率 0.60–0.67，约 50 s 降到 0.35，~57 s 起永久 `0/21574 上色`；最终仅 6,334,382 点（只剩前 57 s 地图）。日志 `|v|` 单调发散 0.5→2.4→6.2→24→42→69→125 m/s，同期一次 `[Guard] lio/odom 单帧跳变 2.37 m / 10.75 deg`（`b_lio` 同样出现一次但不受影响），而 `守卫: 速度限幅=0`。机制：`clampSpeed` 只在 `degen_sev >= guard.sev_threshold(0.9)` 分支调用，Super-LIO 未发出 `lio/degeneracy`（节点报「未收到退化签名」超时），退化分支永假 ⇒ 绝对上限 3 m/s 从未生效，own 位姿飘出图像后投影 0 点。**`b_own` 当前不可用，不能作为紧耦合优劣的判据**（暂不修，按决定先固化 `b_lio` 结论）。

**产物**：`solution_b_tight_coupled/super_livo_visual_b/src/node/visual_b_node.cpp`（`poseFromSamples` 助手、视觉帧固定 LIO 位姿、`image.qos_reliable`）；`experiments/octvox_ablation/16b_run_superlio_b_color.sh`（`COLOR_EXTRA+=(-p image.qos_reliable:=true)`）；`/home/ywj/slam_data/colored_superlio_b_20261115/{b_lio,b_own}/`。

---

## 附：数据集与输出目录清单

**数据集**（均在 `/home/ywj/slam_data/`，另有 `dev_ws` / `dog_slam` 消息包依赖）：

| 数据集 | 规格 | 用途 |
|---|---|---|
| `Retail_Street` | 1.99 GB / 135.47 s / 30157 条；lidar 1355、imu 27447、image 1355 | OctVox 消融、body 云验证、按帧染色、RGB 标定对比（§2/4/5/14） |
| `rosbag2_fastlivo2_2026_10_30-18_00_25` | 160.3 s / 35239 条（PointCloud2 lidar 1602、imu 32033、image 1604） | §1 三算法对比与 yaw 漂移诊断 |
| `geom_replay_20261110/replay3(+replay_full)` | 4 话题同窗重放（1345/27242/1307/1315） | §3 离线几何与反插对照 |
| `octvox_ablation_20261110/`、`octvox_closedloop_20261110/` | `inputs.npz`、`map_{A,B,C,D}.npz`、`pcd/`、逐帧残差 CSV；`rep_{A,B,C}/` | §4 |
| `ntu_virna_data/` | NTU VIRAL 07/09/13 bag 与 e1_2 系产物 | §6/8/11/12/15 |
| `fixed_world_map/` | 官方 FAST-LIVO2 NTU 产物 + 本仓库同 bag 产物 | §6/8/9 |
| `SMBU/` + `fastlivo2_smbu_out/` + `fastlivo2_all_{0,1,2}.log` | SMBU a/b/c 全部场景 | §6/8 |
| RTK-SLAM Dataset（15 个含 `/gnss/fix`，5 个已跑） | `fastlivo2_rtk_out/`、`rtk_smbu_eval/`、`superlio_rtk_eval/`、`smbu_rtk_001_superlio_repro/`、`smbu_rtk_010_superlio/` | §7/8/10 |
| `livo_bag/{all.bag→all_ros2, move.bag→move_ros2, move_livoxclk}` | 标定包（板距 ~2 m）+ move 包（1280×1024 bgr8 + CustomMsg） | §25 外参标定链与远场验证、§28 幽灵层归因；输出 `/home/ywj/slam_data/output/move_{a,b}(,_holefix)` |
| `livox_record_{noshell,drone,drone_1f}` | 683.8 s/6789 帧；466.9 s/~297 m；637.4 s/~775 m | §13（结果在 `third_party/Super-LIO/exp/results_*`）；noshell 早期跑图在 `pcd/fl2_livox_bag/`、`pcd/superlio_livox/` |
| `pcd/fl2_bag_{lio,livo}/`、`pcd/superlio_bag/`、`pcd/retail_street_{livo_official,livo_wrongcalib,superlio}/` | §1/§2 全部 PCD 输出与重放/着色系列 | §1/§2 |

**代码内产物**：`experiments/fl2_img_ablation_result.json`、`fl2_odo_diff.json`、`experiments/*.png`（`frame_color_full_preview`、`pcd_color_*`、`retail_street_*`、`retail_bk_white_*`、`ntu_eee03_traj_cmp`、`walloff_*`）、`experiments/fastlivo2_bag_yaw_drift_report.html`、`scripts/run_smbu_rtk.sh`、`scripts/rtk_eval/`。

**声明的未入库项**：`solution_{a,b,c}/README.md`、`solution_a/test_results.md`、测速原始 CSV；`/tmp/probe_body_color_v3`、`/tmp/fz_src`、`/tmp/sl_build`、`/tmp/fz_install` 为临时目录（随 WSL 重启清空）。

**上游依据**：`third_party/Super-LIO/stairwell_drift_diagnosis.txt`（M.0–M.40 原始记录，约 488 KB）与 `docs/HANDOVER_2026-11-08.md`（§0–§22）；本文件 §16–§24 为其索引级压缩，数字以原文为准。§25–§29 依据 2026-11-13~15 的标定/测速/提交记录与根 README「关键实验结论」、两份方案 README 退化章节、`experiments/speed_test.md`。
