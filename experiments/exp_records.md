# 实验记录总账（exp_records）

> 本文件汇总仓库**所有实验**的前提、流程、结论与产物，每条结论均保留文档/日志中的原始数字。
> 详细论证见 `docs/HANDOVER_2026-11-08.md`（§编号）与 `docs/` 专项文档；本文件是索引级总账。
> 最近更新：2026-11-16（本轮补录 §38：**相机-雷达外参用同步后标定参数 + `all_ros2` 全量点云复标定，并三臂（FL2/A/B）跑 move**。① 标定：FAST-Calib 直出 RMSE **7.9 mm**（8,260,032 点 + `img_0120.png`），其外参与 remote7.9 逐位一致；holefix 新解使四孔重投影 **1.00/1.62 px**（FAST-Calib 1.26/2.17、旧 holefix 1.25/1.65）⇒ 定稿并写入 A/B/FL2 三份配置；② move 跑批：A 739 帧 / 73.8 s / 9.554 m / 闭环 0.029 m、B 739 帧 / 9.557 m / 闭环 0.024 m，**A-B 逐点最大差 5.0 mm**、彩色地图 3,342,799 / 3,351,876 点；FL2 691 帧 / 72.1 s / 10.834 m / **闭环 0.344 m**，其 t≈16.9 s 的 0.59 m 跳变与 bag 图像 **2.634 s 空洞（14.87–17.51 s）**、12 条「IMU and LiDAR not synced 0.50–0.56 s」同刻；③ 新旧外参对照：彩色图着色统计无差（均值 RGB 122.5 vs 122.1、黑点 0.2%），远场边缘 DT 指标两版不可分辨（#592 15.78 vs 16.41 px、#296 2.98 vs 3.07 px）。上轮补录 §37：**方案 B 现状态 Retail_Street 三轮跑批——融合轨迹无回归，但 `visual_b_app` 无法优雅退出 ⇒ PCD 只能靠周期落盘**。① 默认参数（`pcd.save_interval_sec=30`）：`/lio/odom` n=1293 / 134.10 s / 路径 **66.83 m** / 闭环 **23.0 mm**，注入 push=1283 生效=1248（97.3%），彩色地图 **16,815,272 点 / 269 MB**（5 次定时落盘 2.82M→6.57M→10.85M→14.92M→16.82M）；② `save_interval=0`：`/lio/odom` 66.81 m / 闭环 34.2 mm、注入生效 1281，**无 PCD**（SIGINT 无响应，45 s 后 SIGKILL，析构落盘不执行）；③ reliable QoS 未消除 **t≈42 s 的 3.2 s 数据时间空洞**（三轮 + 11-14 旧运行同样存在；原始 bag 该段 lidar/imu/image 全连续、最大间隔 0.1195 s ⇒ 丢样在消费侧，根因未定位），该轮 B 自有 odom 还异常（85.37 m / 闭环 16.35 m / z 到 −2.99 m）。三轮融合轨迹 66.80–66.83 m / 闭环 23–35 mm，与 11-14 旧代码 B 无回归。上一轮 §36：**pose_refiner Phase 0 离线判决——P0-A~F 全部完成，判决为「开（带前置条件）」**。① 口径订正（外参侧别 b→c、IMU 右乘积分、Δt_camera=110 ms）；② P0-C/D 通过：**旋转优先路线成立**（视觉旋转随机误差 0.043°、σ_full 0.0053°、协方差 `(JᵗJ)⁻¹` 可信、三边自洽证视觉与 LIO 误差独立）；③ P0-E 完成但**结论是验收指标本身不合格**：旋转→法向厚度实测斜率仅 **12.2 mm/°**（解析高估 5.6×），0.13°→0.05° 只值 0.97 mm，本底 6.79 mm 中 **5.05 mm 与位姿无关** ⇒ **`≤5 mm` 靠修位姿物理不可达**，改为「去曲率 hf_θ + 平面芯区间距 sep_cm」主判据；平移通道仅贡献 0.16 mm ⇒「分层由平移主导」的反向假设不成立；④ P0-F 通过：修掉两处数值缺陷（`arccos(clip(trace))` 把 ≤0.1° 误报为 0、协方差多乘 D2R 使先验支配）后，去共模区间误差 **0.1387°→0.051°**（达纯图像链天花板 0.0489° 的 1.10 倍）、真实正弦保留 89.4%、**公平的无延迟 SG 平滑基线四档全部劣于不平滑** ⇒「只要平滑不要图像」的分叉关闭；门 5（绝对残余 ≤0.05°）**判为不适定不作硬门**，改用表 E2（已到共模不可观下限）；⑤ **作废** §34–35 链条中 `19a`/`19d`/`18` C-D 段的绝对数字与第一版 19e、第一版 19i 的结论；⑥ §36.11 **E0 前置否决门实测通过**：LIO(0.1 s) 区间旋转误差 **0.1493~0.1619°** 是图像链 0.0489° 的 **3.0~3.3×**，e ∝ Δt^0.211（非白噪声），分解 c=0.1381° 与 P0-F 基线 0.1387° 逐位吻合 ⇒「余量归零」风险闭合；附带发现视觉误差 lag-1 自相关 +0.461 ⇒ 初判 W3 约 1.5× 过自信须改 AR(1) 噪声模型（**该初判已被下述⑦实测推翻，终结论见 36.12**）；并**纠错** §36.2 数据行「相机 1 Hz」为误记（实为 ≈10 Hz）；⑦ §36.12 **R10 数值验证探针**：初稿「W3 过自信 1.52× ≈ AR(1) 膨胀 1.65×」为口径错配（实测边际/口径比仅 **1.08×**），逐轴 ρ=[0.024, 0.802, 0.488] 强各向异性，λ0 灵敏度极平且尺度不变性成立 ⇒ **R10 不阻塞 M2**，P0-F 的 λ0=0.30/clip=0.30 可直接移植 C++；ρ≈0.80 轴归 E4，36.11 附带产出②初稿结论已相应改写；⑧ §36.13 **M2 C++ 移植落地并模块级复现**：新增 `rotation_refiner.{hpp,cpp}`（只依赖 Eigen、body 系输入、不含外参），与 `19i` 的 `run_slide` 逐帧对齐 **max|Δu|=2.3e-6°**，`imu_dt` **0.1428°→0.0532°**（两侧完全相同），`colcon build` 通过；尚未接入节点；⑨ §36.15 **E3 前置可探测性**：向 LIO 旋转注入 0.05/0.1/0.2°，用同一运行的点云+位姿重建五臂地图（1313 帧、各 483.6 万点），**0.1° 在地图指标上不可分辨**（kNN 厚度 +0.12 mm / `hf_θ` +7.1 mdeg / 贴片 P90 +0.48 mm，本底 13.6 mm），0.2° 刚可分辨（+0.54 mm / +34.9 mdeg / +1.91 mm），2° 正控明显 ⇒ 关闭「地图分层能分辨 0.1°」这一假设；「重建图 vs 节点图差 ~4 cm」经 ICP 判定非刚性（平移 ≤5.6 mm、转角 0.064°），属 OctVox 代表点稀疏；⑩ §36.16 **NTU 序列筛查**：本地只有 `eee_03` 原始包，其 GT 净圈数 **−0.04**、回起点 2 次、远离帧最近距离 P50 **2.45 m** ⇒ **不是多圈**；真多圈是 `eee_01`（净 1.66 圈、6 次回起点、78.6% 帧有 >30 s 重访伙伴、19 段≥1 s 重访中最长 29.6 s），但原始包未下载（4.53 GB）⇒ E3 的"多圈错位放大"方案在现有数据上不可做；⑪ §36.17 **E3 多圈终判（eee_01 下载后实测）**：精简基线（CycloneDDS + lidar reliable）odom 3,754 条 / span 398.0 s / 最大间隔 0.177 s 无断流，ATE(RMSE) **0.098 m**；895 个重访对（时间差 P50 181 s）、15,525 个配对 patch，arm0 跨趟偏移 |m| 中位 **23.3 mm**/P90 47.1 mm；**0.1° 注入的地图效应只有 RMS 4.4 mm（逐帧随机）/1.3 mm（半趟系统 yaw）**，0.2° 也只有 8.0/2.7 mm，法向夹角通道 0.1°→79.4 mdeg 随机 / 2.8 mdeg 系统（天然不一致度 566.6 mdeg）⇒ **E3 判据不成立：0.1° 在地图层面不可分辨，且"可分辨"只在有 arm0 反事实配对时才拿到**；M5 的证据只能走位姿级（§36.11 的 3.0~3.3×）。上轮补录 §35：方案 B `b_own` 发散三层修复后恢复全程彩色点云；§25–§29：外参标定链 holefix 与远场验证、RK3588 绑核四档对照、B 退化对抗 8 跑、move bag 幽灵层归因、FAST-Calib-ROS2 fork 治理；上轮补录楼梯井诊断 M.0–M.40 全系列与 HANDOVER §19.11–§22，见 §16–§24）；⑫ §36.18 **评测协议补全**：`tools/odom_vs_gt.py` 增区间旋转 RPE（IMU 陀螺积分参照 + Kabsch 常值旋转 + 逐轴去中位 + 时间偏移扫描），合成自检（注入 0.15°/帧 → 0.365°）与真实数据双向验证；**eee_01 LIO 0.1 s 区间旋转误差中位 0.1741°/RMS 0.3184°/偏航 0.0955°**（=图像链 0.0489° 的 3.56×，独立复现 §36.11 的 E0 门），同时位置 ATE 0.098 m 与 §36.17 逐位一致；⑬ §36.19 **旋转精修接入节点终判（作废上轮「r_vis 反号」）**：`PatchObservation::u/v` 为当帧 LK 像素、`a=prev/b=cur`、`R_b2b1=R_bc·R_c2c1·R_bcᵀ` 均正确，插值（slerp）与 C++ 估计器（合成单测 0.0069–0.0116°）也已排除；上轮 `M≈diag(−1.1,−1.16,−1.22)` 是诊断脚本把区间旋转取成 `R_{k-1}←R_k` 的反号伪影。真实根因是**视觉噪声被低估 40–100×**：W3 主轴 0.0235/0.0278/0.0567°、估计器自报 σ 中位 0.0020/0.0044/0.0083°，而实测残差稳健 σ = **0.3001/0.2194/0.1499°**、RMS 0.4121/0.4133/0.7851° ⇒ `|u|` 中位 0.2027°、**44% 帧顶到 clip 0.30°**；长尾随 `px_med` 明显（>1.0 px 档 RMS 2.766°，`corr=0.651`）。视觉增量在中位上优于 LIO（图像自身区间 0.1580° vs 0.2277°）但 RMS 差 2.4×（0.8642° vs 0.3531°）。真实 `rotation_refiner.cpp` 离线重放：`w3` 现行档 0.2864°（略差于 LIO 0.2810°、偏航 0.0385→0.0532°）；换 **σ=0.3° + `px_med≤1.0 ∧ n_inl≥100`** 后 RMS **0.2810→0.2573°（−8.4%）**、中位 0.1183→0.1100°，但偏航退化到 0.0469°；clip 放开无额外收益。⇒ 现行默认不开该通道，W3 不能当线上有效噪声，§36.14 的「0.1387°→0.051°」是去共模口径、与线上绝对 RPE 不可直接对比。

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

## 35. b_own 发散根因排查与三层修复：恢复全程彩色点云（2026-11-15）

**前提/动机**：§34 判定 `b_own` 不可用并「暂不修」。本轮按任务要求排查该发散，目标是让 `b_own` 也能输出与 `b_lio` 可比的正常彩色 PCD。

**排查（三条独立证据链）**

1. **守卫确为死代码**：整段 `守卫: 速度限幅=0`、`退化 sev=0.00` 恒定，WARN 在案「未收到 `lio/degeneracy` 退化签名（超时 5 s）」。`clampSpeed` 原先只挂在 `degen_sev >= guard.sev_threshold(0.9)` 分支下 ⇒ 绝对速度上限从未参与。
2. **发散与"用谁的位姿上色"无关**：同一 bag 下 `b_lio` 与 `b_own` 的估计器 `|v|` 同段都涨（末值 113 vs 125 m/s，见各自 `vis.log` 的 `视觉帧:` 行）。根因是**速度维度没有任何观测**：视觉为 `pose_only`（`dx` 速度块恒 0），D2 激光先验只注入 R/p 块，IMU 传播留下的速度误差自由随机游走。`b_lio` 之所以看不出问题，只是它的世界变换与相机位姿全取 `/lio/odom`，估计器状态飘了也不进产物 —— 即 §34 的"b_own 独有问题"实为估计器本体缺陷第一次暴露到产物上。
3. **结构性上色瓶颈（决定性）**：Retail_Street 相机 **1 Hz** ⇒ 视觉锚点间隔 1.0 s；`kPoseHistoryMaxGapSec=0.5` 使 `poseFromSamples` 直接拒绝落在两个锚点之间的所有 10 Hz 点云查询，`kPendingColorTimeoutSec=1.0` / `kMaxPendingColorFrames=8` 再把等待队列里的帧丢掉 ⇒ own 口径理论上限只有约 1/10 的点云能上色。只补到第 2 条时实测：`|v|` 被钳在 6.16、`速度限幅=570`，但上色率仍在 0.63/0.05/0.00 之间跳，产物只 9,509,003 点。

**修复（三处，均在 `super_livo_visual_b`）**

1. `guard` 重构：绝对速度上限改为**无条件生效**（原挂在退化分支内），退化段仍按 `vel_ref × vel_ratio` 收紧并再压到绝对上限之下；`guard.vel_abs_cap` 由 3.0 提到 6.0。
2. `buildLidarPrior` 增补**速度观测块**：新增 `lioVelocity(t_pose)`，在 `/lio/odom` 缓存上取 ~0.1 s 基线差分（要求 `dt∈[0.05,0.30]`、`|v|<10 m/s`，位置取 `(T_wl·T_li)` 平移），按 `lv.prior_vel_sigma=0.25`（该记录为改名前命名，现名 `update.prior_vel_sigma`）注入 `H(kVel,kVel)`。配套把 `UnifiedEstimator::lidarUpdate` 的步长限幅按**实际先验间隔**放宽：新增 `lidar_dt_accum_`，`k=min(10, max(1, lidar_dt_accum_/0.1))`，限幅逐项 ×k（1 Hz 视觉时 k=10），否则 1 s 间隔的先验会被按 0.1 s 标定的限幅压死。
3. **own 轨迹与 `/lio/odom` 同频**（关键）：新增 `pushOwnPoseSample()`，对最近一次视觉锚点的融合残差做零阶保持 —— `R_own = ΔR_anchor·R_lio`、`p_own = p_lio + Δp_anchor`，残差带信任域截断 `color.own_res_max_pos=0.5 m` / `color.own_res_max_rot=0.035 rad(≈2°)`；`onLivOdom` 每帧（10 Hz）落 `own_pose_cache_`，`poseAtStamp` 在 own 口径改查该缓存。1 Hz 锚点不再造成位姿空洞。

**结果**（`BAG_RATE=1.0 SUB_DIV=2 KNN_K=5 SYNC_MAX=0.15 TAGS=own`，LIO 建图参数与 §34 b_lio 基线一致）

| 口径 | 点数 | 上色行数 | 末帧上色率 | `|v|` 末值 | 速度限幅 | bbox (m) |
|---|---|---|---|---|---|---|
| b_own §34（修前） | 6,334,382 | — | 0/21574（57 s 起归零） | 125 | 0（死代码） | — |
| b_own step1（仅第 2 条） | 7,446,706 | 123 | 0/18967 | 37.20 | 0 | — |
| b_own step2（+第 1 条与间隔放宽） | 9,509,003 | 121 | 8821/19920 | 6.16 | 570 | 143.9×158.8×52.1 |
| **b_own 修后（+第 3 条）** | **17,113,313** | 122 | **14157/19920 (71.1%)** | 6.21 | 393 | 128.4×126.2×36.2 |
| b_lio 基线（§34） | 17,130,040 | 116 | 13327/19920 (66.9%) | 113.17 | 0 | 101.4×116.7×36.9 |

末帧均为 `帧#1288`（b_lio `#1281`），`t_cloud` 跨度 946685438.8→946685573.0 与 b_lio 全程一致；纯黑点（`max(rgb)≤10`）0.0%，均色 (134.6,123.1,116.9) vs b_lio (133.3,121.8,115.7)。WARN 画像已与 b_lio 同量（`位姿空洞`3、`待定队列超限`2、`等待位姿超时`2 各臂皆有），own 独有的只剩守卫限速与一次 `lio/odom 单帧跳变` 撤销先验。**`b_own` 现已能输出全程正常彩色 PCD。**

**仍未解决 / 判据边界**

- 速度只是"被限速"而非"被观测到位"：σ_v=0.25 的信息先验乘上已塌缩的 `P_vv` 后 `dx_v` 极小，`|v|` 仍单调爬到 6.0 上限被反复钳住（393 次）。本数据集轨迹 67.5 m / 135 s（均速 ~0.5 m/s），6.0 m/s 不削真实运动，属安全网；但产物精度目前依赖这条安全网，而不是依赖估计器本身。
- own 图比 b_lio 更"散"（**机制在 §36 已定因，不是出图质量波动**）：1 cm 体素内色 std 10.79 vs 7.21、多点数体素 4.7% vs 18.5%、5 cm 体素数 1.76 M vs 0.61 M、bbox 长边 +27 m。**成因是同一帧存在两个位姿源**：逐帧点云的几何/去畸变与地图世界变换走 `/lio/odom`，而图像投影与 own 残差走 `own_pose_cache_`（锚点间零阶保持，下一个 1 s 锚点到来时残差整体跳变）⇒ 两套位姿的**差**被直接注入彩色点，表现为分层/散色。这正是 pose_refiner 方案要消除的 `b_own` 双位姿源缺陷的产物级证据，**不能作为"紧耦合更差"的判据**（见下条结论边界）。
- 结论边界：**b_own 可用于出图，但仍不能作为"紧耦合优于 b_lio"的判据**；层裂/发糊对比仍以 §34 的 b_lio 为准。

**产物**：代码 `solution_b_tight_coupled/super_livo_visual_b/src/node/visual_b_node.{hpp,cpp}`（`prior_vel_sigma_`、`own_res_max_pos_/rot_`、`own_anchor_dp_/dR_/t_`、`lioVelocity()`、`pushOwnPoseSample()`、`buildLidarPrior` 增加 `t_pose` 形参、`onLivOdom` 内 10 Hz own 落缓存、守卫块重构）、`src/estimator/unified_estimator.{hpp,cpp}`（`lidar_dt_accum_` 与步长放宽）、`config/visual_b_params.yaml`（`lv.prior_vel_sigma: 0.25`、`guard.vel_abs_cap: 6.0`；此处键名为改名前写法，现分别写作 `update.prior_vel_sigma` 与 `guard.vel_abs_cap`，且都在 `super_livo_visual` 分支下）。数据 `/home/ywj/slam_data/colored_superlio_b_20261115_fix34/b_own/`（最终产物 273,813,196 B），同级 `b_own_step1_velobs/`、`b_own_step2_capdt/` 为两次中间态留证；b_lio 基线沿用 `/home/ywj/slam_data/colored_superlio_b_20261115/b_lio/`。

## 36. pose_refiner Phase 0 离线判决：口径订正 + 图像旋转通道能力实测（P0-A~F 全部完成，2026-11-15~16）

**前提/动机**：方案 v2（`docs/../.trae/documents/image-pose-hf-refiner.md`）拟用图像相邻帧相对位姿作为独立观测，对 Super-LIO 的逐帧位姿做高频修正。外部评审指出**整个方案唯一的硬假设是"图像相对旋转本身足够准"，而这一条恰恰没有被验证过**，并要求 Phase 0 必须先区分"随机噪声 vs 系统偏差"、必须做特征子集重复估计、必须重列 hf 指标与地图厚度的对应关系。本轮**只做 Phase 0、不写一行 Phase 1 代码**，全部数字实测。

数据：Retail_Street（**相机 ≈10 Hz**，与 LiDAR 头戳 1:1 同戳；P0-A 实测 `std=0`，`19e` 用 `read_lidar_stamps(stride=1)` 配 `(k,k+1)`，故视觉区间 = LiDAR 帧间隔 = 0.1 s。**本节初稿此处原写「相机 1 Hz」是误记**，订正依据见 36.11；LiDAR ≈10 Hz、IMU 203 Hz），LIO 轨迹 `/home/ywj/slam_data/retail_abl/base.txt`（TUM，1317 行，比 LiDAR 头戳晚 0.796 s 起）；有效窗口 k∈[90,1290]，取样 k=100 起。

### 36.0 本轮先作废自己先前的结论（错误链条留证）

| 脚本 | 状态 | 原因 |
|---|---|---|
| `19a_rot_extrinsic_probe.py` | 绝对数字**作废** | ① 外参侧别反了（见 36.1）；② 视觉解用裸 5-point/Procrustes，低视差退化 ⇒ 它的 1.293° 大部分是估计器退化，不是外参 |
| `19d_rot_debias_real.py` | 残差绝对值**不得引用** | 参照旋转在错误共轭侧别下计算，且 Δt 仍按旧假设只扫 ±100 ms（真值 110 ms 在网格外）⇒ 结论偏悲观 |
| `18_timealign_probe.py` C/D 段 | 残差绝对值**不得引用** | 用 5-point R5，同样退化 ⇒ Δt_camera 改由 36.2/19e 判；**A/B 段有效** |
| 第一版 `19e`（曾判"旋转优先路线不成立"，1.6~2.0°） | **作废并撤销** | 建立在两个口径错误之上（外参侧别 + IMU 积分方向/时间相位），改正后重跑结论**反转** |
| 「原生 hf_θ P50=0.51° 是可修抖动」 | **口径失效** | 见 36.3：其中 95% 是真实角运动曲率 |

已在上述脚本头部写入作废/取代说明，防止后续被再次引用。

### 36.1 三条被实测钉死的口径（P0 的真正前置工作）

1. **外参侧别**：`V.R_CL` 实测为 **b→c**（v_c = R_CL·v_b；重力检验 + 侧别网格 + 3-DOF 自由拟合三方一致）⇒ `R_wc = R_wb·R_CLᵗ`，body 区间相对旋转映到相机系的共轭统一写成 `b2c(L) = R_CL·L·R_CLᵗ`（各脚本统一走这一个函数，不再手写转置位置）。
2. **IMU 积分方向**：陀螺在 body 系 ⇒ Ṙ = R[ω]× ⇒ 离散递推**右乘** `R(k+1)=R(k)·Exp(ω̄Δt)`。旧版左乘把 IMU 参照污染 0.454°，B/C/D 三段旧数字全受影响。
3. **时间语义（评审第 3 条）**：实测 LiDAR 周期 99.8 ms、**扫描时长也 ≈99.8 ms** ⇒ `t_end[k]≈t0[k+1]`，"odom 戳是头/中/尾"在戳上**不可分辨（同一 100 ms 网格）**，v1 争的"半周期相位"是伪问题；真正要测的是**图像内容相对其戳的滞后**，实测 **Δt_camera ≈ 110 ms**（`image.header.stamp` 与 `lidar.header.stamp` 逐帧相等，n=1355）。按 20 °/s 折算 110 ms = 2.2°，比 0.2° 目标大一个量级 ⇒ **必须用"曝光中点时间戳 + 在 LIO 连续轨迹上插值"取代固定减 50 ms**，评审这条完全成立。

`18_timealign_probe.py` A/B 段（有效）另印证：TUM/LIO 轨迹与 200 Hz IMU 积分在 slerp 插值下自洽（偏置扫描极小点处的残差量级 0.1°），即**位姿插值本身没有把真实运动错认成抖动**。

### 36.2 P0-C：图像旋转通道能力实测（`19e_rot_channel_quality.py`，n=500，k=100..599，50 s，123 s）

**A 类（内部自洽，不碰外参、不碰 Δt_camera ⇒ 纯随机噪声地板）**

| 口径 | 裸 Procrustes | 深度辅助(t=LIO) | 换算 σ |
|---|---|---|---|
| 逆一致性 ∠(R̂12·R̂21)（≈√2σ） | 0.003° (n=500) | 0.005° (n=450) | 0.002° / 0.004° |
| 三角闭合 ∠(R̂12·R̂23·R̂13ᵀ)（≈√3σ） | 0.036° (n=495) | 0.027° (n=438, max 3.074) | 0.021° / 0.015° |

**B 类（外部参考）+ 定偏差/随机拆分**（Δt=110 ms；`Log(R̂ᵗ·R_ref)` 去均值）

| 模型 | vs IMU 总 | 定偏差 μ（相机 x,y,z） | \|μ\| | 随机 med | vs LIO |
|---|---|---|---|---|---|
| 裸 Procrustes | 0.351° | [−0.001, **+0.199**, +0.004]° | 0.199° | 0.234° | 0.373° |
| **深度辅助(t=LIO)** | **0.048°** | [−0.006, +0.006, −0.016]° | 0.018° | **0.043°** | 0.130° |
| 参照之间 LIO↔IMU 地板 | — | — | — | — | **0.126°** |

Δt 扫描（0~300 ms，10 ms 一格）：深度辅助模型在 **IMU / LIO 两个独立参照上极小点都落在 110 ms**（0.048° / 0.130°）⇒ 复现 36.1 的 Δt_camera；裸 3-DOF 因无平移模型，极小点被视差耦合偏差推开（vs LIO 落在 100 ms、残差 0.366°）⇒ **不可用裸 Procrustes 判相位**，其 0.35° 也不可当噪声。

**链式漂移** ∠(∏R̂(j→j+1)·R_IMU(k→k+N)ᵀ)：深度辅助 0.081°(N=2) → 0.162°(N=5) → 0.416°(N=20)，N=5→20 比值 **2.57**（纯随机 √N⇒1.41，定符号偏差线性⇒4.0）；裸模型 0.706°→7.129°（比值≈4.0，纯偏差）⇒ 深度辅助项**已把定符号偏差压掉，残余以随机为主**，滑窗优化可以有效摊平它。

**独立性检验（评审第 2 条的关键）**：三边自洽 0.130° ≈ √(0.048² + 0.126²) = 0.134° ⇒ 视觉误差与 LIO 误差**统计独立**，深度辅助模型对 LIO 的泄漏可忽略（由 36.4 直接扰动实验独立印证）。

**P0-C 判决（按评审建议的三档：≤0.10 主路线成立 / 0.10~0.30 条件通过 / >0.30 降级，且只看随机分量）**：随机 **0.043° ≤ 0.10° ⇒ 主路线成立**。先前据第一版 19e 写下的"旋转优先判死"撤销。同时确认评审对"0.0004° 理论值"的批评：实测 σ 比理想值大约 10 倍（理想 0.0004° vs 三角闭合 0.015°），但仍远低于所需量级。

### 36.3 P0-B：`hf_θ` 验收口径失效（`/tmp` 探针，已并入本节）

3-tap 二阶差分 `hf_θ`：在 **LIO 轨迹上 0.5192°**，在同一段的 **IMU 角速度积分上 0.4910°**（比值 1.06）⇒ 所谓"原生抖动 0.51°"中 **95% 是真实角运动的曲率，不是可修噪声**。后果：
- v2 的"纯平滑 Pareto 下限"与"hf_θ P50 ≤ 0.20°"两条验收**同时失效**（任何把 hf_θ 压到 0.20° 的平滑都在削真实转向）；
- 真正的修正对象是 **区间旋转误差 ≈0.13°**（36.2 的 LIO↔IMU 地板），视觉可达 **≈0.05°**；
- ~~换算到 5 m 墙面：5·tan(0.13°)=11 mm → 5·tan(0.05°)=4.4 mm ⇒ 与 ≤5 mm 地图厚度目标首次自洽~~
  **【§36.8 P0-E 实测推翻，勿再引用】**：`d·tan θ` 是**像面切向鬼影位移**，而验收指标 `M1` 用的是 `_probe_layering.py` 的 **k=16 邻域法向厚度**，两者不同量。实测 5 m 档（中位深度 5.04 m）斜率仅 **a = 12.2 mm/°**（解析 88.0 mm/°，**高估 5.6×**），本底 6.79 mm ⇒ 把区间旋转误差 0.13°→0.05° 只得 **5.82 mm（收益 ≈1 mm，非 6.6 mm）**，且 ε=0 也只能到 5.21 mm ⇒ **单靠修旋转达不到 ≤5 mm**。"两套指标差 3~4 倍"没有被消解，反而扩大；详见 §36.8。
- 另按评审第 5 条：Savitzky-Golay 中心窗 5/7/9 在 10 Hz 上要等 200~400 ms，**离线零相位 / 在线因果 / 固定延迟滑窗三种口径不可横比**，纯平滑若要重新上会必须分列。

### 36.4 P0-C 补充：`t_c` 敏感性 / placebo（`19g_tc_placebo.py`，n=150，54 s）

对深度辅助模型的平移先验 `t_c`（来自 LIO 相机系基线）做六种扰动，量"泄漏进旋转解"的量：

| 变体 | vs 基准 med | p90 | vs IMU med | Δ vs IMU |
|---|---|---|---|---|
| A 基准 t_c | 0.0000 | 0.0000 | 0.0448 | +0.0000 |
| B 0.5× | 0.0000 | 0.0000 | 0.0431 | −0.0017 |
| C 2.0× | 0.0000 | 0.0000 | 0.0448 | −0.0000 |
| D −t_c（取反） | 0.0000 | 0.0000 | 0.0448 | −0.0000 |
| E 借用 k+37 的方向 | 0.1871 | 0.5604 | 0.2042 | **+0.1594** |
| F 方向随机转 10° | 0.0726 | 0.1599 | 0.0881 | **+0.0433**（=1.0 σ） |

**结论必须分两件事（合成一句就是错的）**：
- **平移模长零敏感**：×0.5 / ×2 / 取反 与基准**逐位相同**（`dθ̂/d ln|t_c| = 0.0000`），机理是 λ 为自由标量、把 t_c 的整标度连符号一起吸收。两面推论：① 视觉旋转因子**不继承** LIO 的平移模长误差（36.2 的三边自洽得到独立印证）；② 也正因如此，它**不具备修正 LIO 平移模长的能力** ⇒ 评审第 2 条成立：**该因子只是旋转约束，不是六自由度测量，第一阶段平移必须沿用 LIO**。
- **平移方向仍是 LIO 的注入通道**：随机转 10° 就注入一个 σ（+0.043°），借别的帧方向 +0.159° ⇒ Phase 1 里基线方向要当**带协方差的量/状态量**，不能当精确常数。
- 分层旁注：小基线（|t_c|<0.03 m，n=19）基准残差 0.0612° > 大基线（≥0.07 m，n=69）0.0358° ⇒ 门控按 |t_c| 分层、小基线收紧权重。

### 36.5 P0-D：特征子集重复估计（`19f_repeat_precision.py`，n=150，129 s ≈6000 次求解）

评审点名的实验。口径：**disjoint split-half**（两半各 m 点、点集不相交 ⇒ ∠(R_A·R_Bᵗ)≈√2σ(m)），避免"子集相对全集"的收缩偏差；bootstrap 只作条件口径。

基准（全图单解）：|R̂| med 1.404°、n_pts 229、n_inl 219、|t| 0.0652 m、d̄ 10.1 m、像素残差 0.075 px、**内点 4×4 空间覆盖率 med 0.88 / p10 0.75**。共点逆一致 0.0008°（正反共用同一批跟踪点 ⇒ 只反映求解器确定性，必然 ≤ 19e 的独立跟踪口径 0.005°）。

| m/半 | n对 | 两半互差 med | σ(m)=互差/√2 | 模型 σ̂ | σ/σ̂ | σ·√m |
|---|---|---|---|---|---|---|
| 25 | 150 | 0.0227° | 0.0160° | 0.0087° | 1.85 | 0.0802 |
| 50 | 150 | 0.0159° | 0.0112° | 0.0086° | 1.31 | 0.0794 |
| 100 | 126 | 0.0132° | 0.0093° | 0.0062° | 1.51 | 0.0932 |

- `σ·√m` 近似恒定（末/首 **1.16**）⇒ 随机项确实按 1/√N 收敛；外推满特征 **σ_full = 0.0053°**。
- 全解 vs IMU 总 0.045°（定偏差 μ=[−0.006,+0.002,−0.021]°、|μ|=0.022°、随机轴向 σ=[0.029,0.040,0.028]°）⇒ 与 σ_full 对比可得：**0.045° 里约 99% 是与 N 无关的相关误差**（离面点、动态物、畸变残差、LIO 深度误差），**加特征点压不掉**。
- 模型自报 σ̂(JᵀJ)⁻¹ med 0.0039° vs 实测 σ_full 0.0053°（比值 1.34）、各规模 σ/σ̂ 中位 1.51 ≤3 ⇒ **协方差可信**，Phase 1 可直接用 Σ_vis,k=(JᵀJ)⁻¹ 加权。
- 分格敏感性（评审第 2 条"按网格划分特征"）：2×2 单格解 vs 全图解 P50 0.0438°、P90 0.0950°、**最差 2.846°**；6×6（每格≈6 点）P50 0.0603° ⇒ **门控必须加空间覆盖率与分格一致性项，不能只看内点率**。
- 分层：d̄>12 m 的 vsIMU 0.069° > 6~12 m 的 0.039°，而两半互差只从 0.0182° 变到 0.0128° ⇒ 再次印证上升来自**相关偏差**而非噪声。

**P0-D 判决**：重复性达标（σ_full 0.0053° ≤ 0.10°），求解器/跟踪的随机性**不是瓶颈**；瓶颈是相关误差 ⇒ 单帧视觉残差的下限约 0.04~0.05°，不靠 N 而靠**深度源、离面点剔除与自标定**改善。

### 36.6 据实测对方案 v2 的修订（评审逐条对账）

| v2 原文 | 修订（依据） |
|---|---|
| 图像因子提供完整六自由度约束 | **第一阶段只作旋转约束**，平移沿用 LIO（36.4：对 \|t_c\| 零敏感 ⇒ 无平移模长观测能力） |
| 0.0004° 作视觉旋转精度理论依据 | 降级为"理想噪声下限"；实测 σ_full **0.0053°**、总残差 **0.045°**（99% 相关项）（36.5） |
| LIO 戳固定减 50 ms | 改"曝光中点戳 + LIO 连续轨迹插值"，**Δt_camera=110 ms 实测**；头/尾戳之争是伪问题（36.1） |
| 平移/旋转统一门控 | **分离**：旋转门控看覆盖率+分格一致性，平移（若日后引入）看视差与 \|t_c\| 分层（36.4/36.5） |
| 绝对钳位 + 旋转 slew | 补平移 slew、修正后速度/角速度一致（`nav_msgs/Odometry` 的 twist 必须由修正后轨迹算）、门控退出**渐进淡出**、求解失败沿用最近可信解（评审第 4 条；钳位≠连续，硬钳位仍可在 ±0.35° 间跳 0.7°） |
| hf_θ ≤0.20° + 纯平滑下限 | **两条同时作废**：hf_θ 95% 是真实曲率；改对齐"区间旋转误差 0.13°→0.05°"，并要求 P0-E 给出 hf↔厚度实测映射（36.3）。**注：36.8 的实测映射反过来又把"0.13°→0.05° 只值 1 mm"钉死，并判定 `≤5 mm 厚度` 本身不可作验收门** |
| 「分层完全由位姿注入造成」 | 收敛为「当前消融表明帧间位姿注入是**已测试配置下**的主要因素；图像观测质量、时间同步与投影位姿一致性仍需独立验证」 |

### 36.8 P0-E：误差 → 地图法向厚度的实测映射（`19h_theta_to_thickness.py`，52 次成图，114 s）

评审第 4 条（"hf 指标与 ≤5 mm 厚度目标差 3~4 倍，必须实测对齐"）的落地。资产：`inputs.npz` 245 帧×189,127 点，用 replay3 bag 的 `/lio/odom` 按 `header.stamp` 排序配对（|dt|max=0、245/245 唯一）刚体逆变换回 body 系，**往返自检 9.86e-06 m**；成图口径逐字复用 `_probe_layering.local_thickness()`（**k=16 邻域法向厚度**，与 §4 的 M1 同尺）。注入：每帧**左乘**已知幅度、方向各向同性随机的扰动（R 臂只动旋转 / T 臂只动平移模长 / Tc 臂平移块内恒定 25 帧=0.2 Hz），每格 2~3 个独立实现取中位。深度档按 θ=0 名义位姿定档、各注入共用同一批点：**[1.5,2.5) n=390（中位 2.22 m，样本不足，仅参考）/ [3,5) n=9450（4.45 m）/ [4,6) n=15004（5.04 m，主档）/ [8,12) n=108016（9.62 m）**。全体深度分位 3.59/4.92/8.64/9.81/12.0/20.53/30.2 m ⇒ **本 bag 几乎没有 2 m 近场点，"2 m 档"无法回答**。

**（一）旋转通道**：本底（θ=0，含真实 ε0=0.13°）15.30 / 5.89 / 6.79 / 8.64 mm。注入 θ 后的实测增量（主档 [4,6)，mm）：θ=0.02/0.05/0.10/0.20/0.30/0.50/1.00° ⇒ +0.09/+0.25/+0.89/+3.07/+4.26/+7.44/+9.54。

| 档 | 中位深度 | 近端斜率 a(θ≤0.3°, mm/°) | R² | 全程斜率 | 解析 17.455·d | **实测/解析** |
|---|---|---|---|---|---|---|
| 4.45 m | | 13.99 | 0.977 | 11.29 | 77.7 | **0.180** |
| **5.04 m** | | **15.85** | 0.980 | 10.37 | 88.0 | **0.180** |
| 9.62 m | | 13.85 | 0.983 | 7.32 | 167.8 | **0.083** |

标定关系：注入 θ 与实测**区间**旋转误差 **ε/θ = 1.410 ≈ √2**（两帧独立误差正交合成）⇒ 把 0.13° 当单帧误差代入 `d·tanθ` 会再多高估 √2。

⇒ **§36.3 的纸面换算被推翻**（原文已加删除线）：`d·tanθ` 是**像面切向鬼影位移**，验收指标 M1 是**法向**厚度，二者不同量。实测主档 a=12.2 mm/°（按 ε_tot 口径），本底 6.79 mm ⇒ **ε 0.13°→0.05° 只得 5.82 mm（收益 0.97 mm，占本底 14.4%，非纸面 6.6 mm）；ε=0 也只有 5.21 mm ⇒ 达 ≤5 mm 需 ε ≤ −0.017°，物理不可达**。

**（二）平移通道**：注入 s 后主档增量（mm）：s=0.5/1/2/5/10/20/50 ⇒ +0.01/+0.04/+0.05/+0.25/+1.23/+3.72/+10.08。斜率 `b`：近端(s≤5) **0.050 mm/mm**、全程 0.206 mm/mm；实测/自推解析(跨帧法向层间距 √(2/3)=0.816 mm/mm) = **0.062**。自洽校验：实测区间平移误差 = **1.407·s**（白噪声解析 √2）⇒ **注入机制无误，"实测/解析只有 0.06"是指标的问题、不是实验的问题**。Tc（低频块内恒定）斜率折扣 `b_Tc/b_T = 0.28`，且其**区间误差恒为 0**（244 个区间中只有 9 个块边界）⇒ Tc 只作敏感性臂，主判据为 T。

**（三）两通道同口径对账**（3-tap 与"区间"口径不可直接混代，脚本 §9 现算换算因子，非高斯假设）：ε0=0.13°(区间) 直接代入 a ⇒ **X = 1.58 mm**；hf_p=3.85 mm 经注入场自身 3-tap 标定（kT_N=2.317 / kT_A=1.049）反解逐帧绝对等效 **s_eq = 3.19 mm**（逐轴口径匹配 3.01、高斯换算 3.35、W=5 去曲率下界 2.27）代入 b ⇒ **Y = 0.16 mm**（保守下界 0.11）。**Y/X = 0.10（T）/ 0.02（Tc）**；旁档 [3,5)=0.14、[8,12)=0.08、[2,2)=0.64（样本不足）。

⇒ **反向假设"分层主要由平移不连续驱动"未被证实**：在法向厚度这个指标上旋转通道是平移通道的 ~10 倍。但排名**不稳健**：全程斜率口径给 Y/X=0.69；按"造成 +1 mm 所需幅度"内插，旋转 θ*=0.105°（真实 0.0922° ⇒ 饱和度 0.88）、平移 s*=8.83 mm（真实 3.19 ⇒ 饱和度 0.36）⇒ 该指标**分不出主次**（见下述第 2 条）。

**（四）本判决的真正落点：`k=16 法向厚度 ≤5 mm` 不是位姿改进的合格验收量**。三条实测依据：
1. 本底 6.79 mm 中两通道合计仅 **1.74 mm（26%）**，剩余 **5.05 mm 与位姿误差无关**（LiDAR 测距噪声 / 面元粗糙度 / 动态物 / 扫描时间畸变）⇒ 修位姿最多拿回 26%，**≤5 mm 靠修位姿不可能达成**；
2. 该指标对小幅度扰动是**饱和 / 近平方根型**响应（[8,12) 档全程斜率 7.32 ≪ 近端 13.85），排名随口径翻转；
3. 旋转位移 `δ×r` 垂直于视线，`a/(d·tan1°)=0.180` 是**墙面朝向混合后的平均投影**，对正前方墙面实际投影 ≈0%，法向厚度对前视墙的鬼影是**盲区**。

补充口径自查：另建"切向鬼影"代理指标 D（k=16 邻域按帧分组→投影到垂直视线平面→帧间质心分离）。实测**本底 D=72.55 mm**（比注入大两个数量级，每帧仅摊到 2~4 点，被子集真实几何散布主导）、`a_g(θ≤0.3°)=8.44 mm/°`（实测/解析 0.083）、平移臂给负斜率 ⇒ **当前实现的 D 不可用作验收指标**，只作趋势参照。若要真正测鬼影，须改用 `_probe_layering.global_planes` 的**平面芯区多峰间距 `sep_cm`**（厘米级响应、不受局部噪声 floor 压制）或"同一面墙两层点集质心距"。

**（五）据 P0-E 追加修订 v2**：

| v2/v2.1 | 修订（依据） |
|---|---|
| 验收 `地图局部平面厚度 ≤5 mm` | **作废为硬门**。实测其位姿无关地板 5.05 mm ⇒ 降级为**守门指标**（只要求相对 `b_lio` 不劣化）。主判据改为 ① 去曲率的 hf_θ（线性响应）② 平面芯区多峰间距 `sep_cm` |
| "0.13°→0.05° ⇒ 5 m 处 11→4.4 mm" | 改为 **6.79→5.82 mm（+0.97 mm）**；实测斜率 a=12.2 mm/°、ε/θ=√2 |
| `clip_rot` 待定 | 反解表（主档，θ/Δε）：额外分层 ≤0.5 mm ⇒ 0.032°/0.041°；≤1 mm ⇒ **0.063°/0.082°**；≤2 mm ⇒ 0.126°/0.164°；≤5 mm ⇒ 0.315°/0.410° ⇒ 若验收取"旋转通道额外分层 ≤1 mm"，**`clip_rot ≈ 0.06°（单帧）/ Δε ≈ 0.08°（区间）`**；旧口径 `hf_θ≤0.20°` 对该目标**过松 3~5 倍** |
| 平移是否需单独立项 | 通道贡献 0.16 mm（下界 0.11）**不支持**在本场景为平移建因子；但 s_eq 只有 3-tap 一个来源、且 ε0 与 hf_p 出自**不同轨迹**（base.txt vs replay3 odom 段，实测同段 s_eq 3.35~4.04 mm）⇒ 该结论**带 ±20% 风险**，需实数据再判 |

**已知可疑口径（不掩盖）**：① 本底绝对值与 §4 的 7.11 mm **不可横比**（这里是降采样原始点累积，非 OctVox A/B/C/D 代表点成品），判决只用"相对本底增量"；② 2 m 档 n=390≪3000、3 次实现极差 1.7 mm，其斜率与 Y/X=0.64 不具统计意义；③ 注入为逐帧独立白噪声型（对分层最坏、对时间相关误差最"干净"），真实 LIO 误差含低频分量 ⇒ `ε≈√2θ` 与斜率换算**只在本口径内成立**；④ 3.85 mm 的口径归属（逐轴 vs 模长）**仓库内无任何文字确认**，本轮实测同段 逐轴 3.80 / 模长 8.88 mm ⇒ 3.85 只可能是逐轴，若误按模长代入 s_eq 差 2.21 倍；⑤ s_eq 假设真实平移误差逐帧独立，W=5 去曲率残差/3-tap=0.61~0.68 ⇒ 主口径**偏大**，已给下界；⑥ 高斯 MC 因子（f_axis 0.6738 / f_norm 1.5385）仅作交叉核验，主口径已改用注入场自身标定。

### 36.9 P0-F：已知旋转扰动注入 → 恢复力实验（`19i_perturb_recovery.py`，350 帧 k=500..849，159 配置，30.8 s）

评审第 6 条"必须做已知旋转扰动注入→恢复力实验"的落地。方法：阶段一用 `estimate_rotation_d` 逐区间测视觉相对旋转 `R_vis` 并缓存求解器自报 `(JᵀJ)⁻¹`（扫参不重测图像）；把**已知幅度**扰动左乘到 LIO 区间上构成注入轨迹，再用**在线因果滑窗 W=15（0 额外延迟）**的纯旋转因子图逐帧解窗取最新 dθ、热启动；整段稀疏批处理另用作**离线零相位上界**（延迟口径不得与滑窗横比）。S1 绝对钳位 + S2 slew（限**修正量的一阶差分**）以"解完后按序硬投影"实现。自检：解析 Jacobian vs 中心差分相对误差 **7.69e-07**；RNG 全部固定字典种子 ⇒ 冷热逐位一致；`--serial` 校验与并行逐位相同。四档注入：L0 零扰动 / L1 单帧各向同性 0.05° / L2 随机 0.10°（实测 RMS 0.0975°）/ L3 交替 0.20° / L4 正弦（真实转向）+ 噪声。

**（〇）先记账：本轮发现并修掉两处数值缺陷 ⇒ 第一轮（`/tmp/p0f.log`）结论整体作废**
1. `ang_pair` 用 `arccos(clip(0.5·(trace−1)))`：`V.R_CL` 的正交精度只有 ~1e-6 ⇒ `b2c(L)` 后区间矩阵乘积的 trace 会**超出 3**（实测 +8.6e-7）被 clip 截成 1，**凡是 ≤0.1° 的角度被系统性误报为 0**。改为先 SVD 极分解 `so3_proj` 再取角后三个地板重取：**图像 vs IMU 0.0283 → 0.0489°**、**LIO vs IMU 0.1299 → 0.1416°**、**图像 vs LIO 0.1409 → 0.1464°**（自洽：√(0.0489²+0.1416²)=0.1499≈0.1464）。
2. W3 协方差 `g_vi[i]=log_b(...)[0]*D2R` **多乘一次 D2R** ⇒ 协方差缩小 3283 倍、特征值全落 `SIG_FLOOR`、W3 退化成 σ=0.020° 的常数。修正后视觉本征主轴 σ=[0.0236, 0.0277, 0.0453]°（等效 0.0322°）。

⇒ 第一轮"视觉信息量不足、修正器修不动"的归因**不成立**；真正的负结果是下面（二）的**噪声口径伪影**。（另：我曾据 `0.1923≈√(0.1299²+0.141²)` 推"注入没被拿掉"，亦不成立——dB 的参照 `R_clean`=LIO 自带 0.1416° 真实误差 > 注入 0.0975°，拉向图像必然使 res 变大、dB 变负，属"离开 LIO"而非"没在修"。）

**（一）决定性基线（交付①）：纯图像链（零优化）区间误差 vs IMU**

| 跨帧 N | 1 | 2 | 5 | 10 | 20 | 50 |
|---|---|---|---|---|---|---|
| 图像链 med | **0.0489** | 0.0827 | 0.1656 | 0.2368 | 0.3798 | 0.8294 |
| LIO med | 0.1493 | 0.1527 | 0.2392 | 0.2842 | 0.3936 | 0.9234 |

N=1 是**构造恒等**（链的区间就是 `R_vis`），含义是"若完全信任图像，区间误差就落到视觉本征水平"⇒ **区间级天花板 0.0489° = 基线的 0.35 倍**，落在 ≤0.06° 的可达带内 ⇒ **门 1 True**；但链的跨帧累积在 N=20 已达 0.380°（与 §36.2 记录 0.081@N=2 / 0.416@N=20 同量级）⇒ **天花板只对区间级成立，绝对/低频朝向不可用**。

**（二）因子噪声口径是决定性的（交付③）**：`r_k = Log(V_kᵗ·R'_区间)` 对未知量 dθ 而言**噪声项只有视觉本征误差**，基线区间误差 `(a_{k+1}−a_k)` 是**待求信号**。同网格仅换噪声口径（表内为去共模 vs IMU 区间误差，越小越好）：

| 口径 | 等效 σ | L1 | L2 | L3 | L4 |
|---|---|---|---|---|---|
| W1 κ(图像 vs LIO)=43.1 ×自报（第一版标称） | 0.8623° | 0.0857 | 0.1050 | 0.1812 | 0.1149 |
| W2 κ_imu=12.2 ×自报（视觉本征） | 0.2438° | 0.0560 | 0.0606 | 0.0755 | 0.0694 |
| W3 实测视觉本征协方差 | 0.0322° | **0.0515** | **0.0529** | 0.0774 | **0.0692** |

改善比由 W1 的 0.62~1.31 变为 W2/W3 的 **0.37~0.56** ⇒ **"修不动"= 用"图像 vs LIO"定 κ 等于把待消除对象当噪声 ⇒ 先验支配的伪影，坐实（门 3 True）**。副产物：`λ0(sp)` 是主要挡路项；`λ2(sd)=off` 恒优于 0.50；`clip` 0.50 与 1.00 同分（S1 不再激活）⇒ **钳位不是瓶颈**；**S2（slew）在 159 个配置中激活次数恒为 0**，S1 最多 115/335、164/335。

**（三）去共模后的真实改善（交付②）**。去共模 = `g_k=Log(R_imu,kᵗ·R_corr,k)` 逐轴减全段均值 + 减一次线性趋势后取模长中位，**基线走同一变换**（剥掉残余外参/时间戳偏置/陀螺常值偏置等系统性分量，不计入功劳）。不变性自检（表 D.0）：把 1.00° 常值共模精确左乘到修正后位姿，所有**区间级**指标最大变化 **2.62e-11°**（只有位姿级 res 0.077→1.005）⇒ **共模不能白嫖区间级指标**。基线 0.1387° → 修正后（在线因果滑窗、0 额外延迟、clip=0.30）：

| 档 | L0 零扰动 | L1 0.05° | L2 0.10° | L3 0.20° | L4 正弦+噪声 |
|---|---|---|---|---|---|
| 去共模 | 0.0499 | **0.0515** | **0.0507~0.0529** | 0.0559 | 0.0692 |
| 改善比 | 0.36 | **0.37** | **0.37** | 0.40 | 0.50 |

纯图像链极限 0.0463 ⇒ **修正器达天花板的 1.10 倍**。**门 2 True**（主判据档 L1/L2 达标）。

**（四）纯平滑不能顶替图像修正器 ⇒ 分叉关闭**。为公平，专门建**无群延迟/等延迟**的平滑基线：局部二次 Savitzky-Golay（win 5/7/9），`sg-0` 离线零相位、`sg-c` 因果固定 4 帧延迟；另测 EMA(a=0.70) 与中心窗(ctr h=1)（两者更差，0.26~0.88°）。核心指标用去共模 vs IMU 而**不用 dB**（平滑与修正都在"偏离 LIO"，dB 对平滑天然不公平）：

| 档 | 不平滑(基线) | 修正器最优 | 纯平滑最优 | 纯平滑自身达标? |
|---|---|---|---|---|
| L0 | 0.1387 | 0.0499 | 0.2331 (sg-c w=5) | 否（1.68×） |
| L1 | 0.1387 | 0.0515 | 0.2367 | 否（1.71×） |
| L2 | 0.1387 | 0.0507 | 0.2590 | 否（1.87×） |
| L3 | 0.1387 | 0.0559 | 0.3753 (ema) | 否（2.71×） |
| L4 | 0.1387 | 0.0692 | 0.2795 | 否（2.02×） |

⇒ **纯平滑四档全部比"不平滑"更坏，修正器 4/4 档全胜** ⇒"连图像都不要、直接接平滑位姿"这条路在本段实测中不成立，**门 4 True、分叉关闭**，Phase 0 不因它降级。

**（五）"求解器到底在不在修"的干净检验**。表 E 把 δ 注入"干净基线"（先修正出的 `R_base`，其去共模 vs IMU 仍有 **0.0557°** 不可约误差）：res 最优 L1 **0.0407 ✓** / L2 **0.0604 ✗** / L3 **0.0725 ✗** ⇒ 该口径在 `ε(R_base 误差) ≈ δ` 时**根本不可辨**，**门 5 如实保持 False**。唯一干净的判据是把 δ 注入**纯图像链**（未扰动时视觉残差按构造 ~0，res 只含 δ、不掺任何"该往图像拉"的真实信号）：

| 档 | 注入 RMS | 最优配置(W3, sp=1.0, sd=off, cl=1.0) | res med / p90 | 共模不可观下限 | dB |
|---|---|---|---|---|---|
| L1 | 0.0500° | | **0.0141** / 0.0316 | 0.0134 | **+7.79** |
| L2 | 0.0975° | | **0.0206** / 0.0569 | 0.0261 | **+8.02** |
| L3 | 0.2000° | | **0.0138** / 0.0402 | 0.0535 | **+11.01** |

同表 W1 口径仅 +0.10~+0.21 dB ⇒ **求解器确实在修，且已修到窗口内共模不可观的下限（门 5b True）**。

**（六）真实运动保真（门 6）**：L4 正弦（真实转向）保留率 ≥80% 的滑窗配置 **18/18**；最优 W3 sp=1.0 sd=0.5 clip=0.30 给 ret **85.3%**、cor 0.826，可达配置口径给 ret **89.4%**、cor 0.868 ⇒ **不是把真实运动当噪声抹掉**。但如实标注：剥掉正弦后**残余噪声 nz=0.155~0.157° > 0.05° 目标** ⇒ L4 档噪声抑制不达标（该档本身含 0.2° 幅度正弦，绝对值不代表稳态水平）。3-tap `hf_θ` 只作旁证（§36.3：约 95% 是真实曲率）：L2 0.4557→0.4541、L3 0.5825→0.4903、L4 0.4535→0.4541。L0 误修量：res med 0.0590(W1)/0.1264(W2)/0.1515(W3)——注意这**不是坏事**，res 的参照是带 0.1416° 真实误差的 LIO，res 变大只说明"离开了 LIO"。

**（七）写进 Phase 1 的可达配置**：因子噪声口径 **W3（实测视觉本征协方差）或 W2（κ_imu=12.2×自报形状）**；`λ0(sp)=0.30~1.00°`；`λ2(sd)=0.50 或 off`；`clip=0.30°`；**在线因果滑窗 W=15、0 额外延迟**。**禁用 W1**（κ 用"图像 vs LIO"标定 = 把待消除对象当噪声）。

**已知可疑口径（不掩盖）**：① 去共模的"窗口/阶数"（全段一次性减均值 + 减一次趋势，N−1=349 个区间）是自定口径，改成滑窗内去共模数字会变；② 门 4 的等延迟不严格（滑窗 0 额外延迟 vs `sg-c` 固定 4 帧延迟，但后者对二次运动无稳态失真），严格等延迟需把滑窗输出同样推后 4 帧，本轮未做；③ 滑窗 LM `iters=4`（表 A2 末附 3/4/6/8 收敛对照，确认未截断）；Huber/IRLS 只在批处理里生效；S1/S2 是**解后硬投影**，非严格约束优化；④ σ 特征值下限 0.020°、κ 取标量、L4 正弦绕世界 Z 轴、`tc` 取自未扰动 LIO；⑤ 表 0 / 表 E2 的锚定取 `R_clean[0]`，绝对朝向本就不可观，只有跨 N 累积列与锚定无关；⑥ **本轮 `图像 vs IMU = 0.0489°` 与 §36.2 记录的 `0.0283°` 不是同一口径**（后者受 trace-clip 污染）。已审计：`_p0_visual.angle_of()/log_so3()` 同样用 `arccos(clip(trace))`，已就地补 SVD 极分解（`rotm()`）并重跑 `19e` ⇒ §36.2/§36.5 的全部数字**逐字不变**（逆一致性 0.005、三角闭合 0.027、vs IMU 总 0.048 = 定偏差 0.018 + 随机 0.043、vs LIO 0.130、LIO↔IMU 地板 0.126、A 类地板 0.003），因为 19e 的乘积是 `R̂ᵗ·R_ref`（`R̂` 已被求解器 SVD、`R_ref` 由四元数构造），trace 不越 3；本节的 0.0283° 之所以被污染，是 19i 自己的 `ang_pair` 把 `b2c_b(L)=R_CL·L·R_CLᵗ` 直接喂进 trace（`R_CL` 正交精度仅 1e-6）。⇒ **§36.2/§36.5 无需重取**，两处 0.1° 级差异属窗口不同（k=100..600 vs k=500..849）。

### 36.10 判决门状态

| 项 | 状态 |
|---|---|
| P0-A 时间戳/插值核验 | **完成**（Δt_camera=110 ms；插值自洽） |
| P0-B 纯平滑/hf 口径 | **完成**（结论：原口径失效，改目标量） |
| P0-C 视觉旋转可观测性 | **通过**（随机 0.043° ≤0.10°） |
| P0-D 重复估计/协方差 | **通过**（σ_full 0.0053°；σ/σ̂=1.51） |
| P0-E hf_θ→地图厚度实测映射（2/5/10 m）+ λ0/λ2/钳位定标 | **完成**（36.8：旋转斜率 12.2 mm/°、平移 0.05 mm/mm、ε/θ=√2、`clip_rot≈0.06°`；**但实测判定"厚度 ≤5 mm"含 5.05 mm 位姿无关地板，不可作验收门**；2 m 档因本 bag 无近场点而**无法回答**） |
| P0-F 已知旋转扰动注入→恢复力实验 | **通过**（36.9：门 1/2/3/4/6 True，门 5b True；区间误差 0.1387°→**0.051°**、达天花板 1.10 倍、L4 真实正弦保留 89.4%、纯平滑四档全部劣于不平滑）。**唯一 False 的是门 5，判据本身不适定**（见下行） |
| 门 5 拍板（绝对残余 ≤0.05°） | **不作为硬门**。表 E 的"干净基线"`R_base` 自带 0.0557° 不可约 vs IMU 区间误差，当 `ε(基线误差) ≈ δ(注入)` 时 res 无法区分"修掉 δ"与"继续往图像拉"⇒ 该判据在此场景**不可辨**，不是被证伪。恢复力判据改用**表 E2/门 5b**（基线=纯图像链，res 只含 δ）：三档 res 0.0138~0.0206° 均**已到或优于**共模不可观下限 ⇒ 恢复力成立。门 5 数值如实保留 False 作记录。 |
| E0（M0 前置否决门，B2 任务书） | **通过**（36.11：LIO(0.1 s) 区间旋转误差 **0.1493~0.1619°**，是图像链 0.0489° 的 **3.0~3.3×**；e ∝ Δt^0.211 而非 √Δt ⇒ 白噪声型「余量归零」假设被实测否掉；c=0.1381° 与 P0-F 基线 0.1387° 自洽） |
| **Phase 1** | **开（带前置条件）**。Phase 0 判决：**图像通道信息量足够、修正器可达区间级天花板、不伤真实运动、纯平滑不可替代**。前置条件（必须先做，否则 Phase 1 的成果无法验收）：① 验收指标按 36.8 改为**去曲率 `hf_θ` + 平面芯区多峰间距 `sep_cm`**，`≤5 mm` 法向厚度降为守门；② 因子噪声口径按 36.9（七）取 **W3/W2、禁用 W1**；③ ~~按 `so3_proj` 口径重审 §36.2 中 <0.1° 的引用值~~ **已闭合**：`_p0_visual.angle_of()/log_so3()` 已补 SVD 极分解并重跑 `19e`，§36.2/§36.5 数字逐字不变（见 36.9 已知可疑口径⑥）⇒ 无需重取。 |

**产物**：脚本 `experiments/octvox_ablation/18_timealign_probe.py`（A/B 有效、C/D 标注不得引用）、`19a`~`19i`（`19a`/`19d` 头部作废说明；`19e_rot_channel_quality.py`、`19f_repeat_precision.py`、`19g_tc_placebo.py` 为 P0-C/D 判决脚本，`19h_theta_to_thickness.py` 为 P0-E 三臂注入脚本（argv `MODE` 默认 `RTC`），`19i_perturb_recovery.py` 为 P0-F 注入→恢复力脚本，argv `350 500`、`--serial` 可校验并行一致性，测量缓存 `/tmp/p0f_meas_<Wn>_<K0>.npz`），依赖 `_p0_bag.py`/`_p0_visual.py`/`_p0_rot.py`（`estimate_rotation_d` 为深度辅助 6-DOF LM，`cam_rel_from_lio` 已订正为 `R1[0]·R_CLᵗ`）、厚度口径 `_probe_layering.py`。运行：`python3 19e_rot_channel_quality.py 500 100`（123 s）、`python3 19f_repeat_precision.py`（129 s）、`python3 19g_tc_placebo.py 150 100`（54 s）、`python3 19h_theta_to_thickness.py`（114 s，需 `source /opt/ros/humble/setup.bash` 以读 rosbag2）。全部临时探针脚本置于 `/tmp`，本轮结束清空；实验目录内无临时文件。

### 36.11 E0：LIO 区间旋转误差的 Δt 标度（前置否决门，2026-11-16）

**前提纠错（本轮最重要的留痕）**：36.2 初稿把数据描述写成「相机 1 Hz」，我据此（错误地）把「P0-F 只在 1 Hz 上验证过、迁到 10 Hz 后余量可能归零」列为最高风险（R1）并挂了前置否决门。复核后确认该行是**误记**：`19e_rot_channel_quality.py:124` 用 `read_lidar_stamps(stride=1)`、`:142` 配 `(k, k+1)`，视觉区间就是 LiDAR 帧间隔（0.1 s）；P0-A 实测「图像戳 = LiDAR 头戳、1:1、std=0」⇒ **相机是 ≈10 Hz，P0-F 本来就在 10 Hz 上跑**。36.2 该行已订正，本条留痕防复犯。

**为什么仍要做**：即便相机是 10 Hz，仍存在一个真正的否决性问题——若 LIO 的区间旋转误差是**白噪声型**（∝√Δt），则 0.1 s 处只剩 0.1493/√10 ≈ 0.047°，与图像链的 0.0489° 相等，修正器无余量可用。这一条必须实测，不能推断。

**数据与口径**：Retail_Street 的 LIO 轨迹 `/home/ywj/slam_data/retail_abl/base.txt`（10 Hz TUM，1317 行）+ `/livox/imu`（203 Hz）。区间误差 = `angle_of(R_lio(t+Δt)ᵗ · R_lo(t+Δt))`，其中 `R_lo` 为以 LIO 姿态为起点、仅用 IMU 陀螺积分的独立参照（**右乘** `R(k+1)=R(k)·Exp(ω̄Δt)`；左乘会污染参照 0.454°，见 36.1）。取角一律先 `rotm()` SVD 投影回 SO(3)（见 36.9 已知可疑口径⑥的教训）。

**实测结果**（n=200 区间，脚本 `/tmp/e0_dt_scaling.py`，日志 `/tmp/e0_dt_scaling.log`，未改动仓库）

| Δt (s) | 0.1 | 0.2 | 0.3 | 0.5 | 1.0 |
|---|---|---|---|---|---|
| LIO 区间误差 med | **0.1619°** | 0.1501° | 0.1796° | 0.2102° | 0.2484° |
| P90 | 0.2841° | 0.2547° | 0.3185° | 0.3435° | 0.4558° |
| √Δt 白噪声预言（锚 0.1 s） | 0.1619° | 0.2294° | 0.2804° | 0.3619° | 0.5119° |
| 实测 / 预言 | 1.00 | 0.65 | 0.64 | 0.58 | 0.49 |

- **标度极弱**：拟合 `e ∝ Δt^0.211`（白噪声应 0.5、常值偏置应 1.0）；e(1 s)/e(0.1 s) 仅 **1.53×**。
- **物理解释**：分解 `e² = c² + w²Δt` ⇒ **c = 0.1381°**（与 Δt 无关的逐帧抖振）+ **w = 0.2088°·s^−½**，拟合残差 rms 0.0092°（5%）。相关项占方差：**0.1 s 81% / 0.5 s 47% / 1.0 s 30%**。
- **自洽校验**：c = 0.1381° 与 36.9 的基线区间误差 **0.1387°** 逐位吻合 ⇒ 修正器要消除的正是这个"与 Δt 无关的逐帧抖振项"，机制清楚，不是巧合拟合。
- 稳健性：换段 k=100/500/900 趋势一致；τ=0（直接取发布戳，不做半帧对齐）时 0.1225~0.2402，形状不变。

**判决**：LIO(0.1 s) = **0.1493~0.1619°**（用与视觉同一批 325 个区间时为 0.1493°），是图像链 **0.0489°** 的 **3.0~3.3×** ⇒ **余量充足，E0 门通过，M2 可开**；「余量归零」的负向假设实测否掉。

**E0 附带产出（比门本身更有价值，直接改 Phase 1 设计）**

1. **视觉误差是短记忆相关的**：图像链 0.1 s 区间误差 lag-1 自相关 **+0.461**，lag-3 已≈0（+0.043）；1 s 链的非重叠 lag-1 为 +0.445 ⇒ 因子图把区间残差当 i.i.d. 会**过自信**。
2. **定量校准（初稿结论已被 36.12 探针推翻）**：初稿称"膨胀因子 1.65× ≈ σ_eff 0.0322°/实测 0.0489° = 1.52× ⇒ W3 过自信 1.5×，须膨胀 σ 后重搜 λ0"。该比较口径不对等（主轴均值 σ vs 向量模 rms）；探针实测边际/口径比仅 **1.08×**，AR(1) 逐轴强各向异性 ρ=[0.024, 0.802, 0.488]，且 λ0 灵敏度极平（σ×1.52 不动 λ0 ⇒ 结果变化 ≤0.001°）⇒ **R10 不阻塞 M2、不需重搜 λ0**。全部数字见 36.12。
3. **LIO 发布戳对帧内插值极不友好**：τ=0.055 s（半帧）处 LIO 区间误差 **0.343°**，比 τ=0 的 0.14° 差 **2.4×** ⇒ `T_corrected(t)` 查询必须**优先精确戳、次选 SO(3) slerp，禁止姿态分量线性 lerp**。

**产物**：`/tmp/e0_dt_scaling.py`、`/tmp/e0_dt_scaling.log`、`/tmp/e0_vis_cache.npz`（均为临时探针，不入库）。

### 36.12 R10 数值验证：AR(1) 过自信对 M2 标定的影响探针（2026-11-16）

**动机**：36.11 附带产出②初稿断言"W3 σ 膨胀 1.52× ≈ AR(1) 膨胀 1.65× ⇒ M2 须改 AR(1) 噪声模型或膨胀 σ 后重搜 λ0"。本节数值实测裁决该结论是否成立、对 M2 影响多大。
**探针**：`/tmp/r10_probe.py`（临时，日志 `/tmp/r10_probe.log`），复用 `19i` 的 `measure()` 与 `solve_cfg('slide', wk, 'L0-zero', sp, 0.50, 0.30)` 接口；因 `/tmp/p0f_meas_*.npz` 缓存已清，测量段为 **k=350..849**（重建耗时 25.8 s），base=**0.1428°**、最优改善 **0.0532°**，与头条段 k=500..849 的 0.1387°/0.051° 差 ~0.004°——**段差不影响结构性结论**，但跨段对照须留 ±0.005° 容差。

**口径核对（推翻初稿 1.52× 的依据）**：

| 量 | 实测值 |
|---|---|
| W3 等效 σ（Sig_vi 主轴均值的中位） | 0.0360° |
| \|g\| 边际 rms / √3 | 0.0389° |
| **边际/口径比** | **1.08×** |
| Sig_vi 主轴 σ | [0.0235, 0.0278, 0.0567]° |

初稿"0.0322° vs 0.0489° = 1.52×"是主轴均值 σ 与向量模 rms 的错配比较，作废。

**逐轴 AR(1) 实测**：lag-1 ρ = **[0.024, 0.802, 0.488]**（均值 0.438，平均口径膨胀因子 1.60×）⇒ 相关**不是各向同性**：一轴白、一轴 ρ≈0.80（慢变偏置特征）、一轴 ≈0.49。

**λ0 灵敏度网格**（sp ∈ [0.30, 1.30]，clip=0.30，it=4，指标 imu_dt med）：

| 口径 | sp=0.30 | sp=0.45 | sp=1.30 | 最优 |
|---|---|---|---|---|
| W3 | **0.0532°** | — | 0.0584° | 0.30 |
| W3 σ×1.52 | **0.0532°** | 0.0539° | — | 0.30 |
| W3 σ×1.60 | 0.0536° | — | — | 0.30 |

尺度不变性校验成立（σ×s 且 λ0×s ⇒ 同解）；σ×1.52 而 λ0 不动 ⇒ 结果移动 ≤0.001°。

**判决（R10 对 M2 的影响）**：
1. **R10 不阻塞 M2**——P0-F 的 λ0=0.30 / clip=0.30 可直接移植 C++，不需重标 σ、不需重搜 λ0。
2. AR(1) 结构的真实约束在 M2 之外两处：① 窗口联合一致性门（NIS）**禁止按 i.i.d. 计算**（逐轴 ρ 最高 0.80）；② ρ=0.80 轴本质是慢变偏置 ⇒ 归 **E4** 相关误差治理。
3. C++ 实现必须文档化 **σ–λ0 配对口径**（改 σ 必须同倍改 λ0）。
4. 结果的主导权衡仍是 **clip=0.30**，与 R10 无关。

**产物**：`/tmp/r10_probe.log`（证据日志；脚本已清理）。对应任务书 §7 R10 行已按本节改写。

### 36.13 M2：rotation_refiner C++ 移植与模块级复现（2026-11-16）

**交付**：`solution_b_tight_coupled/super_livo_visual_b/{include/super_livo_visual_b/estimator,src/estimator}/rotation_refiner.{hpp,cpp}`，并追加到 `CMakeLists.txt` 的 `add_library` 源列表（不新建 target）。模块只依赖 Eigen，不读写 ROS 状态，**不接入** `UnifiedEstimator`。

**接口约定的可等价性**：模块接收 **body 系**区间旋转，故不含相机外参。设 `R_CL` 为 b2c 用的 body→camera 旋转，则
`r_cam = R_CL·r_body`、`Cw_body = Cw_cam·R_CL`（由 `r_cam = log(R_CL (AᵀE) R_CLᵀ) = R_CL log(AᵀE)` 推出），
于是 `Cw_cam·r_cam = Cw_body·r_body`，整个最小二乘问题逐项等价。这一条是 Python→C++ 能逐位对齐的前提。

**验证方法**：取 `19i` 缓存段 k=350..849（N=500），噪声用 **W3**（Sig_vi 主轴 [0.0235, 0.0278, 0.0567]°，floor 0.02°）；参数 `λ0=0.30°, λ2=0.50°`、`clip=0.30°`、`slew=2×clip`、`iters=4`、`irls_rounds=1`（P0-F 滑窗档不做 Huber 重加权）。Python 侧 `run_slide`，C++ 侧逐帧 `addFrame`，同为因果、同段、同参数。

| 指标 | 基线（u=0） | Python | C++ |
|---|---|---|---|
| `imu_dt`（区间级、逐轴去趋势，任务书口径） | **0.1428°** | **0.0532°** | **0.0532°** |
| 区间旋转 vs IMU（未去趋势，旁证） | 0.1430° | 0.0492° | 0.0492° |
| `\|u\|` 峰值 | — | 0.3000° | 0.3000° |

**逐帧一致性**：`max|u_py − u_cpp| = 2.325e-06°`、`rms = 4.362e-07°`（LM 收敛容差量级）。触发 `clip` 48 帧、`slew` 0 帧，两侧计数一致。

**构建**：`colcon build --packages-select super_livo_visual_b` 通过（52.2 s）。

**结论与边界**：
1. **模块级复现通过**（0.1428°→0.0532°，与 Python 完全一致），M2 可进入下一阶段（E2 生产节点保真）。
2. 复现段为 k=350..849，与 P0-F 头条段 k=500..849（0.1387°→0.051°）差 ~0.004°，属段差而非算法差异。
3. 保留 P0-F 的**解后硬投影**（U8/S1/S2）以对齐复现；显式信任域约束属 M5（R8 修正量阶跃），此处未做。
4. 尚未接入节点，也**未**添加单元测试/自检入口：Jacobian 正确性通过"与 Python 解逐帧一致"间接验证（若解析 Jacobian 有误，4 次迭代的迭代路径与解都会偏离）。
5. 复现产物：`/tmp/m2_export.py`（导出 + Python 参考解）、`/tmp/m2_driver.cpp`（C++ 驱动）、`/tmp/m2_compare.py`（对照）、`/tmp/m2_rr_{input,u_py,u_cpp}.bin`（均为临时，不入库）。

### 36.14 E2：生产节点保真（10 Hz 真实数据流，`visual_b_node` 落盘 → C++ RotationRefiner，2026-11-16）

**要回答的问题**：§36.13 的 0.1428°→0.0532° 是"同一份 Python 测量缓存换 C++ 求解器"的模块级复现。E2 再往前一步：**从真实 10 Hz 生产节点的图像流里取特征与位姿**，用与 P0-F 完全同口径的 `estimate_rotation_d` 重算 `r_vis`，顺序喂 C++ `RotationRefiner`，看离线结论能否复现。要否掉的是 R7"离线脚本 ≠ 生产节点"。

**数据链（不改核心滤波）**：`visual_b_node` 新增可开关落盘（`e2.dump_enable` / `e2.dump_dir`），在 `frontend_->process()` 之后写 `e2_pose.csv`（`stamp,t_pose,Rwl(9),pwl(3),n_obs`）与 `e2_obs.csv`（`stamp,n,id,u,v,...`）。落盘与滤波完全隔离，`/lio/odom` 位姿照旧由 Super-LIO 提供（`color.use_own_odom=false`，E2-A 组）。离线侧按 feature_id 求交、`tc` 由节点位姿算、`RT.estimate_rotation_d` 重估、body 系换算后喂 `/tmp/m2_driver`。

**三个被实测钉死的坑（按发现顺序，均会单独改变结论）**：

1. **`std::setprecision` 是有效数字位数，不是小数位**。ROS 时间在 1.7e9 量级，`setprecision(9)` 把小数部分全部丢掉，落盘 `stamp` 被量化到整秒（实测写出 `946685438`，真值 `946685438.799955`）。离线按该戳插值 IMU 会错最多 0.5 s ⇒ 同一条 LIO 轨迹的 `imu_dt` 被报成 **1.4393°**（P0-F 同口径 0.1428°），一度被误判为"节点 odom 帧间抖动"。改 `setprecision(17)` 后降到 **0.1340°**，"节点位姿抖动"的怀疑当场推翻。
2. **`Rwc = Rwl @ Rcl` 少一次转置**。`V.R_CL` 是 b→c（`p_c = Rcl·p_b`），故 `R_wc = R_wl · R_CLᵀ`（与 `_p0_rot.cam_rel_from_lio` 同口径）。写反后 `tc` 落在差 120° 的坐标系里，深度辅助 LM 的基线项随之错，视觉-IMU 残差 **1.54°→1.95°**（改对后仍不可用，因为还有第 3 条）。
3. **生产前端 `PatchTracker` 的单层 5×5 SSD 模板匹配在本数据集上不可用**（细节见下）。

**坑 3 的两级证据**：

| radius | 现象 |
|---|---|
| 4（默认） | 对应点位移**精确饱和在 4√2 = 5.66 px**，58.7% 的特征顶到边界（真实光流中位 6~25 px） |
| 20 | 饱和消失（位移 med 13.89 px），但 82% 仍 >5.66 px 且部分顶到 28.28 px；`r_vis` 区间旋转 med 1.777° |

**隔离对照**（同图像对、同 `tc`（离线 `base.txt` 在 `te=lidar+0.11` 插值）、同 IMU 参照，只换对应点来源）：

| bag 帧对 | 节点 PatchTracker | P0-F `track_pair`（双向 LK） | 真实区间旋转 |
|---|---|---|---|
| 100 | 0.2041° | 0.0168° | 0.9459° |
| 400 | 1.3456° | 0.0141° | 1.0184° |
| 600 | 5.3107° | 0.0569° | 2.2708° |
| 1000 | 9.0025° | 0.0318° | 2.4505° |

同口径下 LK 回到 P0-F 量级（0.014~0.057°），节点前端在真实区间旋转 1~2.5° 的帧上偏差 1.3~9.0°。⇒ **E2 的唯一阻塞项是前端跟踪器，不是时间同步、不是位姿、不是 `tc`**。

> 口径订正：本条目第一版曾用 `initUndistortRectifyMap(K_FULL, D, None, K_HALF, …)` 做去畸变，等于按半幅归一化坐标去畸变 + 光流减半，LK 因此也只报 0.66~1.01°。正确写法与节点 `rectifyBgr` 一致：输入输出都用 `K_HALF`（= 节点 `cam_fx/fy/cx/cy` = 646.78/646.66/313.46/261.40）。修正后 LK 才回到 0.014~0.057°。

**修复**：`PatchTracker` 改为**金字塔 LK**（`calcOpticalFlowPyrLK`，窗 = `2·search_radius+1`、`maxLevel = pyramid_levels-1`、亚像素迭代终止 30 次/0.01 px），保留原有"以 `1-TM_SQDIFF_NORMED` 打分 + `lssd_threshold` 门限"与"角点补充 + `min_distance` 去重"，`reset()` 释放 `prev_gray_`。`search_radius` 的语义随之从"SSD 搜索半径"变为"LK 窗口半边长"（默认 4 ⇒ 9×9）。

**E2 主结果**（`Retail_Street`，`TAGS=lio`，`BAG_RATE=1.0`，生产默认参数 `patch_size=5 / pyramid_levels=4 / search_radius=4 / max_features=200`）：

| 指标 | 修复前（`radius=20`，SSD） | 修复后（LK，默认 4） |
|---|---|---|
| 帧数 / 时长 / 频率 | 1309 / 134.0 s / 9.76 Hz | 1313 / 134.4 s / 9.76 Hz |
| 平均特征数 | 195.1 | 171.3 |
| 可用视觉因子 | 1136/1308 = 86.9% | 1135/1312 = 86.5% |
| 失败原因（全部） | `too_few/lb` 172 | `too_few/lb` 177 |
| `\|tc\|` 中位 / 内点中位 | 0.0551 m / 60 | 0.0552 m / **140** |
| 视觉区间旋转中位 | 1.777° | 1.480° |
| **`imu_dt`（区间级、去趋势）** | **0.1315° → 0.1521°（比 1.157，未过门）** | **0.1330° → 0.0540°（比 0.406，过门）** |
| `imu_med` / `imu_p90` | — | 0.1351→0.0569 / 0.2772→0.1293 |
| `e_int` / `e_int90` | — | 0.5107→0.4894 / 0.9763→0.9475 |
| `\|u\|` 峰值 | 0.300°（clip 生效） | 0.300°（clip 生效） |
| 真实转向：幅值保留 / 逐轴相关 | 0.994 / 0.995,0.993,0.996 | **0.994 / 0.991,0.991,0.999**（1135 帧运动段） |

**交叉校验（LK 版）**：

| 校验项 | 结果 |
|---|---|
| 视觉因子 vs IMU（全段 134 s，n=1135） | med **0.0572°** / p90 0.1277° |
| P0-F 同段（k=350..849，n=437） | med 0.0477° / p90 0.0895° |
| 节点 `r_vis` vs P0-F `r_vis`（逐因子对齐，n=666） | med **0.0439°** / p90 2.4022° |
| 对应点位移：节点 vs 同起点 LK（帧对 300/600/900/1200） | med **0.037 / 0.023 / 0.053 / 0.029 px**（修复前 5.92 / 9.02 / 13.58 / 16.13 px） |
| 时基扫描：视觉旋转 vs IMU 参照偏移 `d` | **V 形最小值精确在 d = 0.110 s（0.0572°）**；d=0.100→0.1267°，d=0.120→0.1670° |

时基扫描另外直测出两条事实：`/livox/lidar` 的 header 戳与 `/left_camera/image` 的 header 戳**逐帧完全相等**（差 0.000000 s），lidar 帧跨度 0.0998 s 恰为 `[img_i, img_{i+1}]`；落盘 `stamp` 与图像 header 逐帧差 0.000000000 s。⇒ 图像内容真实时刻 = **图像 header + 0.110 s**，与 §36.1 的 `DT0=0.110` 一致。

**判决门（任务书 §3 E2）**：

| 门 | 要求 | 实测 | 结论 |
|---|---|---|---|
| 0.1 s 区间旋转误差 vs 同区间 LIO | ≤ 0.5 × | **0.0540 / 0.1330 = 0.406** | **通过** |
| 真实转向保留率 | ≥ 80% | **99.4%**（逐轴相关 0.991/0.991/0.999） | **通过** |

**结论与边界**：
1. **E2 生产节点保真通过**：0.1330°→0.0540°（比 0.406），真实转向保留 99.4%。与 §36.13 模块级（段 k=350..849，0.1428°→0.0532°）同量级，差异属段差（本节为全段 134 s）。
2. 视觉因子质量在生产链路上达到 0.0572°，P0-F 离线同段 0.0477°（差 1.20×），对应点位移与 LK 差 0.02~0.05 px ⇒ **离线结论在生产前端可复现**，R7 风险大幅下调（剩余部分是场景覆盖与运动模糊，本轮未覆盖）。
3. **失败因子 177/1312（13.5%）全部是 `too_few/lb`（共同特征 <40 或 `|tc|` < 0.008 m）**，没有一例是求解器 `usable=false`。这是"纯旋转段不可观"的结构性退化，属预期行为，不是缺陷。
4. **未做**：E2-B/C/D 三组的横向对照（本轮只跑 E2-A 输入 + P0-F 口径测量）；场景覆盖（走廊/楼梯/运动模糊）、视觉丢帧的自然退化、`P0-F` 与 B 原有平移约束的因子替换（E2-D 的防重复计数）——均留待 M3 后续条目。
5. 旁证（不在本题范围）：节点染色的 `img_time_offset` 配 0.100，实测图像内容时刻是 header+0.110，存在 **10 ms** 固定偏差；对 E2（自带 `te` 口径）无影响，但会影响染色/LV 约束的配准，需在 M3 单独处理。
6. 产物：`/home/ywj/slam_data/e2_lk_20261116/dump/`（LK 版，**本次结论依据**）、`e2_20261116/dump/`（`radius=20` SSD 版）、`e2_20261116/dump_r4/`（`radius=4` **饱和证据**，保留）。离线脚本 `/tmp/e2_{export,check,clock,imgtime2,track2}.py`、二进制 `/tmp/e2_rr_{input,u_cpp}_lk.bin`、结果 `/tmp/e2_result_lk.npz`（均临时，不入库）。

---

### 36.15 E3 前置可探测性测试：向 LIO 旋转注入已知扰动，重建五臂地图（2026-11-16）

**要回答的问题**：E3 计划用"地图分层"判断 M5 在线旋转修正值不值得做。第一个问题不是修正效果，而是**判据本身的灵敏度**——0.05°/0.1°/0.2° 的旋转误差，能不能在重建地图上看出来、量出来？若量不出，E3 的分层判据就是空转。

**数据链（全程同一条运行，不跨流复用位姿）**：
- 原始点云：`/home/ywj/slam_data/Retail_Street/Retail_Street.db3` 的 `/livox/lidar`（CustomMsg，1355 帧），按 header 戳与 `e2_lk_20261116/dump/e2_pose.csv` 精确配对 **1313 帧 / 134.4 s**（0 ms 差）。
- 位姿：E2 落盘的 `T_wl`（`stamp,t_pose,Rwl,pwl,n_obs`）。已确认 `t_pose = header + img_time_offset(0.100 s)`，即**本帧扫描结束时刻**；Super-LIO 侧 `undistortCloud(..., T_end, ...)` 也把 body 云去畸变到**帧末**位姿，两者同口径。
- 逐帧流程：帧 k 的点按 `offset_time∈[0, dur]` 在 `[k−1, k]` 两个帧末位姿间 slerp/lerp 得 `p_w(τ)=R(τ)p_l+t(τ)`，再存为 body 系 `R_kᵀ(p_w−t_k)`；各臂只改 `R_k`。
- 注入口径与 §36.9 一致：各向同性随机方向、逐帧独立、固定模长、**左乘世界系** `R' = Exp(δ)@R`，种子 `SEED_INJ + int(θ×1000)`。
- 采样：每帧每 6 点取 1（stride=6），0.3~80 m 范围滤波；未体素化（`median_voxel_size=0.0`）。

> 口径修正留证：第一版把 `T_wl` 当作帧首、用 `[k, k+1]` 插值，整段地图被多推了一个帧间隔。改回帧末 `[k−1, k]` 后 kNN 厚度 P50 由 14.15 → **13.61 mm**，五臂结论不翻转但基线更干净。

**五臂产物**（各 4,836,027 点 / 77.4 MB，`/home/ywj/slam_data/e2_lk_20261116/pcd_arms/`）：`arm0_lio.pcd`（灰 140）、`arm1_fix_e2.pcd`（绿，`Exp(u)@R`）、`arm2_inj050.pcd`、`arm3_inj100.pcd`、`arm4_inj200.pcd`（蓝/橙/红）。

**实测（同一批点、同一套去畸变，只换帧末旋转）**：

| 臂 | `hf_θ` P50 / P90 [mdeg] | kNN 厚度 P50 / P90 [mm] | 最一致平面贴片 `P90\|d−med\|` [mm] |
|---|---|---|---|
| arm0 LIO 原始 | 510.7 / 976.3 | 13.61 / 50.03 | 16.71 |
| arm1 E2 修正 | 489.4 / 947.6 | 13.76 / 50.29 | 16.45 |
| arm2 注入 0.05° | 513.4 / 990.3 | 13.64 / 50.00 | 16.76 |
| arm3 注入 0.10° | 517.8 / 997.0 | 13.73 / 50.06 | 17.19 |
| arm4 注入 0.20° | 545.6 / 1034.9 | 14.15 / 50.45 | 18.62 |

**结论**：
1. **0.1° 不可分辨**：kNN 厚度只 +0.12 mm（本底 13.6 mm，0.9%），`hf_θ` +7.1 mdeg（本底 511，1.4%），贴片 P90 +0.48 mm。三个指标全部落在同臂重采样/贴片选择带来的波动量级内。
2. **0.2° 刚可分辨**：厚度 +0.54 mm（4.0%）、`hf_θ` +34.9 mdeg（6.8%）、贴片 P90 +1.91 mm（11.4%），且三指标方向一致（单调递增）。视觉上可在贴片法向直方图里看到主峰整体右移 4~5 mm 并展宽（`preview_patch_hist.png`）。
3. **0.05° 完全不可见**（厚度 +0.03 mm、`hf_θ` +2.7 mdeg、贴片 P90 +0.05 mm）。
4. **正控（另一条数据链，245 帧 + `replay3` 位姿）**：θ=2.0° 时 kNN 厚度 14.08→15.94 mm、逐帧墙面中位偏移 std 56.8→141 mm ⇒ 注入—建图—测量管道本身有效，上表"未见响应"是灵敏度不足，不是管道断路。
5. **E2 修正本身**：`hf_θ` 510.7→489.4 mdeg（−4.2%），kNN 厚度 +0.15 mm、贴片 P90 −0.26 mm——方向与 §36.14 的 0.133°→0.054° 一致，但在地图指标上远小于其量级，说明**地图指标不是旋转精度的敏感放大镜**。
6. 因此 E3 若照原计划只做 0.05/0.1/0.2° 三臂，**主判据 `sep_cm` 与厚度都判不出差异**；要么把注入量级提到 ≥0.5°（但那就不是目标工况），要么改判据（例如同场景多圈重访的逐帧相对偏移、或直接以视觉/IMU 区间残差为判据）。本轮不改结论方向，只关上"地图分层能分辨 0.1°"这个假设。

**重建保真校验（解释"重建图与节点图差 ~4 cm"）**：
- `_probe_layering.load` 抽样 300k 后与节点 OctVox 图 `b_lio/super_livo_visual_b_colored.pcd` 比：NN 距离 P10 13.5 / P50 41.1 / P90 246.6 mm（前 300 帧）。
- 两侧再各按 0.10 m 体素下采样（131,391 vs 73,084 点）后做点到点 ICP：**平移 [4.1, 4.7, 5.6] mm、转角 0.064°、fitness 0.810、rmse 87.5 mm**，对齐前后 P50 均为 79.3 mm ⇒ **没有刚性错位**，残余距离是弥散的。
- 反向距离（节点图 → 重建图）P10 14.6 / P50 30.8 / P90 60.4 mm，量级相同。结合 OctVox 每 0.25 m 子体素只留 1 个代表点（`merge_dist=0.1 m`）、且两侧都做过 300k 抽样，可判定 **~4 cm 是代表点稀疏 + 抽样造成的统计距离，不是位姿/时基错误**。重建链路可信。

**产物**：`pcd_arms/{arm0..arm4}.pcd`、`pcd_arms/overlay_patch.pcd`（五臂同一贴片叠加，按臂着色，298,733 点 / 4.8 MB）、`pcd_arms/preview_top.png`、`preview_patch.png`、`preview_patch_hist.png`；脚本 `/tmp/build_pcd_arms.py`、`/tmp/preview_arms.py`、`/tmp/icp_check.py`、`/tmp/probe_arms_control.py`、`/tmp/check_rebuild.py`（均临时，不入库）。

### 36.16 NTU VIRAL 序列筛查：eee_03 不是多圈，多圈候选是 eee_01（2026-11-16）

**动机**：§36.15 结论 6 给出的 E3 备选之一是"换多圈重访场景，用同一墙面被不同时刻观测的错位放大旋转误差"。先确认手上的 NTU 数据能否支撑。

**本地数据实况**：`/home/ywj/slam_data/ntu_viral` 只有 **`eee_03` 的原始包**（`bags2/eee_03/eee_03.db3` 4.24 GB、`raw/eee_03/eee_03.zip`），`gt/` 下 18 个序列的 `ground_truth.csv` 齐全；其余 17 个序列的原始数据未下载（官方 ZIP 3.7~8.7 GB，`docs/README.md` 列有 Hugging Face 直链）。GT 全部为 Leica 全站仪棱镜位置。

**方法与判据**（只读 GT 的 position，脚本 `/tmp/ntu_loops.py`、`/tmp/ntu_turns.py`、`/tmp/ntu_revisit_detail.py`、`/tmp/ntu_plot_paths.py`）：① 回环对＝时间间隔 >30 s 且水平距离 <1 m；② 绕质心方位角净变化/360°＝净圈数（原路返回→0，绕闭环一圈→±1）；③ 距起点 <2 m 的时间簇数＝回起点趟数；④ 把 ① 的帧按时间连续切成段，看是否真的**持续**重走同一段路。

**实测**（`gt/*/ground_truth.csv`）：

| 序列 | 时长 | 里程 | 净圈数 | 总转角圈 | 回起点 | 回环帧占比 | 最远重访时间差 |
|---|---:|---:|---:|---:|---:|---:|---:|
| **eee_03** | 150.8 s | 122.5 m | **−0.04** | 2.52 | **2** | 19.4% | 128 s（仅起点区 10 s） |
| eee_01 | 336.4 s | 201.6 m | **1.66** | 5.17 | **6** | **78.6%** | 232 s（19 段≥1 s，最长 29.6 s / 跨 24.4 m） |
| eee_02 | 281.0 s | 158.8 m | −3.03 | 5.01 | 5 | 58.5% | — |
| nya_03 | 393.4 s | 306.9 m | −0.41 | 16.25 | 6 | 96.5% | — |
| tnp_01 | 577.3 s | 216.4 m | 1.41 | 7.08 | 5 | 92.6% | — |
| spms_01 | 446.3 s | 480.2 m | 1.44 | 13.69 | 5 | 59.2% | — |

**结论**：
1. **`eee_03` 不是多圈**：净圈数 −0.04（≈0）说明是"出去、绕一个闭环、回到起点附近"的**单趟**；轨迹图上同一走廊只走一次。19.4% 的"回环帧"全部来自起点/终点附近 10 s 的重叠（时间差中位 126 s），以及 4 段 2 s 左右的短重访；远离帧最近距离 P50 = **2.45 m**（eee_01 仅 0.46 m）。用它做"多圈错位放大"证据量太少。
2. **`eee_01` 才是真多圈**：6 次回起点、净 1.66 圈、78.6% 帧存在 >30 s 的 1 m 内伙伴、19 段≥1 s 的持续重访（时间差 53~232 s，最长一段连续 29.6 s、空间跨 24.4 m）⇒ 同一面墙被相差上百秒的两趟观测覆盖，正是 E3 想要的放大条件。**代价：原始包未下载（4.53 GB）**。
3. `nya_03` / `tnp_01` 重访比例更高，但是无人机在 13×8 m / 22×9 m 的狭小空间里高频往返（`nya_03` 307 m 里程挤在 104 m² 内），墙面重叠是"原地盘旋"型，yaw 漂移的时间基线短，不如 `eee_01` 的"整栋楼绕圈"适合放大。
4. **对 E3 的直接含义**：方案 2 在现有数据上不可做；要么下 `eee_01`（4.53 GB，已有 GT 与 `tools/odom_vs_gt.py` 全套协议），要么退回 Retail_Street 改用 §36.15 结论 6 的另两个方向（换判据 / 提到 ≥0.5°）。

**产物**：`/mnt/d/traetmp/ntu_gt_all_paths.png`（18 序列轨迹总览）、`ntu_gt_path_{eee_01,eee_03,nya_03,tnp_01}.png`（时间着色放大图）；表格数据来自上述 4 个临时脚本。

---

### 36.17 E3 多圈终判：eee_01 上 0.1° 旋转误差在地图层面仍不可分辨（2026-11-16）

**动机**：§36.15 在 Retail_Street 单趟场景证得 0.1° 不可分辨、0.2° 刚可分辨，结论 6 给出备选方案 2「换多圈重访场景，用同一面墙被相差上百秒的两趟观测放大错位」；§36.16 选出 `eee_01`（净 1.66 圈、78.6% 帧有 >30 s 重访伙伴）。本条目在 `eee_01` 下载完成后把该方案跑通并判决。

**数据与前置**：`bags2/eee_01/eee_01.db3` 9.25 GB / 257,891 条 / 5 话题；Ouster OS1-16，16 线 × 1024 方位 = 16,384 点/帧、`point_step=48`；GT 为全站仪棱镜轨迹（6,616 点，1609059060.11~1609059396.47，336.4 s，**姿态列恒为单位四元数 ⇒ 只有位置真值**）。

**DDS 前置（必须写入跑批口径）**：本机 FastRTPS 跨进程发现完全失效（同进程 pub 正常、其它进程 `topic list/echo` 看不到），改用 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；而 Super-LIO 默认 `best_effort` 订阅在 Cyclone 下收不到 LiDAR（`lidar_buf=0`）⇒ 必须同时 `-p lio.ros.lidar_qos_reliable:=true`。二者补齐后 90 s 冒烟：`frame_num=867`、odom 421 条、span 86.5 s、最大间隔 0.36 s。

**基线跑批**（`/tmp/run_ntu_lean.sh`：只播 `/os1_cloud_node1/points` + `/imu/imu`，只起 `super_lio_node`，只录 `/lio/odom`；`RATE=0.5`，避开 §run_ntu.sh 注释里的 140 ms/帧积压）：`/lio/odom` **n=3754、span 398.0 s、dt P50 0.100 s、最大 0.177 s、>0.5 s 断流 0 次**。

**链路验证**（`tools/odom_vs_gt.py`，体→棱镜外参 + combteeth + Umeyama）：**ATE(RMSE)=0.098 m**、中位 0.087、P95 0.155、最大 0.228；五分段 0.099 / 0.097 / 0.113 / 0.082 / 0.096。数据链可用，后段无恶化。

**方法**（脚本 `/tmp/ntu_e3_multiloop.py`）：
1. **重访对**：用 GT 位置选 `dt>30 s 且距离<0.8 m` 的帧对，按时间差降序去重 ⇒ **895 对**，时间差 P10/P50/P90 = 30 / 181 / 344 s（其中 326 对因无足够共视 patch 跳过，实际 **569 对**）。
2. **逐帧点云**：rosbags 反序列化 PointCloud2，按 fields 偏移取 x/y/z（float32），range 过滤 [0.8, 25] m；按逐点时间在 k−1→k 位姿间插值去畸变，`p_w = R_t p + t`，存 body 系 `p_b = R_kᵀ(p_w − t_k)`（与 §36.15 帧末口径一致）。共 **3,979 帧**缓存。
3. **配对 patch**：1 m voxel 分组（以 arm0 世界坐标建组，各臂共用同一分组），要求 A 趟 ≥40 点、B 趟 ≥12 点 ⇒ 15,766 patch，可用 **15,525**（\|m\|<0.25 m）。
4. **逐 patch 指标**：A 趟 PCA 拟合平面（λ0 ≤ 0.02·λ2），`m = median(B 点沿 A 法向的有符号偏移)`（\|d\|<0.5 m 带内、≥10 点）；同时用 B 趟点自身拟合平面，得两趟**法向夹角**。
5. **注入**：`rand*`＝逐帧独立随机轴、同模长、左乘 R（绕本帧传感器位置转动，模拟逐帧旋转噪声）；`epoch*`＝t ≥ 中位帧（锚点＝帧 1989，t=1609059212.8 s，p=(1.53,−14.48,3.46)）的帧统一左乘 yaw 轴 `Exp(δ)`，并**绕锚点刚性旋转整个后半段轨迹** `p₂ = Exp(δ)(p−a)+a`（模拟后程累积 yaw 偏差；绕本帧原地转动对跨趟相对几何是零效应，故必须带平移）。各 `rand` 臂共用同一组方向、各 `epoch` 臂共用同一 yaw 轴，只改模长 ⇒ 剂量-响应可比。
6. **判据量** `d = m_臂 − m_arm0`：同一批点、只改位姿 ⇒ LIO 自身漂移/采样本底**完全相消**，d 只由注入引起。

**内部对照**：同 epoch 对（A、B 都在 t_split 之后）在各 `epoch` 臂上 `d ≡ 0.00`（`same_epoch_rms_mm=0.000`）⇒ 实现是纯相对位姿效应、无实现噪声；剂量-响应严格线性（0.05/0.10/0.20° → RMS 0.67/1.34/2.68 mm，每档 2.00×）。

**实测**（mm；n=15,525 patch，夹角 n=15,074）：

| 臂 | \|m\|中位 | RMS d | 中位\|d\| | P90\|d\| | 均值 d | 夹角 RMS(mdeg) | 夹角中位(mdeg) | r 1–5 m RMS | r 5–10 m RMS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| arm0 | 23.25 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| rand 0.05° | 23.09 | 2.62 | 1.18 | 3.04 | +0.03 | 39.8 | 28.9 | 2.62 | 3.40 |
| rand 0.10° | 22.99 | 4.37 | 2.36 | 6.07 | +0.03 | 79.4 | 57.6 | 4.35 | 6.83 |
| rand 0.20° | 22.18 | 8.02 | 4.72 | 12.13 | +0.07 | 159.0 | 115.4 | 7.99 | 14.15 |
| epoch 0.05° | 23.57 | 0.67 | 0.09 | 1.14 | +0.31 | 1.41 | 0.30 | 0.65 | 2.23 |
| epoch 0.10° | 23.88 | 1.34 | 0.18 | 2.29 | +0.62 | 2.82 | 0.59 | 1.31 | 4.46 |
| epoch 0.20° | 24.53 | 2.68 | 0.36 | 4.58 | +1.25 | 5.65 | 1.18 | 2.61 | 8.91 |

（r＝patch 点到**全图质心**的平均距离，作杠杆臂代理；arm0 的 566.6 mdeg 为两趟法向夹角的天然不一致度中位。）

**结论**：
1. **本底**：arm0 的跨趟有符号偏移 \|m\| 中位 **23.3 mm** / P90 **47.1 mm** —— 这是真实地图里"同一面墙被两趟观测对不上"的实际量级（含 LIO 漂移 + 采样/代表点误差）。
2. **0.1° 逐帧随机：RMS d 4.37 mm（中位 2.36）**，只有本底中位的 1/10、P90 的 1/20；0.2° 也只有 8.02 mm，其 P90 12.1 mm 仍只有本底 P90 的 26%。
3. **0.1° 半趟系统 yaw 错位：RMS d 仅 1.34 mm（中位 0.18）**，比同角度随机注入小 3.3× —— 原因是 yaw 位移沿切向，投影到墙面法向只余约 1/4。也就是说，**多圈带来的"时间基线放大"在位移通道上被法向投影抵消掉了大半**。
4. **距离分箱确认放大机制存在、但量级不够**：epoch 0.10° 在 r=1–5 m 上 1.31 mm、5–10 m 上 4.46 mm（3.4×），rand 0.10° 4.35→6.83 mm（1.6×）；都远低于纯几何上限 r·δ（10 m × 0.1° = 17.5 mm），因为还要乘法向投影（≈0.25~0.65）与 patch 内平均。
5. **法向夹角通道更灵敏但同量级**：0.1° 随机 ⇒ 夹角变化 RMS 79.4 mdeg / 中位 57.6 mdeg；系统 yaw 0.1° ⇒ RMS 2.8 mdeg / 中位 0.59 mdeg。后者只有天然不一致度（566.6 mdeg）的 1/1000。
6. **判决：E3 判据不成立。** 0.1° 引起的地图几何效应（≤4.4 mm RMS）系统性低于真实地图自身的跨趟不一致度（23.3 mm 中位 / 47.1 mm P90），且"能测出 d"完全依赖 arm0 这个**反事实配对**——实拍地图没有"无误差的 arm0"，配对差分做不出来。即使 0.2° 也只有 8.0 mm / P90 12.1 mm，仍埋在 47 mm 的天然散布里。要把效应抬到本底之上需要 **≥0.5–1°**，比 M5 关心的 0.1° 高一个量级。这与 §36.15 单趟结论一致，且是**物理上更宽松的多圈场景**下的再次否证。
7. **对 M5 的含义**：不能拿"地图质量变好/分层消失"当 M5 的验收或立项证据；M5 的证据只能是**位姿级**——§36.11 的「LIO 0.1 s 区间旋转误差 0.1493~0.1619° 是图像链 0.0489° 的 3.0~3.3×」。若仍想用地图指标，只能走 §36.15 结论 6 的替代判据（去曲率 `hf_θ` / `sep_cm`），且必须在 ≥0.5° 量级上重新标定灵敏度。
8. **附带负结论**：原本想加一条"GT 位姿臂"（用全站仪位姿替 LIO 位姿，量真实位姿误差对跨趟对齐的影响）——实测 GT 的姿态列全为单位四元数（只有位置），无法构造 GT 位姿，该臂作废；GT 只能用于平移通道与 ATE。

**产物**：`/tmp/ntu_e3_multiloop.py`、`/tmp/ntu_e3_multiloop.json`（各臂全指标）、`/tmp/ntu_e3_frames.npz`（3,979 帧 body 云缓存）、`/tmp/ntu_eee_01_ate.png`、`/tmp/run_ntu_lean.sh`；录制 `/home/ywj/slam_data/ntu_odom_eee_01_a`（`/lio/odom` 3,754 条）。

### 36.18 评测协议补全：`odom_vs_gt.py` 增加区间旋转 RPE（2026-11-16）

**动机**：此前 `tools/odom_vs_gt.py` 只有位置 ATE。位置 ATE 对**绕运动方向的旋转**几乎不敏感，而 M5 要证明的恰恰是旋转（§36.11 的 3.0~3.3×）。更根本的是：**NTU VIRAL 真值只有位置**——`ground_truth.csv` 的 10 列里四元数列在全部 19 个序列恒为 (0,0,0,1)（实测 `std = 0`，§36.17 附带负结论已记）⇒ GT 无法约束旋转，旋转通道只能另找参照。

**口径**（与 §36.11 对齐，但多两层剥离）：

- 区间旋转：`ΔR_est = R_iᵀ R_j`（body 系），`ΔR_imu =` 陀螺梯形积分（端点线性内插，`dt > 50 ms` 的段跳过）。
- 误差：`e = log( (R̂ ΔR_imu R̂ᵀ)ᵀ ΔR_est )`。三层剥离各自吃掉一种系统项：
  1. `R̂`：Kabsch 拟合 IMU↔odom 的**常值**旋转（吃掉外参 / 轴系差异；NTU 配置里 `lidar_imu` 旋转是单位阵，实测拟合出 1.1°，说明仍有残余轴差）；
  2. 逐轴去中位：吃掉陀螺零偏造成的常值漂移；
  3. 时间偏移扫描 δ∈{−0.02…+0.02}：雷达/IMU 时钟错位会伪装成旋转误差，扫描找谷底。
- 输出 `|e|` 的 RMS/中位/P90 与 `|e_z|`（偏航）；`e0`（不拟合、不去中位）单独打印，这一列才与 §36.11 的 `angle_of(ΔR_lioᵀ ΔR_imu)` 严格同口径。
- 角度一律走 `log_so3()`：SVD 投影回 SO(3) + 180° 特判，**不用 `arccos(clip(trace))`**（§36.9 已确认它会把 ≤0.1° 误报为 0）。

**代码改动**（`tools/odom_vs_gt.py`，位置 ATE 部分未动）：新增 `load_imu` / `exp_so3` / `log_so3` / `integrate_gyro` / `pairs_at_dt` / `fit_const_rot` / `_rot_stats` / `rotation_interval_rpe`；`load_gt` 改读 10 列并显式检查姿态列有效性；CLI 新增 `--imu-bag`（默认取第一个 `--bag`）、`--imu-topic`、`--rot-dt`（逗号分隔，默认 0.1）。

**合成自检**（`/tmp/test_rot_rpe*.py`，临时不入库）：

| 用例 | 注入 | 期望 | 实测 |
|---|---|---|---|
| 白噪声 | 0.15°/轴·帧（右乘） | 区间差分放大到 √2×0.15≈0.21°/轴 | `|e|` RMS **0.365°**、中位 0.321°、P90 0.531°、`|e_z|` 0.210°；`R̂` 拟合回 0.010°；δ 扫描平坦（白噪声天然无谷） |
| 10 ms 时钟错位 + 平滑角速度 | odom 戳早 10 ms | 理论上谷底在 δ=+0.010 | 不可分辨（10 ms × 角加速度对 0.1 s 区间旋转的扰动仅 **0.013°**，远低于 0.37° 噪声）⇒ 该场景本身不敏感，非工具缺陷 |
| 10 ms 时钟错位 + 高动态（yaw 2 Hz 调制） | 同上 | 谷底在 δ=+0.010 | 扫描 **4.357°**（δ=+0.010）< 7.971°（δ=0）⇒ 方向与量级正确；残余大项来自合成"真值"用 10 Hz 梯形递推 2 Hz 调制角速度本身失真 |

自检还暴露一个**使用约束**：`integrate_gyro` 的 `dt > 50 ms` 空洞保护在 IMU 采样率 ≤20 Hz 时会把每一段都跳过、积分恒为单位阵（第一版自检就踩了这个坑，误差退化成真值区间旋转的几十度）。NTU 是 384 Hz、Retail 是 203 Hz，安全；换数据源需先看 `load_imu` 打印的间隔 P50。

**实测 1：NTU eee_01**（odom `/home/ywj/slam_data/ntu_odom_eee_01_a` 的 `/lio/odom`；IMU 取原始包 `ntu_viral/bags2/eee_01` 的 `/imu/imu`，n=153,347、P50 5.1 ms、零空洞；GT `ntu_viral/gt/eee_01/ground_truth.csv`）

位置 ATE(RMSE) = **0.098 m**（与 §36.17 精简基线逐位一致，说明读取链路无偏）：

| Δt | n | `e0` RMS（同 §36.11 口径） | `|e|` RMS | 中位 | P90 | `|e_z|` RMS | `|e_z|` 中位 |
|---|---|---|---|---|---|---|---|
| 0.1 s | 3524 | 0.3221° | **0.3184°** | **0.1741°** | 0.5306° | 0.0955° | 0.0196° |
| 1.0 s | 3747 | 1.1521° | 1.0467° | 0.5429° | 1.7814° | 0.4667° | 0.0430° |

- 0.1 s 中位 **0.1741°** 是图像链 0.0489° 的 **3.56×**（RMS 0.3184° 则为 6.5×）⇒ **E0 否决门在 eee_01 上独立复现**，不只成立于 Retail_Street。
- δ 扫描（0.1 s，中位°）：−0.020:0.189 / −0.010:0.180 / **0.000:0.175** / +0.010:0.175 / +0.020:0.178 ⇒ 谷底在 0~+10 ms，时钟对齐正常。
- 1.0 s 的 δ 扫描单调偏向负值（0.499/0.527/0.546/0.580/0.621），说明长基线下 LIO 的缓慢航向漂移会被 δ 部分吸收——**这也说明 δ 扫描只应在 0.1 s 量级基线上用于判时钟，不能拿长基线去调时序**。
- 偏航通道显著优于全轴（中位 0.020° vs 0.174°）：误差主要来自 roll/pitch，与 §36.11 的「逐轴强各向异性」一致。

**实测 2：Retail_Street 交叉校验**（`retail_odom_a` 的 `/lio/odom` n=420 / 41.9 s；`Retail_Street` 的 `/livox/imu` n=27,447 / 203 Hz）

0.1 s：`e0` RMS 0.1137°、`|e|` RMS 0.1100°、中位 **0.0890°**、P90 0.1727°、`|e_z|` 中位 0.0274°；δ 扫描 **−0.020:0.244 / −0.010:0.155 / 0.000:0.090 / +0.010:0.107 / +0.020:0.176**——真实数据上谷底清晰地落在 δ=0（±10 ms 即抬升 70%+），这是扫描机制有效的最强证据。

注意**该 0.089° 不能直接与 §36.11 的 0.1619° 对比**：`retail_odom_a` 只有 42 s（§36.11 用的是 `retail_abl/base.txt` 的 131.7 s 全段），且不是同一次运行。它在这里的作用是第二数据集 + 真实数据的 δ 谷底验证。

**结论**：
1. `odom_vs_gt.py` 现在一次调用同时给位置 ATE 与 0.1 s / 1.0 s 区间旋转 RPE（含偏航），且旋转通道**不需要 GT**（GT 无姿态时的唯一出路）。
2. eee_01 上 LIO 0.1 s 区间旋转误差中位 0.1741°，与 §36.11 Retail_Street 的 0.1493~0.1619° 同量级，3.56× 于图像链 ⇒ M5 的位姿级证据在第二个数据集上成立。
3. 后续 M2/融合/精修三者的旋转收益，统一用本工具报数（`--rot-dt 0.1`），不再各自造脚本。

**产物**：`tools/odom_vs_gt.py`（入库）；`/tmp/test_rot_rpe.py`、`/tmp/test_rot_rpe2.py`、`/tmp/test_rot_rpe3.py`、`/tmp/xcheck_retail.py`（临时探针，不入库）。

---

### 36.19 旋转精修接入节点后的终判：接线无误，瓶颈是视觉噪声被低估 40–100×（2026-11-16）

**动机**：§36.13 的 `RotationRefiner` 已接进 `visual_b_node`（只做旋转后处理），但 eee_01 三路实测「精修 0.2864° vs LIO 0.2926°」，几乎没有改善，且 dump CSV 里修正量 `|u|` 中位 0.2027°、**44% 帧顶到 clip=0.30°**。本条目定位该现象，并给出量化判决。

**1. 接线核对（结论：口径正确，不是接线 bug）**

- `PatchObservation::u/v` 写的是当帧 LK 结果（`pts_curr`），`rot_prev_.obs` 是上一帧的同一字段 ⇒ `a=上一帧像素`、`b=当前帧像素`，`est.R` 即 `R_c2c1`。
- 节点算 `R_b2b1 = R_bc·R_c2c1·R_bcᵀ`、`r_vis_body = log(R_b2b1)`，方向是 body_{k-1}→body_k，与 `rotation_refiner` 的 `R'ᵀ_{k+1}R'_k` 同向。
- `R_wb` 取自 `T_wl_frame_`（`poseFromSamples` → `LidarPoseInterpolator`，位置线性 + **slerp**），`t_pose = stamp + img_time_offset`。

**2. 三项排除**

1. **不是插值**：`pose_interpolation.hpp` 用 slerp（不是 log 线性插值）⇒ 姿态接近 ±180° 时不会出现轴符号翻转。
2. **不是 C++ 估计器移植错误**：合成单测 5 组，`err(R vs Rgt)=0.0069–0.0116°`、`err(R vs Rgtᵀ)=0.89°`、`px_med≈0.5 px`（与 CSV 的 0.5–0.6 px 一致）⇒ `est.R` 方向与数值均正确。
3. **自我纠错（作废上一轮「r_vis ≈ −1.1×r_lio」的结论）**：上一轮诊断脚本把区间旋转取成 `dRl = log(R[k−1]ᵀ R[k])` = `R_{b_{k-1}←b_k}`，而节点 `r_vis` 是 `R_{b_k←b_{k-1}}`，两者本就反号。改成同向后 `r_lio` 误差 RMS 0.37°/中位 0.25°（与 §36.18 工具口径的 0.281°/0.118° 同量级）⇒ **视觉增量没有反号，`M ≈ diag(−1.1,−1.16,−1.22)` 是诊断脚本的口径伪影**。附注：3×3 反对称矩阵的相似变换永远是 proper rotation，**不可能**产生 `−I`，这一点当时就该怀疑。

**3. 真实根因：噪声尺度错约两个数量级 + 长尾无门限**

- 权重：`visualNoiseW3()` 主轴 **0.0235/0.0278/0.0567°**，节点 CSV 里估计器自报 σ 中位 **0.0020/0.0044/0.0083°**；而实测（674 对）`r_vis` 对齐后残差稳健 σ（1.4826·MAD）= **0.3001/0.2194/0.1499°**、RMS = **0.4121/0.4133/0.7851°**。即视觉因子被赋了 **40–100×** 过高的权重。
- 后果：`|u|` 中位 **0.2027°**、P90 = max = **0.3000°**，**521/1188（44%）帧顶到 clip**；修正量由饱和规则而非观测决定。
- 长尾无门限：按 `px_med` 分档 RMS = 0.572 / 0.428 / 0.625 / **2.766°**（>1.0 px 档 n=59）；按 `n_inl` 分档 = 1.151 / **2.111** / 0.733 / 0.476°；`corr(|e|, px_med)=0.651`、`corr(|e|, n_inl)=−0.196`；`|e|>1°` 的离群 **60/674**。

**4. 视觉增量本身的精度（统一 body1→body2；Kabsch 常值旋转 + 逐轴去中位；n=674，Δt 中位 0.100 s）**

| IMU 积分区间偏移 | r_vis RMS | r_vis 中位 | r_vis P90 | r_lio RMS | r_lio 中位 | r_lio P90 |
|---|---|---|---|---|---|---|
| +0.00 s（= t_pose） | 0.9782° | 0.3992° | 0.9554° | 0.3718° | 0.2528° | 0.5727° |
| −0.05 s | 0.9150° | 0.3041° | 0.7230° | 0.3578° | 0.2412° | 0.5499° |
| **−0.10 s（图像自身区间）** | 0.8642° | **0.1580°** | 0.5193° | 0.3531° | **0.2277°** | 0.5415° |
| −0.15 s | 0.8711° | 0.1931° | 0.5525° | 0.3416° | 0.2197° | 0.5436° |

`r_vis vs r_lio` 几何角差 RMS 0.9226°/中位 0.2987°（与偏移无关）。关键：**在中位上视觉（0.158°）确实优于 LIO（0.228°），但 RMS 差 2.4×**——视觉是"多数更准、少数很糟"，而现行权重把"很糟"的那部分也当成 σ≈0.003° 来用。

**5. 离线重放（直接链接真实 `rotation_refiner.cpp`，输入为实测 `r_vis`/`R_lio`）**

`w3` 档复现节点结果逐位一致（`|u|` 0.2028°、饱和 43.9% vs CSV 的 0.2027°/44%），证明重放可信。评测口径同 §36.18（Δt=0.1 s，n=1187）：

| 臂 | 门限 | `|e|` RMS | 中位 | P90 | 偏航 RMS |
|---|---|---|---|---|---|
| LIO 基线（同采样） | — | 0.2810° | 0.1183° | 0.4605° | 0.0385° |
| `w3` 现行接线（clip 0.30） | 无 | 0.2864° | 0.1265° | 0.4659° | 0.0532° |
| σ=0.3° 各向同性 | 无 | 0.2582° | 0.1108° | 0.4149° | 0.0553° |
| σ=0.3° | px≤1.0 ∧ n_inl≥100 | **0.2573°** | **0.1100°** | **0.4221°** | 0.0469° |
| σ=0.5° | px≤1.0 ∧ n_inl≥100 | 0.2631° | 0.1094° | 0.4278° | **0.0419°** |
| σ=0.5°，clip=1.0° | px≤1.0 ∧ n_inl≥100 | 0.2631° | 0.1094° | 0.4279° | 0.0419° |

（`σ=0.5°` 两臂数值相同是因为饱和已降到 0.0%，clip 放开不再起作用。）

**6. 结论**

1. **现行 W3 权重下这条通道不该开**：0.2864° 略差于同采样 LIO 0.2810°，偏航还从 0.0385° 退化到 0.0532°——权重过大让视觉长尾直接注入修正量，再被 clip 削顶。
2. **换成实测稳健 σ≈0.3° 各向同性 + 门限（`px_med≤1.0 px`、`n_inl≥100`）后有收益但有限**：0.1 s 区间旋转 RMS **0.2810→0.2573°（−8.4%）**、中位 0.1183→0.1100°、P90 0.4605→0.4221°，可用帧 681→556；代价是偏航 RMS 0.0385→0.0469°（+22%）。**偏航通道本来就很准（0.0385°），任何视觉注入都在稀释它**，这与 §36.12「逐轴强各向异性」一致。
3. **§36.12 的 W3 不能当线上有效噪声**：它描述的是理想口径下的本征散布，而线上残差还叠了 `t_pose` 与图像时刻错位（0.1 s）、LIO 位姿插值误差、特征跟踪长尾。§36.14 的「0.1387°→0.051°」是**去共模 + 理想区间**口径，与线上 Δt=0.1 s 的绝对 RPE 不可直接对比；线上视觉通道绝对精度只能到 **中位 0.158° / RMS 0.864°**。
4. 若坚持投偏航，需要逐轴各向异性权重（yaw 单独压权重）+ 只放开 roll/pitch，属于 M5 之后的事；当前不改默认行为。
5. `rotation_estimator.cpp::procrustes()` 返回 `V D Uᵀ`（Python 参考实现是 `U D Vᵀ`），初值被转置；合成单测已证明 LM 收敛后无影响，本轮不改，仅登记。

**产物**：`/tmp/diag_rot9.py`（统一口径精度对照）、`/tmp/diag_rot10.py`（饱和/长尾分档）、`/tmp/prep_replay.py`（重放输入）、`/tmp/replay_refiner.cpp` + `/tmp/run_replay.sh` + `/tmp/eval_replay.py`（真实修正器重放与评测）；均为临时探针，不入库。

---

## 37. 方案 B 现状态 Retail_Street 跑批：三轮参数对照（2026-11-16）

**前提/动机**：在 `visual_b_app`（进程内 Super-LIO + B 视觉支路，`app.visual_inject:=true`，该记录为改名前命名，现名 `super_livo_visual.inject`）的**当前代码与配置**下，于 Retail_Street 跑一轮完整 B 方案，产出可核对的轨迹与彩色点云，并确认相对 2026-11-14 旧结果没有回归。

**配置（三轮共用）**：`config/retail_lio.yaml` + `solution_b_tight_coupled/super_livo_visual_b/config/visual_b_params.yaml`（下列键名为该记录当时的写法，2026-11-19 格式统一后全部收进 `super_livo_visual` 分支，见 §40.3；Retail 相机内参、`scale 0.5`、`color.use_own_odom false`、`rot_refine.enable false`、`est.enable_visual true`、`lio.ros.lidar_qos_reliable false`、`lio.ros.imu_qos_reliable false`）；`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、`ROS_DOMAIN_ID=86`、`ros2 bag play /home/ywj/slam_data/Retail_Street_d2 --rate 1.0`。每轮并行录 `/lio/odom`、`/super_livo_visual_b/odom`、`/lio/degeneracy`（rosbag2）+ `odom_recorder.py` CSV。

**三轮只改一个变量**：

| 轮次 | 变量 | 输出目录 |
|---|---|---|
| 1 | 现行默认（`pcd.save_interval_sec=30`） | `/home/ywj/slam_data/b_retail_20261116` |
| 2 | `pcd.save_interval_sec=0`（只在退出时存） | `/home/ywj/slam_data/b_retail_20261116_clean` |
| 3 | 轮 2 + `lio.ros.lidar_qos_reliable:=true` + `image.qos_reliable:=true` | `/home/ywj/slam_data/b_retail_20261116_qos` |

**实测（录制流一律先按 `header.stamp` 排序再算，见 §0 约定）**：

| 指标 | 轮 1（默认） | 轮 2（不周期落盘） | 轮 3（reliable） |
|---|---|---|---|
| `/lio/odom` 条数 | 1293 | 1308 | 1316 |
| 时长 | 134.10 s | 134.70 s | 134.60 s |
| 路径长度 | 66.83 m | 66.81 m | 66.82 m |
| 闭环差（首末点） | **23.0 mm** | 34.2 mm | 35.0 mm |
| z 范围 | [−0.01, 1.25] | [−0.01, 1.19] | [−0.01, 1.22] |
| 空洞 >0.3 s（起点, 长度） | (8.7,0.4)(42.1,3.2)(88.5,0.4)(94.1,0.4)(105.4,0.4)(126.7,0.4)(127.6,0.4) | (42.7,3.2)(66.2,0.4)(100.3,0.4)(113.9,0.4) | (42.6,3.2) |
| B 自有 odom | n=1283 / 69.37 m / 16.7 mm | n=1299 / 68.17 m / 102.3 mm | n=1314 / **85.37 m / 16352.1 mm** / z[−2.99,1.24] |
| CSV（到达序→排序） | n=1271 / 66.79 m / 26.6 mm | n=1305 / 66.80 m / 33.8 mm | n=1307 / 66.80 m / 34.7 mm |
| 注入 push/take/生效/被拒/过期 | 1283/1250/1248/0/2（97.3%） | 1299/1283/1281/0/2 | 1314/1302/1299/0/3 |
| 原始到达序 header 逆序条数 | `/lio/odom` 66、B 58 | 50、49 | 77、77 |
| 彩色 PCD | **16,815,272 点 / 269 MB** | 无 | 无 |
| Lidar->Odom 末段 avg/max | 17.33 / 58.27 ms | 17.89 / 58.43 ms | 16.90 / 57.62 ms |

**结论**：

1. **融合轨迹无回归**：三轮 `/lio/odom` 路径 66.80–66.83 m、闭环 23–35 mm，注入生效率 97.2–97.3%；与 11-14 同数据集旧代码 B（`/home/ywj/slam_data/output/retail_ab_1114/b/odom.csv`，n=1270、raw span 132.90 s、66.82 m、闭环 19.9 mm、PCD 244 MB）一致。轮间差异（23 ↔ 34/35 mm）属轮次噪声，按 §0 不做跨轮判决。
2. **PCD 只能靠周期落盘拿到**（**已由 §38 的退出同步落盘关闭**）：`visual_b_app` 对 SIGINT 无响应（三轮均需 SIGKILL），析构里的 `saveColoredMap()` 不执行。轮 1 的 PCD 实际来自第 5 次定时落盘（相对 bag 开始 +24/+54/+84/+114/+144 s，点数 2,819,737→6,572,926→10,845,467→14,916,945→**16,815,272**）；轮 2/3 把 `pcd.save_interval_sec` 置 0 后完全无 PCD（`save_seen_after_sigint=0`）。⇒ 需要点云时必须保留 `pcd.save_interval_sec>0`；要评测 odom 不受落盘阻塞时再置 0。
3. **t≈42 s 的 3.2 s 空洞是系统性的，不是数据问题**：原始 bag 该段连续——`/livox/lidar` 40–50 s 内 100 帧（全段 1355 帧、最大间隔 **0.1195 s**）、`/livox/imu` 40–47 s 内 1396 条（最大 0.1169 s）、`/left_camera/image` 70 帧（最大 0.1096 s），无重复戳、该段无逆序块；而三轮与 11-14 旧运行都在 t≈42 s 出现同一 3.2 s（约 32 帧）缺口，`/lio/odom` 与 B 自有 odom 同时缺。换 reliable QoS 无效 ⇒ 丢样发生在消费侧（Super-LIO 输入/同步路径），根因**未定位**，登记待查。
4. **轮 3 的 B 自有 odom 异常**：85.37 m、闭环 16.35 m、z 最低 −2.99 m，而同轮融合 `/lio/odom` 正常（66.82 m / 35.0 mm）、注入 take 1302/1314（99.1%）、生效 1299（98.9%）。该轮 B 自有通道数据作废；异常在 B 的独立状态递推/发布侧，不影响本轮融合结论（`color.use_own_odom=false`，着色用的是 `/lio/odom`）。
5. **header 逆序随运行变化（66/50/77）**，成段出现且与周期落盘时刻不严格对应（轮 2 无落盘仍有 50 条）⇒ 成因未定位；先排序后计算是硬约定，否则同一录制流会算出 250 m 量级路径与 2 m 级假差。
6. **耗时与资源**：单帧 Lidar->Odom 平均 16.9–17.9 ms、最大 58–63 ms（10 Hz 输入，余量约 5.6×）；彩色地图单次落盘阻塞可见（末帧 `彩色=688.1 ms`），但轮 1 的 5 次落盘均未拖慢 30 s 定时（相邻落盘间隔 29.97/30.31/30.11/30.00 s）。

**产物**：`/home/ywj/slam_data/b_retail_20261116/`（轮 1，含 `super_livo_visual_b_colored.pcd`）、`/home/ywj/slam_data/b_retail_20261116_clean/`（轮 2）、`/home/ywj/slam_data/b_retail_20261116_qos/`（轮 3），各含 `node.log`、`odom_lio.csv`、`odom_bag/`、`play.log`、`meta.txt`；跑批/分析脚本在 `/tmp/run_b_retail{,2,3}.sh`、`/tmp/an_b_retail2.py`、`/tmp/probe_bag.py`、`/tmp/probe_hole.py`、`/tmp/probe_imu.py`、`/tmp/probe_wallgap.py`、`/tmp/probe_rate.py`、`/tmp/probe_csvpath.py`，均为临时探针，不入库。

---

## 38. 相机-雷达外参复标定（all bag + 同步标定参数）与 FL2/A/B move 三臂跑批（2026-11-16）

### 38.1 前提、输入与判据

1. **触发**：用户同步了一版标定参数（FAST-Calib-ROS2 子模块 main `8963f32`「MV-CB013 内参 / 标定板 / 距离门限全精度落到 main」），要求用 `all` bag 重标外参，再用 FL2 / A / B 三臂跑 move。
2. **标定输入**（沿用既定协议）：本地 `livo_bag/all_ros2` **全段累积点云**（8,260,032 点）+ 第 120 帧图像 `img_0120.png`，复刻单帧 `img_0120` 协议；`third_party/FAST-Calib-ROS2/config/qr_params.yaml` 已指向本地路径，并注释掉空的 `bag_paths: []` / `image_paths: []`（空序列会抛 `InvalidParameterValueException`）。
3. **运行约定**：四 ArUco（`DICT_6X6_250`，id 1–4）+ 四圆孔；代码内不做 `cv::undistort`，图像先去畸变再按纯针孔 `K` 投影，四孔残差在**去畸变像素**上比较；节点名是 `mono_qr_pattern`，必须 `-r __node:=fast_calib` 才会加载 `fast_calib` 顶层参数（否则静默跑默认参数）。
4. **WSL 前置修复**（否则 ROS2 DDS 只有多播发现、无单播数据）：`ip rule` 的 `ipproto udp lookup 127` 命中 table 127 里的 `127.0.0.1 via 169.254.73.152 dev loopback0`，使发往 127.0.0.1 的 IPv4 UDP 单播被送进 WSL localhost 转发代理（不通）。跑批前执行 `ip route del 127.0.0.1 via 169.254.73.152 dev loopback0 table 127`（root）。

### 38.2 标定结果

- FAST-Calib 直出 **RMSE 0.0079 m**（8,260,032 点 + `img_0120.png`），其外参与 §25 的 remote7.9 **逐位一致**；标定板几何自检（半径/边长/面内残差）与 §25 相同（0.1244/0.1265/0.1289/0.1280 m，长边 493.6/490.9、短边 398.1/394.9 mm，面内残差 1.9/2.0/4.2/4.2 mm）。
- holefix 链（RANSAC 真实板面法向 + 前表面沿法向 +8 mm + 每孔 ±1° 扇区最小半径轮廓 + 圆拟合 + 2D Kabsch + 重力消歧）在新点云上重解，得本轮定稿：

```
Rcl = [-0.016751, -0.999859, -0.001174,
        0.458555, -0.006640, -0.888641,
        0.888508, -0.015424,  0.458601]
Pcl = [0.016642, -0.113077, -0.037398]
```

- 四孔重投影终判（帧 120，去畸变像素）：FAST-Calib 直出 **1.26/2.17 px**、旧 holefix **1.25/1.65 px**、**新 holefix 1.00/1.62 px** ⇒ 取新 holefix，已写入 `visual_a_move.yaml`、`visual_b_move.yaml`、`config/move_fastcalib.yaml`（三份逐位一致）。
- 新旧两版差 **0.63°/33 mm**（2 m 处 <1 px）。move 场景远场边缘距离场评分（Canny 边 + DT，边缘点取自单帧雷达非平面点，范围 1.5–15 m）：#592 旧 15.78 / 新 16.41 px（远场 >3 m 均值 16.31 vs 16.76）；#296 旧 2.98 / 新 3.07 px（远场均值 5.89 vs 5.72）⇒ **远场边缘指标对两版不可分辨**（差值 <1 px，本底 3–16 px），定稿依据仍是标定板四孔残差；更严格的远场判别需把板放到 ≥8 m。

### 38.3 move 三臂跑批

环境：`ROS_LOCALHOST_ONLY=1`、`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`、`ROS_DOMAIN_ID=78`；A/B 用 `move_livoxclk`（图像 header 已改写为 Livox 时钟）+ `lio_move.yaml` + `visual_{a,b}_move.yaml`，FL2 用 `move_ros2`（图像 header 保留主机时钟，靠 `img_time_offset: -204262818.9974` 对齐）+ `config/move_fastcalib.yaml`（内参由 `qr_params.yaml` 合并注入，`cam_model=Pinhole`）；轨迹话题 A/B 为 `/lio/odom`、FL2 为 `/aft_mapped_to_init`，均用 `odom_recorder.py` 落 CSV。

| 臂 | 样点 | 时长 | 路径长 | 闭环(end−start) | 输出 |
|---|---|---|---|---|---|
| FL2 | 691 | 72.10 s | 10.834 m | **0.344 m** | `odom.csv`、`all_raw_points.pcd`（4,119,817 点）、`all_downsampled_points.pcd`（54,087 点） |
| A | 739 | 73.80 s | 9.554 m | 0.029 m | `odom.csv`、彩色地图 3,342,799 点（调用 1375 / 实际着色 711 帧 / 无图像丢弃 47 / 等位姿超时 6 / 延后 226） |
| B | 739 | 73.80 s | 9.557 m | 0.024 m | `odom.csv`、彩色地图 3,351,876 点 |

1. **A-B 一致性**：同一 LIO，逐点差中位 **1.8 mm** / P95 3.5 mm / max **5.0 mm** ⇒ 新外参下 B 的紧耦合增量对轨迹无可见影响。
2. **FL2 与 A 的差异**：时间对齐后 Umeyama（不含尺度）对齐，残差中位 0.0998 / P95 0.3236 / **RMSE 0.1785** / max 0.5890 m，对齐旋转 **4.99°**（两套系统世界系偏航不同，正常）；路径长比 A 多 13%。
3. **FL2 的 0.59 m 瞬态跳变定位**：t=16.92 s 单帧位移 **0.592 m**（−0.024,0.025,0.676 → 0.295,0.520,0.737），随后 0.4 s 内分四步回落到 (0.157,−0.064,0.711)，y 最远到 −1.944 m（A 为 −1.512 m）；同段日志在 t≈16.0–16.1 s（wall 1794843161.17，相对 bag 起点换算）连报 12 条 `IMU and LiDAR not synced! delta time: 0.5595→0.5043`。原始 bag 该段 `/livox/imu`（最大间隔 0.041 s）与 `/livox/lidar`（最大 0.113 s）均无空洞，但 **`/left_camera/image` 有 2.634 s 空洞（14.87–17.51 s）**（另有 0.730 s 空洞 70.47–71.20 s），跳变发生在该空洞尾部前后 ⇒ 疑似图像中断/恢复期的视觉更新异常，**根因未定位，登记待查**（A/B 同期 `/lio/odom` 无任何跳变，最大帧间位移 0.069 m）。
4. **FL2 保存后 exit 139**：`all_raw_points.pcd` / `all_downsampled_points.pcd` 均已完整落盘（日志确认点数），随后进程段错误；不影响本轮结果。
5. **着色无劣化**（新旧外参同场景对照，彩色地图逐点统计）：newA 3,342,799 点 / mean RGB (122.5,122.1,121.0)、oldA 3,345,135 / (122.1,121.8,120.7)、newB 3,351,876 / (122.4,122.0,120.9)、oldB 3,330,340 / (122.2,121.8,120.8)；四者黑点(<10) 0.2%、彩色点(sat>40) 0.3%、sat 中位 4 ⇒ 覆盖与亮度分布一致。FL2 侧 PCD 无 RGB（不做彩色对照）。

**产物**：`/home/ywj/slam_data/output/move_fl2/`、`move_a/`、`move_b/`（各含 `odom.csv`，A/B 含 `super_livo_visual_{a,b}_colored.pcd`，FL2 含两版 PCD；旧对照 `move_{a,b}_holefix/`）；三臂俯视轨迹 `/home/ywj/slam_data/output/move_cmp3_traj.png`，远场双外参叠加 `/home/ywj/slam_data/output/edge_cmp_{592,296}.png`。跑批脚本 `/home/ywj/tmp/move_fl2.sh`、`move_run.sh`、`wrap_move_cfg.py`，分析脚本 `/home/ywj/tmp/{odom_stat,odom_jump,odom_win,traj_cmp_move,edge_cmp_move,color_stats,bag_gaps,bag_topic_gaps}.py`，均为临时产物，不入库。

**结论**：① 新标定参数下外参定稿为 §38.2 那版（四孔 1.00/1.62 px，优于 FAST-Calib 直出与旧 holefix）；② 三臂均跑通，A/B 轨迹与着色指标和旧外参无差异，新外参不引入回归；③ FL2 本轮轨迹质量弱于 A/B（闭环 0.344 m vs 0.024–0.029 m、13% 路径虚长），并定位到 t≈17 s 的 0.59 m 跳变与图像空洞同窗，需后续单独排查。

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

---

## 39. 方案 B 单体化（S2）接线核查与退出同步落盘（2026-11-19）

### 39.1 触发与判据

用户把 B 做成单体可执行 `visual_b_app.cpp`（`ROSWrapper` + `VisualPipeline` 同进程，节点名 `super_lio`），要求核查：① 启动时间对齐机制是否仍生效；② 跑批链路是否可用；没问题就提交。判据：`node.log` 出现对齐日志、`odom_rows` 与延迟行数不低于旧双进程口径、`pcd/` 必须真的有文件。

### 39.2 对齐机制核查（结论：继承，无需改码）

- 三路输入在单体内全部直推进 `VisualPipeline`：图像 `submitImage`、IMU `submitImu`、LIO 输出 `onLioOutput → submitOdom`，对齐样本链（图像 / 雷达帧 / IMU 滞后 D）完整；
- 参数经 `RosParamSource` 懒声明，`align.enable` 默认 `true`，`tryAlign()` 里 `off = −(D + a)` 逻辑不变；
- 实测 test4（离线回放）日志：`启动时间对齐放弃：系统钟与图像 header 相差 154697.2 s（离线回放，系统钟不是传感器主机钟），无法恢复雷达钟域；沿用配置 img_time_offset=-204701439.493700 s` ⇒ 机制运行且按设计回退，CLI `-p img_time_offset` 覆盖 YAML 生效。

### 39.3 发现的三处缺口

1. **`run_one.sh` b 臂仍起两个进程**（`super_lio_node` + 已删除的 `visual_b_node`）⇒ 跑不起来。改为单个 `visual_b_app`、同时喂 `livox_360.yaml` 与 `visual_b_move.yaml`；残留清理正则与 `tools/ros_batch_guard.sh` 的 `_LIVEPAT` 加入 `visual_b_app`。
2. **配置整块静默失效**：`visual_b_move/rtk/room.yaml` 顶层键写 `super_livo_visual_b:`，而单体节点名是 `super_lio` ⇒ 内参、`img_time_offset`、开关全部回落默认值且无告警。三份统一改 `/**`（2026-11-19 的 §40 在此基础上进一步把视觉参数收进
   `super_livo_visual:` 分支并做 A/B 同类合并改名）。
3. **退出时 PCD 不落盘**（本轮主问题）：见 §39.4。

### 39.4 退出落盘：异步投递被进程退出杀掉

| 现象 | test4/b 全量 1x（96 s）`pcd/` 只有目录、`files=0`；smoke 15 s 却出 7.1 MB 文件 |
| --- | --- |
| 首次误判 | 疑写盘失败 / 权限 / rosout 丢日志 —— 逐个排除 |
| 关键实验 | 手动起 app + `-p pcd.save_interval_sec:=10.0` 跑 45 s：运行中四次周期落盘全部成功（249,429 / 691,075 / 1,122,353 / 1,630,754 点）⇒ 写盘路径与权限无问题 |
| 时间戳证据 | 全量运行的 `pcd/` mtime 与 `[彩色统计]` 同刻（08:39:51.83），目录建了文件没写；smoke 的文件 mtime 与目录同刻 = 写完 |
| 根因 | `on_shutdown` 里 `requestSaveColoredMap()` 只是把快照投给独立 save 线程（PROBE 记录单次 737.7 ms，全量地图 60,000 voxels / 364 万点更久），主线程随即 `return 0`，线程被杀 |
| 修法 | 新增 `VisualPipeline::shutdownAndSave()`：`stopWorkerThreads()` → 补 `pending_visual_` / `drainPendingClouds()` / `drainPendingColorization()` → **同步** `saveColoredMap()`；`std::call_once` 幂等，shutdown 钩子、`main` 收口、析构三路调用，后到者阻塞到写盘完成。`flush()` 改为停机后立即返回（此前白等 30 s 打 `flush 超时` 告警） |

> 15 s smoke"通过"是假信号：图小刚好赶在退出前写完。**验证落盘必须用全量时长**。

### 39.5 验证与口径提醒

| 项 | 修前（`test4_check_b2`，脚本改后首跑） | 修后（`test4_check_b3`） |
| --- | --- | --- |
| `pcd/` | `4.0K / files=0` | **56 MB / files=1**，`3,642,499` 点（`FIELDS x y z rgb`） |
| `odom_rows` | 502 | 502 |
| `latency_lines` | 1407 | 1407 |
| `dt_img_cloud` | +0.0036 s | +0.0035 s |
| 退出码 | 0 | 0（同步落盘在退出前完成） |

**口径变化**：单体路径 `app.visual_inject`（该记录为改名前命名，现名 `super_livo_visual.inject`）默认 `true`，b 臂从此**真正把视觉约束写回 ESKF**；
`paper_20261119` 的 B 行出自双进程跑批（跨进程无法注入，见 §38 / `paper_compare/REPORT.md` 前言）。
论文矩阵的 B 行需用单体形态重跑后才可与其他臂并列，**不可混用两轮数据**。

**产物**：`/home/ywj/slam_data/test4_check_b{,2}/`（修前，无 PCD）、`/home/ywj/slam_data/test4_smoke_b/`（15 s smoke）、`/home/ywj/slam_data/test4_check_b3/`（修后，含 PCD）；诊断脚本 `/tmp/repro_b_exit.sh`、`/tmp/diag_b_save.sh` 为临时探针，不入库。代码改动：`bf916ef`。

---

## 40. 全仓 params YAML 格式统一与 A/B 同类参数合并改名（2026-11-19）

**结论**：仓库所有 ROS 2 params YAML 收为同一套格式——顶层只写 `/**` + `ros__parameters`，
原版 Super-LIO 主干参数保持扁平 `lio.*` 名字不动，两条视觉支路的参数统一收进
`super_livo_visual` 分支并**共用同一套键名**（B 与 A 语义相同的旧名并入 A 的命名族）。
合并后的同一份文件既能喂 `visual_a_app` / `visual_b_app`，也能直接喂官方 `super_lio_node`
跑纯 LIO；历史上"顶层写死节点名导致整块参数静默失效"的坑（§39.3 ①）一并消灭。
**本轮只动配置与文档 + 读取侧前缀，未改任何算法逻辑。**

### 40.1 规则原文

```yaml
/**:
  ros__parameters:
    lio.ros.lidar_topic: "/livox/lidar"     # ① 原版 Super-LIO 主干段：扁平 lio.* 名字，与上游一字不动
    super_livo_visual:                       # ② 视觉支路段（方案 A / 方案 B 共用同一套名字，同类已合并）
        cam_width: 1280
        Rcl: [-0.0171, -0.9998, -0.0064, ...]
        gate.max_reprojection_rmse: 6.0
        pcd:
            save_dir: "/home/ywj/slam_data/output/move_a"
```

三条硬约束：① 顶层一律 `/**`，**禁止写死节点名**（`super_livo_visual_a:` / `super_livo_visual_b:`
都不允许）；② 视觉参数只能挂在 `super_livo_visual:` 分支下，分支内允许 `a.b` 扁平键与
`pcd:` 嵌套块混用；③ `experiments/config/livox_360_visa*.yaml` 里的 `lio.visual.*` 是
**上游 Super-LIO 自带视觉**的参数，属于 ① 段，不迁移、不改名。

### 40.2 摊平机制与读取侧

Humble 的 rcl 会把 `ros__parameters:` 下的**嵌套 map 摊平成点号参数名**，因此分支里的键最终
在 ROS 参数服务上的真实名字是 `super_livo_visual.cam_width`、`super_livo_visual.pcd.save_dir`。

| 侧 | 加前缀的位置 |
| --- | --- |
| A | `solution_a_lightweight/super_livo_visual_a/include/super_livo_visual_a/node/visual_a_node.hpp` 里的 `vp("name")` helper，声明/读取统一走它 |
| B | `solution_b_tight_coupled/super_livo_visual_b/src/app/visual_b_app.cpp` 的 `getOrDeclare` 内用常量 `kVisualParamPrefix` 统一加 |

推论：命令行 `-p` 覆盖**必须写全名**（`-p super_livo_visual.pcd.save_dir:=…`），写短名等于给一个
未声明的参数赋值 ⇒ 静默无效。文档与注释里写分支内短名（`gate.*`、`pcd.save_dir`）时一律省略前缀。

### 40.3 B 侧旧名 → 新名（同类合并）

其余名字不变，只是整体加上 `super_livo_visual.` 前缀。

| B 旧名 | 新名（分支内短名） | 说明 |
| --- | --- | --- |
| `lv.huber_delta` / `lv.sigma_pix` / `lv.sigma_anchor` / `lv.min_constraints` / `lv.max_pixel_error` / `lv.max_pixel_error_anchored` | `reproj.` 同名 | 与 A 的 `reproj.*` 合并 |
| `lv.pixel_radius` | `depth.pixel_radius` | 与 A 的 `depth.pixel_radius` 同一概念（两处都喂 `associationVariance`） |
| `lv.anchor_keep_sec` | `depth.anchor_keep_sec` | — |
| `lv.prior_pos_sigma` / `prior_pos_sigma_degen` / `prior_pos_sigma_healthy` / `prior_rot_sigma` / `prior_vel_sigma` | `update.prior_*` | 与 A 的 `update.prior_*` 合并 |
| `lv.prior_directional_en` | `update.directional_prior_en` | — |
| `est.enable_visual` | `update.enable` | 与 A 的 `update.enable` 同义 |
| `app.embed_visual` / `app.visual_inject` / `app.visual_inject_max_dt` | `embed` / `inject` / `inject_max_dt` | 挂在 `super_livo_visual` 分支根下 |

**刻意不合并**的三组（机制不同，强行合并会改行为）：A 的 `gate.*`（退化时整帧门控 / 降权阈值）、
B 的 `est.visual_info_ratio(_degen)`（信息矩阵比例）、B 的 `guard.*`（LIO 跳变守护）。

| 归属 | 参数族 |
| --- | --- |
| A 独有 | `gate.*`、`klt.*`、`evo.*`、`image_save.*`、`odom_topic`、`cloud_topic`、`body_cloud_topic`、`degen_topic`、`lio_source` |
| B 独有 | `est.*`、`msckf.*`、`photo.*`、`rot_refine.*`、`scheduler.*`、`lifecycle.*`、`kf.*`、`guard.*`、`patch.*`、`e2.*` |
| A/B 同名 | `cam_*`、`Rcl`、`Pcl`、`scale`、`image_topic`、`imu_topic`、`align.*`、`color.*`、`pcd.*`、`image.qos_reliable` |

### 40.4 验收（实测）

| # | 判据 | 结果 |
| --- | --- | --- |
| 1 | 同一份合并 YAML 可直接喂官方 `super_lio_node` 跑纯 LIO（未声明的 override 被 rclcpp 静默忽略） | ✅ `ros2 param get /super_lio super_livo_visual.embed` 返回 `Parameter not set`；`lio.sensor.blind`、`lio.kf.kf_max_iterations` 正常生效 |
| 2 | `visual_a_app` 读到分支内参数 | ✅ `super_livo_visual.cam_width=1280`、`gate.max_reprojection_rmse=6.0` |
| 3 | `visual_b_app` 读到合并改名后的参数 | ✅ `reproj.sigma_pix=1.5`、`update.enable=True`、`depth.pixel_radius=3`、`inject=True` |

收益：`super_livo_visual` 分支对主干是透明的，纯 LIO / 带视觉不必再维护两份配置。

### 40.5 改动文件

14 份 YAML：`common/super_livo_common/config/lio_move.yaml`、`config/retail_lio.yaml`、
`experiments/octvox_ablation/20_move_{a,b}_node.yaml`、
`solution_a_lightweight/super_livo_visual_a/config/visual_a_{params,move,ntu,room,rtk}.yaml`、
`solution_b_tight_coupled/super_livo_visual_b/config/visual_b_{params,move,ntu,room,rtk}.yaml`。

代码（仅读取侧加前缀）：`visual_a_node.hpp`（`vp()`）、`visual_a_node.cpp`、`visual_a_app.cpp`、
`visual_b_app.cpp`（`kVisualParamPrefix`）、`pipeline/visual_pipeline.cpp`。

17 个脚本共 **96 处 `-p` 覆盖**改为全名，按参数名分布：

| 参数名 | 处数 | 参数名 | 处数 |
| --- | --- | --- | --- |
| `pcd.save_dir` | 17 | `pcd.save_interval_sec` | 15 |
| `odom_topic` | 10 | `cloud_topic` | 10 |
| `body_cloud_topic` | 10 | `color.use_own_odom` | 8 |
| `color.cam_odom_topic` | 6 | `image_topic` | 5 |
| `color.sync_max_offset` | 2 | `inject` / `img_time_offset` / `image.qos_reliable` / `gate.cond_lo` / `gate.cond_hi` / `pcd.median_voxel_size` / `color.{patch_radius,patch_color_tol,own_res_max_rot,own_res_max_pos,min_quality,depth_tolerance,check_depth_edges}` | 各 1 |

脚本清单：`experiments/octvox_ablation/{10,11,12,13,14}_run_colored_*.sh`、
`experiments/octvox_ablation/16_run_superlio_a_color.sh`、`16b_run_superlio_b_color.sh`、
`20a_run_move_a.sh`、`20b_run_move_b.sh`、`_probe_body_qos.sh`、`_probe_replay_aligned.sh`、
`_probe_replay_monitor.sh`、`_probe_two_bag_node.sh`、`_replay_node_on_record.sh`、
`experiments/scripts/paper_matrix/run_one.sh`、`experiments/scripts/retail_ab/run_ab_retail.sh`、
`tools/run_ntu.sh`。

文档同步：根 `README.md`（新增「参数文件规范」小节）、`solution_b_tight_coupled/README.md`、
`monolith_refactor_plan.md`、`monolith_refactor_progress.md`、`experiments/paper_compare/REPORT.md`。

### 40.6 历史口径提醒

本文档 §1–§39 与 `solution_b_tight_coupled/README.md` §9、`monolith_refactor_progress.md` §2
中已跑完的实验记录**保留当时的旧参数名与数值**（`lv.*` / `app.*` / `est.enable_visual` /
短名 `-p img_time_offset:=…`），未回填改写；涉及处已就地标注"该记录为改名前命名"。
`paper_20261119`、`test4_check_b*` 等历史结果如需复现，按旧名写 `-p` 不会生效，须用 §40.3 的新名。

