# 实验记录、评测与对照

本目录保存 Super-LIVO 的**实验过程、评测方法与结论**。仓库根 [README](../README.md) 只保留
项目定位与使用说明，所有跑批数据、消融、对比与踩坑记录都在这里。

## 目录

| 路径 | 内容 |
| --- | --- |
| [exp_records.md](exp_records.md) | **主线实验日志**（按编号追加，§1–§43）：每次改动的原因、命令、数据与结论 |
| [scripts/paper_matrix/](scripts/paper_matrix/) | **当前主线跑批**：四臂 `fl2 / a / b / sl` × 七个序列，1x 实时、统一探针、互斥与残留清理 |
| [paper_compare/REPORT.md](paper_compare/REPORT.md) | 论文对比矩阵汇总（时效性 / 精度 / 成图） |
| [fl2_img_ablation/](fl2_img_ablation/) | 官方 FAST-LIVO2 有/无图像消融（×3 轮） |
| [octvox_ablation/](octvox_ablation/) | 地图表示（OctVox）消融；含 LIO 核 + 方案 A 上色对比跑批 |
| [scripts/README.md](scripts/README.md) | 历史一次性脚本归档（B2/B3/χ² 门控/退化实验），硬编码 `/tmp`，仅作复现参考 |
| [config/](config/) | 实验期配置快照（含已废弃版本，仅作历史对照） |
| [speed_test.md](speed_test.md) | RK3588 实机测速原始报告 |
| [../docs/monolith_refactor.md](../docs/monolith_refactor.md) | A/B 单体化的设计、实施与验证数据 |

统一跑批入口：

```bash
OUTROOT=/home/ywj/slam_data/paper_$(date +%Y%m%d) RATE=1.0 \
  bash experiments/scripts/paper_matrix/run_one.sh <seq> <arm>
# seq: ntu01 ntu03 rtk retail move test4 room
# arm: fl2 a b sl        （SMOKE=秒 只播前 N 秒；FORCE=1 重跑；禁止并行）
```

产物 `<OUTROOT>/<seq>/<arm>/`：`node.log` `play.log` `latency.csv` `odom/` `pcd/` `meta.txt` `done`。

---

## 1. 评测纪律与指标口径

- **只认同 bag 逐点差或多跑分布**；±0.05 m 内的差异不算改善，不接受用危险单帧跳变换来的终点收益。
- **Super-LIO 在裁剪输入下不可复现**（同输入终值随机），一切对照只看同 bag 内逐点 `|VIS−LIO|`
  或多跑分布；单次数字禁止外推。
- 不自造退化检测，只消费 Super-LIO 的 `/lio/degeneracy`（或进程内 `degeneracySnapshot()`）。

必须同口径的三项指标：

| 指标 | 定义 |
| --- | --- |
| odom 发出延迟 | 雷达消息到达回调（steady 墙钟）→ 该帧 odom publish 完成的墙钟差；含排队与线程调度 |
| 整帧处理时间 | LIO 帧全流程 CPU 耗时：Super-LIO `[FrameProcess]`、FL2 `[LIOTotal]` / `[VIOTotal]`、A/B 视觉 `合计` |
| 波动 | 全体样本 std + P95/max（A/B 视觉是 ~1 s 一条采样，不是全帧） |

---

## 2. 与官方 FAST-LIVO2 的差异

三者共享"LiDAR 建图 + ESIKF 更新"的主干，差异在**视觉观测怎么构造、以谁为锚点**：

| 维度 | 官方 FAST-LIVO2 | 方案 A（轻量） | 方案 B（紧耦合） |
| --- | --- | --- | --- |
| 基座 | 自带 LIO+VIO 一体的 ESIKF | Super-LIO（复用其 ESIKF） | Super-LIO（复用其 ESIKF） |
| 视觉观测形式 | LiDAR 地图点投影到图像，取 **patch 光度误差**（直接法） | 角点（FAST + PyrLK）+ LiDAR 深度关联，**2D-3D 重投影误差**（几何法） | 同 A 的几何法，另加 **MSCKF 多帧零空间约束** |
| 视觉锚点来源 | 全局 LiDAR 地图点（地图漂了锚点就漂） | 全局 LiDAR 地图点（同 FL2，深度来自地图） | 地图点 **+ 健康期建立的持久锚点**（跨代绝对锚定，`anchors_` 只在成功观测时刷新 last_seen） |
| 是否依赖全局地图 | 是 | 是 | 部分否（MSCKF 用多帧位姿三角化，不依赖地图） |
| 退化处理 | 无显式退化检测，持续融合 | 消费退化签名，退化时才注入 + 状态回灌（`update.reinject`） | 消费退化签名，cond/sev 映射成先验注入强度，另加先验否决/速度限幅守卫 |
| 先验信息上限 | 无显式上限 | 有 | 有（`visual_info_ratio` 常态 3× / 退化 10×）+ σ 地板，防止一帧压塌协方差 |
| 视觉计算量（RK3588 单大核） | LIO 71 + VIO 28 ≈ 99 ms/帧 | 视觉合计 21.9 ms（std 4.7） | 视觉合计 41.8 ms（std 27.2，Patch 前端改 LK 前口径，未重测） |
| 真值精度（NTU eee_03 ATE） | **0.028 m**（最强） | 未单列（对位姿几乎无纠正力） | 见 §3，退化段方向稳定 |

一句话概括：

- **FL2 把视觉当"地图的稠密光度校正"**——精度天花板最高，代价是计算最重、且对 LiDAR 地图
  退化没有专门的对抗手段（地图一漂，光度残差就跟着错）。
- **A 把视觉当"退化时的稀疏几何补充"**——代价最小，但走廊段可用重投影约束太少
  （实测 n=0~33），对位姿几乎无纠正力。
- **B 把视觉当"不依赖地图的多帧几何约束 + 跨代绝对锚点"**——纠正力最强且方向稳定，
  代价是计算最重、实时性临界。

---

## 3. 关键实验结论

- **NTU eee_03 ATE**：官方 LIVO 0.028 m / 官方纯 LIO 0.043 m；Super-LIO 0.138→0.130 m
  （观测模型对齐后）。剩余差距主因是地图表示（0.25 m 子体素均值压缩）的内存/速度取舍，不建议改。
- **RTK-SLAM Dataset**（全站仪 <1 cm 真值，不做 SE(3) 对齐，`construction_seq2` 整轮 16 个 GT 点全覆盖）：
  `super-lio` 0.347 m / `fast-lio-sam` 0.374 m / `super-lio-beam3` 0.508 m / **官方 FAST-LIVO2 0.561 m** /
  `super-lio-beam` 0.656 m。fl2 的 SE(3) 误差 0.091 m 是最低的一档——"形状准、绝对位置不准"；
  其余系统无累积漂移，误差主要是子图刚体偏置而非逐帧抖动。
- **三层点云偏差（`tools/rtk_pcdev.py`）**：以 fl2 的稠密建图为参考面网（39457 面 / 覆盖 26.7%），
  量各系统点的 L1 距离分层散布、L2 跨帧重影、L3 绝对刚体偏差。L1 出斜率（如 fl2 σ0=0.96 cm、
  σθ=0.134°，beam3 σθ=0.282°）；**L3 目前分辨率只到 ~10 cm**（参考面厚度上限 8 cm 主导），
  各系统法向 MAD 7.7–10.6 cm 全落在自比对刻度 9.53 cm 的 0.81–1.11 倍内，**不能据此排序**；
  L2 样本稀疏且非单调，暂不结论。要让 L3 可排序需换更薄的面或改点到最近点的法向距离。
- **官方 FAST-LIVO2 的内存坑已解决（fork `b9f931f`）**：开图像时原实现会让每个视觉地图点钉住一帧
  完整原图（`FEATURE::img_`），1.92 MB/帧 × 16.5 Hz ≈ 32 MB/s，加上 VisualPoint 自身增长 ≈
  16 MB/s，合计 ≈48 MB/s 无界涨；`img_en=1` 51.8 MB/数据秒 vs `img_en=0` 4.3 MB/数据秒，
  12 倍差全在视觉侧。`pcd_save.interval` 分片只保住已 flush 的地图，**对这部分内存无效**
  （旧轮 46 分片仍在 480 s / 80% 被 OOM）。现在 `pruneVisualMap` 在每次分片落盘后以当前帧时刻
  为 cutoff 删掉已落盘点（连带释放图像）并清空 `warp_map`，当前帧点留到下一轮匹配：本机 23 GB
  可跑满 **595.5/599.5 s = 99.3%**，58 分片对应 58 次 prune，合并 24.88M → 8.92M 点 @5 cm。
  代价是视觉地图只保留最近一片（约 200 帧 / 12 s），**长时视觉重观测能力被放弃**；
  若要做长序列回环需改成按空间距离淘汰。
- **图像消融**（`fl2_img_ablation/`）：官方 `img_en=1/0` ×3 轮判决——图像确实参与位姿（厘米级），
  稳态精度增益≈0，有效作用是"锚定/防跑飞"。
- **LiDAR 退化对抗**（NTU eee_03 走廊裁剪，`Y_HALF=1.2` 温和退化，B 侧 8 次重复单跑
  tag=y12fix2~y12fix9）：B 方案（绝对锚定 + 历史锚点保持 + sev 映射 + 守卫）**8/8 次不劣于
  同 bag LIO**，且把终点误差的跨跑散布从 LIO 的 14.1~794.4 m（55×）压到 B 的 8.4~127.8 m
  （15×）——视觉介入不仅降低误差，还压缩了不可复现性本身。幅度只按分布口径给：均值
  71.4 vs 267.7 m（−73%）、中位数 84.9 vs 170.3 m（−50%）、逐次比值 0%~87%（中位 41.5%）；
  **单次数字不可复现，禁止再引用"437→123 m / 72%"这类单跑结论**。选轨耗时已修复
  （视差扫描 O(n²)：`kMaxObservations=16` 截断 + 视差缓存），8 次跑 `>50 ms 帧数 = 0/176~178`。
  A 方案状态回灌链路已通但对位姿无纠正力（同 bag `|A−LIO|≤0.7 m`），瓶颈在约束质量（走廊段
  大量帧判"健康不更新"/rmse 拒绝，可用重投影约束 n=0~33 太少）。细节见两份方案 README 的退化章节。
- **自采 2026-10-30 bag**：三种跑法（Super-LIO / FAST-LIVO2 纯 LIO / LIVO）均存在约 17° yaw
  漂移重影，源于 IMU 陀螺零偏（gz≈−0.42°/s）+ 回环未纠正，属数据侧问题；
  诊断过程见 [exp_records.md](exp_records.md) §1。
- **自采 2026-11 move bag（点云上方漂浮"幽灵层"）**：根因是 **bag 内相机话题断流 2.62 s**
  撞上 `sync_packages` 无降级等待——主线程停摆期间视觉更新用过期关联，生成整片漂浮层。
  曾尝试"VIO 更新位移护栏"（`vio.pos_jump_thresh`），实验证伪：整 bag 0 次触发、幽灵层不变，
  临时代码已全部移除。**结论：数据侧问题，只能重录；别再从算法层修这个 bag。**
- **Retail_Street（HKU 公开数据集）**：必须用官方标定 `camera_pinhole.yaml`（fx≈1293.6）+
  官方 `avia.yaml`（`img_time_offset: 0.1`）；bag 自带 `avia_lvi.yaml`（fx≈906、
  `img_time_offset: 1200.25`）是 DeepRobotics 车载配置，误用会导致 RGB 严重错位。
  标定文件与来源注释已归档在 bag 目录 `/home/ywj/slam_data/Retail_Street/`。
- **A/B 单体化后端到端（test4，2026-11-19）**：1x 实时、FastDDS、全 bag 双臂。A 视觉合计
  avg 10.5 ms、`[Img->VisOdom]` 11.4 ms、彩色 PCD 57 MB / 3.69M 点、odom 829 行；
  B 视觉合计 avg 44.0 ms、`[Img->VisOdom]` 83.8 ms、PCD 56 MB / 3.64M 点、odom 502 行。
  详细表与数据分析见 [docs/monolith_refactor.md](../docs/monolith_refactor.md) §8.6 与
  [exp_records.md](exp_records.md) §43。

---

## 4. RK3588 实机对比测试（怎么测才算公平）

平台：RK3588S（A76×4 = 核 4-7，A55×4 = 核 0-3），Ubuntu 22.04 + ROS2 Humble，Release
（`-O3 -march=native`）。完整报告见 [speed_test.md](speed_test.md)。

**1. 绑核（必须与实机 launch 一致，否则数字没有可比性）**

| 进程 | 绑定 |
| --- | --- |
| `ros2 bag play`（顶替 Livox 驱动） | `taskset -c 4` |
| `super_lio_node` | `taskset -c 7` + `lio.single_core:=true` |
| `fastlivo_mapping` | `taskset -c 7` + `OMP_NUM_THREADS=1` |
| `visual_a_app` / `visual_b_app` | `taskset -c 7`（**与 LIO 同挤一个大核**；单体化后视觉支路已在 LIO 同进程内，历史双节点数据见 [speed_test.md](speed_test.md)） |
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

**3. 三个必须注意的坑**

- **B 的实时性数字已过期**：[speed_test.md](speed_test.md) 里 B 视觉 41.8 ms、约束
  22.7±26.6 ms 是**修复选轨 O(n²) 之前**测的（当时单帧最坏 422 ms）。修复后选轨 max 已降到
  2.1 ms，B 是否仍"单大核实时临界"必须重测才能下结论。
- **FL2 在单大核上跑不满实时**：处理需求 ≈0.99 帧周期，长序列必然积压，odom 延迟退化到
  秒级（均值 5.3 s、峰值 20 s）。所以"在 RK3588 上测 FL2"要么放开绑核/降帧频，要么明确
  标注"该组为积压态，延迟指标不代表算法耗时"，否则对比不公平。另外其视觉地图原本常驻累积、
  长序列会 OOM，现已在 fork `b9f931f` 用滚动淘汰修掉。
- **不要只看延迟**：A/B 的价值在退化段，健康段应与 LIO 逐点贴合（`|B−LIO|` p50 ≈ 2~5 cm）。
  建议的判据组合是「延迟分布 + 同 bag 逐点差 + 退化段终点误差多跑分布」三者一起看，
  任何单一指标都能给出误导性结论。

---

## 5. 待办实验

- **B2 退化分段验证**：录新长走廊退化 bag，按 `scripts/` 的方法统计 `[VisRot]` 分段接受率/Δyaw，
  验证 χ² 门控在退化段是否自动开门拉回漂移（配置 B 定版前的最后一步）。
  过程与结论见 [B2B3-光流旋转约束实验记录](B2B3-光流旋转约束实验记录.md)。
- **彩色点云配准（颜色残差 ICP）**：Colored Point Cloud Registration Revisited,
  Park、Zhou、Koltun，ICCV 2017。待测：在点云配准/观测环节引入颜色残差项。
  前置条件：Mid-360 只有反射强度、无 RGB，需先接上着色链路（见 `pointcloud_colorizer`）；
  颜色只对沿墙面等平移退化方向有补充力，**治不了 yaw 漂移**。
- **A 的深度关联排查**：四个 livo bag 上 A 的视觉约束 `n=0`，B 同包 `obs=133–179`，
  需单独核对 A 的外参取值 / 时延 / 坐标系约定。
