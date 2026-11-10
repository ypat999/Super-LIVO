# OctVox 代表点更新策略消融 · 阶段二：完整 LIO 闭环（MAP_UPDATE_CLOSED_LOOP）

- 日期：2026-11-10
- 对应方案：`superlio实验.md` §「阶段二：完整 LIO 闭环」
- 代码版本：`Super-LIVO` @ `1f37053`；`third_party/Super-LIO` @ `67c0acd` + 本地新增 `rep_mode==2` 分支（见 §2.3）
- 数据：`/home/ywj/slam_data/Retail_Street`（原始 bag，1.0× 速率回放）
- 产物：`/home/ywj/slam_data/octvox_closedloop_20261110/`
- 前置文档：[OCTVox_UPDATE_AUDIT.md](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/docs/OCTVox_UPDATE_AUDIT.md)、[MAP_UPDATE_AB_COMPARISON.md](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/docs/MAP_UPDATE_AB_COMPARISON.md)

## 0. 结论速览（含必须先读的限定）

| # | 观察 | 数量级 |
|---|---|---|
| 1 | 三组闭环全部跑满 **1317 帧**，无丢帧、无节点崩溃，地图容量未触发淘汰（`evict=0`） | 系统稳定性三组等价 |
| 2 | 分析窗内（245 帧）地图局部厚度残差 `fit_res_mean` P50：**A 53.40 < B 61.41 < C 67.95 mm**，配对 Wilcoxon `p=6.1e-42` | A 最低，但与 §1 的指标偏置直接相关 |
| 3 | 有效匹配率 `match_ratio` P50：A 0.764、B 0.760、**C 0.691**（C 低 6.7–7.3 pp），C 的平面拟合失败率 30.11% vs A/B 23.4% | C 的有效观测显著变少 |
| 4 | 姿态修正量 `att_corr` P50：0.132 / 0.144 / 0.141°，量级 0.1° 且三组差异 < 0.02° | 无组间可分性 |
| 5 | 轨迹互差（无 GT，仅相对一致性）：B−A P50 28.3 mm、C−A P50 39.0 mm | 远大于组内步长抖动，但在同一量级 |
| 6 | 计算资源：wall 2:18.7±0.2 s、CPU 17%、RSS 223 MB、`MapInsert` 0.074–0.076 ms/帧 | 三组无可测差异 |

> **读法警告（方案要求，必须保留）**：
> 1. `fit_res_mean` 是**地图自洽性**指标（定义见 §3.4），A 的增量均值会**机械地**压低它，因此「A 残差最低」**不能**解释为「A 定位更准」。
> 2. 本次**没有独立真值（GT）**（无动捕/全站仪/RTK，也无更高精度参考轨迹），全部位姿结论只能表述为**相对一致性**与**系统稳定性**，不能表述为绝对定位精度。
> 3. 轨迹更平滑 ≠ 更准确；地图更规整 ≠ 定位更准；残差下降 ≠ 精度提高。

## 1. 事前判读规则

为避免事后挑选指标，先固定判读方式（与阶段一一致）：

- 主指标：`fit_res_mean`（地图局部厚度残差）、`match_ratio`（有效观测比）。
- 显著性：逐帧配对 + Wilcoxon 符号秩检验（`scipy.stats.wilcoxon`，双侧），`p<0.01` 记为「可区分」。
- 效果量：同时报 P50 与 P90 的**差值**（mm 或 pp），不以 p 值单独下结论。
- **不预设新策略更好**：若 B/C 在主指标上劣于 A 且无法用指标偏置解释，则如实报告为「在当前配置下更差」；若不可区分，则报「不可区分」。

## 2. 实验设置

### 2.1 数据与运行

| 项 | 值 |
|---|---|
| bag | `Retail_Street`（Livox Avia `CustomMsg` + IMU），`ros2 bag play --rate 1.0` |
| 回放段 | 全段 134.70 s（`946685438.295` – `946685572.998`） |
| LiDAR 帧 | 1317 帧，平均 9.78 Hz；帧间隔 P50 99.84 ms、P99 99.91 ms |
| 帧间隔空档 | 全段仅 1 处 3.2 s（`t=946685480.999`，bag 级空档，三组完全相同） |
| **分析窗** | **`946685507.0` – `946685531.5`（24.5 s，245 帧，窗内 0 空档）**，与阶段一离线实验同一时间窗 |
| 运行方式 | 三个**独立进程实例**，逐组冷启动（每组前 `pkill` + 2 s 等待），互不共享地图 |

分析窗选在 `946685507.0` 之后，是因为：①与阶段一完全对齐，便于交叉核对；②跳过建图初始化段（前 ~45 s 地图尚在快速扩张）；③避开 3.2 s 空档。全段指标另列（§4.3），仅作补充。

### 2.2 唯一变量与固定项

唯一改动：`lio.hash_map.rep_mode`。

| 组 | `rep_mode` | 语义 | 实现来源 |
|---|---|---|---|
| A | 0 | 增量均值 `μ←(μn+p)/(n+1)` | 原有 |
| B | 2 | 首次观测保留，仅 `++count` | **本次新增**（`OctVoxMap.hpp` L126-130） |
| C | 1 | 距子体素几何中心最近的真实点，`ε=0`（严格 `<` 才替换） | 原有 |

固定不变（方案要求「除代表点更新方式外不改」）：

- 配置文件：`third_party/Super-LIO/src/super_lio/config/r3live.yaml` **原文件**，未编辑；仅用 `-p` 覆盖 `rep_mode` 与日志开关。
- IMU 处理、 Propagation/Undistort、ESKF、残差与权重、KNN（`KNNHeap<8>`）、体素划分（父 0.5 m / 子 0.25 m）、`merge_dist=0.1 m`、`max_points=20`、`clamp_mu=false`、`hash_capacity=2e6` 全部相同。
- 预处理链相同：`filter_rate=3` → `VoxelGridClosest(0.5 m)`（`DownSample`）。**未改动**前级 center-based 降采样（本实验只改跨帧增量平均）。
- 调试开销相同：三组均开 `lio.debug.level=2` 与 `lio.eva.timer=true`，因此相对比较不受日志开销偏置，但**绝对耗时含调试开销**。

### 2.3 新增代码（B 组）

`third_party/Super-LIO/src/super_lio/include/OctVoxMap/OctVoxMap.hpp`，`OctVox::AddPoint` 内、`rep_mode==1` 分支之前：

```cpp
if (g_map_rep_mode == 2) {
  // 首次观测保留：代表点永不变动，只累加计数（实验 B 组）
  ++count;
  return;
}
```

- 位置在「计数上限」与「合并门槛」两条 `return` 之后 ⇒ B 组仍遵守 `n≥20` 拒绝与 `|p−μ|>0.1 m` 拒绝，**不改变准入条件**，只改变「准入之后是否移动代表点」。这是方案要求的「保留全部其他机制」。
- 已 `colcon build --symlink-install` 成功；A/C 路径未改动。

### 2.4 启动命令（对全部组相同，仅 `mode` 不同）

```bash
export ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash   # livox_ros_driver2 CustomMsg 类型支持
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash

/usr/bin/time -v -o $dir/time.txt bash -c \
  "exec ros2 run super_lio super_lio_node --ros-args \
    --params-file .../config/r3live.yaml \
    -p lio.hash_map.rep_mode:=$mode \
    -p lio.debug.level:=2 -p lio.eva.timer:=true \
    -p lio.map.save_map_dir:=$dir" > $dir/node.log 2>&1 &
ros2 bag record -o $dir/bag --qos-profile-overrides-path /tmp/rec_qos.yaml /lio/odom &
ros2 bag play /home/ywj/slam_data/Retail_Street --rate 1.0 --disable-keyboard-controls
```

`/tmp/rec_qos.yaml`：`reliability: best_effort`、`history: keep_all`、`depth: 10000`（避免 `/lio/odom` 被 QoS 丢帧）。

## 3. 指标定义（含单位、样本数、已知偏置）

数据源：`lio.debug.level=2` 写出的 `debug.csv`（45 列，每帧一行，1317 行）与 `/usr/bin/time -v` 的 `time.txt`。统计范围除注明外均为分析窗 245 帧。

| # | 指标 | 定义 | 单位 | 样本数 | 已知偏置 / 说明 |
|---|---|---|---|---|---|
| 3.1 | `path_len` | 逐帧位置 `p_k`（`px,py,pz`）相邻距离之和 `Σ‖p_k−p_{k−1}‖` | m | 244 段 | 累计量，与轨迹抖动正相关；不是精度 |
| 3.2 | `step_p50/p90/max` | 相邻帧位移 `‖Δp‖` 的分位数 | mm | 244 | 受速度和回放速率影响，三组面对同一运动 |
| 3.3 | `frame_dt` / `n_gaps` | `t` 差分；`>1.6×median(frame_dt)` 记为丢帧 | ms / 个 | 244 | 窗内 0，三组同为 99.84 ms（≈10 Hz） |
| 3.4 | `fit_res_mean` | **首轮迭代**中所有「平面拟合成功」点，其 8 邻域拟合平面的**最大邻域内点面偏差**（`calc_plane_coeff` 的 `max_abs_dist`）之均值（`super_lio.cpp` L1786/L2141-2144） | mm | 245 帧 × 每帧数千点 | **地图自洽性**，不是位姿误差。A 的均值把点拉向邻域中心 ⇒ 机械压低该值；C/B 保留真实点 ⇒ 保留观测噪声与厚表面。**跨组比较此指标存在方向性偏置** |
| 3.5 | `match_ratio` | `n_effect / ptsize`；`ptsize` 为本帧送入 `Observe` 的降采样点数，`n_effect` 为通过 KNN(<4 邻居判失败)+平面拟合+残差门的点数 | 无量纲 | 245 | 与地图稀疏度/规则度强相关；C 的代表点更靠子体素中心 ⇒ 邻域更「散」⇒ 更易拟合失败（见 §5.2） |
| 3.6 | `knn_fail_frac` | `n_knn_fail/ptsize`（邻域点 <4） | % | 245 | 度量地图覆盖率，三组应几乎相同 |
| 3.7 | `plane_fail_frac` | `n_plane_fail/ptsize`（`calc_plane_coeff` 返回 false：法向退化或邻域最大偏差 > `plane_fit_threshold=0.15 m`） | % | 245 | 与 3.4 同源，A 亦受同方向偏置 |
| 3.8 | `att_corr` | 后验与预测的旋转差 `‖rv−rv_pred‖`（度） | ° | 245 | ESKF 更新量，非误差；小值可能来自「观测弱」而非「一致」 |
| 3.9 | `dx_inf` | ESKF 最后一次迭代步长 `‖dx‖_∞`（`ESKF.cpp` L379） | 混排（rad 与 m） | 245 | 量纲混排，只能组间相对比较 |
| 3.10 | `deg_cond` / `deg_axes` | 观测 Hessian 退化签名的条件数与退化轴数 | – / 个 | 245 | 由同一 `Observe` 点集算出，见 `computeDegeneracy` |
| 3.11 | 资源 | `time.txt`：wall、user、sys、CPU%、max RSS | s / % / MB | 1 次/组 | 单次运行，无重复；CPU 17% 说明进程未跑满单核，区分度有限 |
| 3.12 | 阶段耗时 | `timer.h` 周期性报告：`DeltaCorrect` `DownSample` `MapInsert` `UpdateMap` `Observe` `Undistort` `MapTransform` | ms/次 | 659/660/660 次 | 含调试埋点开销（三组相同）；`[Lidar->Odom]` 含等待，不作计算耗时代理 |
| 3.13 | 轨迹互差 | `‖p_A(t) − p_k(t)‖`，`t` 取 A 的时间戳，B/C 位置三轴线性插值到 `t` | mm | 245 | **无 GT**：互差只说「不一样」，不说「谁对」 |
| 3.14 | 配对检定 | 逐帧 `Δfit_res = fit_res_k − fit_res_A`、`Δmatch = match_k − match_A` | mm / – | 245 | 最近时间戳对齐；只对 `fit_res>0` 的帧配对 |

## 4. 结果

### 4.1 主表（分析窗 245 帧）

| 指标 | A（均值） | B（首次） | C（最近中心） |
|---|---|---|---|
| `path_len` (m) | **10.370** | 10.420 | 10.379 |
| `step_p50` (mm) | 51.06 | 52.33 | 51.37 |
| `step_p90` (mm) | 70.27 | 70.43 | 71.48 |
| `step_max` (mm) | 84.75 | 91.49 | 92.59 |
| 帧间隔 P50/P99 (ms) | 99.84 / 99.91 | 99.84 / 99.91 | 99.84 / 99.91 |
| 丢帧数 | 0 | 0 | 0 |
| `fit_res_mean` P50 (mm) | **53.40** | 61.41 | 67.95 |
| `fit_res_mean` P90 (mm) | 64.55 | 69.51 | 78.93 |
| `match_ratio` P50 | **0.764** | 0.760 | 0.691 |
| `knn_fail_frac` (%) | 0.344 | 0.345 | 0.340 |
| `plane_fail_frac` (%) | 23.39 | 23.43 | **30.11** |
| `att_corr` P50 (°) | 0.132 | 0.144 | 0.141 |
| `att_corr` P90 (°) | 0.248 | 0.273 | 0.243 |
| `dx_inf` P50 | 1.93e-4 | 2.34e-4 | 1.53e-4 |
| `deg_cond` P50 | 741.7 | 765.3 | 783.9 |
| `deg_axes` 均值 | 2.392 | 2.376 | 2.400 |
| wall | 2:18.74 | 2:18.98 | 2:18.64 |
| CPU% | 17 | 17 | 17 |
| max RSS (MB) | 223.28 | 223.28 | 222.85 |
| user 时间 (s) | 14.78 | 18.07 | 18.79 |

### 4.2 配对检定（基线 A，同一帧）

| 比较 | `Δfit_res` P50 | `Δfit_res` P90 | Wilcoxon p | `Δmatch` P50 | Wilcoxon p |
|---|---|---|---|---|---|
| B − A | **+7.07 mm** | +9.93 mm | 6.1e-42 | −0.0019 | 0.536（不可区分） |
| C − A | **+13.93 mm** | +16.83 mm | 6.1e-42 | **−0.0700** | 6.1e-42 |

结论（窗内）：

1. **B 与 C 的地图局部厚度残差都显著高于 A**，且 C 的劣化幅度约为 B 的 2 倍。
2. **B 的匹配率与 A 统计不可区分**（p=0.54），而 **C 的匹配率显著低 7.0 pp**。
3. 姿态修正量（`att_corr`）三组差异小且不随上述方向单调变化 ⇒ **不能从 `fit_res` 推出姿态精度差异**。

### 4.3 全段补充（1317 帧，含初始化与收尾）

| 指标 | A | B | C |
|---|---|---|---|
| `path_len` (m) | 66.943 | 67.126 | 66.798 |
| `step_p50` (mm) | 54.78 | 54.94 | 55.30 |
| `step_max` (mm) | 2352.8 | 2331.9 | 2317.6 |
| `fit_res_mean` P50 / P90 (mm) | 57.16 / 68.97 | 61.40 / 71.87 | 67.93 / 81.99 |
| `match_ratio` P50 | 0.826 | 0.823 | 0.773 |
| `plane_fail_frac` (%) | 17.15 | 17.13 | 21.68 |
| `att_corr` P50 (°) | 0.122 | 0.123 | 0.124 |
| `Δfit_res` vs A (P50 / p) | — | +2.92 mm / 1.0e-210 | +11.15 mm / 6.8e-217 |
| `Δmatch` vs A (P50 / p) | — | −3.4e-4 / 0.706 | −0.0404 / 2.2e-211 |

- 全段 `step_max ≈ 2.3 m` **不是异常**：来自三组共有的 3.2 s bag 空档（`t=946685480.999`），三组相同，不能作为稳定性差异。
- 全段方向与窗内一致（B/C 的 `fit_res` 均高于 A；C 更差；B 的匹配率仍不可区分），说明结论不只出现在某个窗口。

### 4.4 轨迹互差（无 GT，仅相对一致性）

| 比较 | P50 (mm) | P90 (mm) | max (mm) | 末帧 (mm) |
|---|---|---|---|---|
| 窗内 B − A | 28.3 | 35.9 | 41.9 | 23.1 |
| 窗内 C − A | 39.0 | 46.3 | 50.9 | 42.6 |
| 全段 B − A | 17.0 | 30.0 | 43.1 | 11.9 |
| 全段 C − A | 25.7 | 47.5 | 59.0 | 6.87 |

- 三组在同一 bag、同一运动、同一参数下，24.5 s 内末端就已分开 2–4 cm；**这足以说明代表点策略对轨迹有可观影响**。
- **但方向不可判**：没有 GT，无法说 A 的轨迹更接近真值。全段末帧 C−A 只有 6.87 mm（小于窗内 42.6 mm），说明差异是**局部累积后又部分抵消**的，不是单调漂移 —— 这既可能是误差抵消，也可能是两段都偏离真值。
- 互差量级（P50 28–39 mm）显著大于步长抖动（P50 51 mm 的相对变化仅 1–2%），但与阶段一留出残差差（B/A 差 ~7 mm、C/A 差 ~14 mm）同向：**C 的偏离更大**。

### 4.5 计算资源与阶段耗时

`timer.h` 累计报告（calls 659/660/660，即全段）：

| 阶段 | A | B | C |
|---|---|---|---|
| `DeltaCorrect` (ms) | 0.0375 | 0.0363 | 0.0367 |
| `DownSample` (ms) | 0.204 | 0.204 | 0.204 |
| `MapInsert` (ms) | 0.0737 | 0.0738 | 0.0757 |
| `UpdateMap` (ms) | 0.0766 | 0.0768 | 0.0788 |
| `Observe` (ms) | 4.245 | 4.308 | 4.284 |
| `Undistort` (ms) | 0.218 | 0.218 | 0.219 |
| `MapTransform` (ms) | 0.00124 | 0.00125 | 0.00133 |
| `Lidar->Odom` (ms，含等待) | 14.14 | 9.68 | 9.35 |
| `MapStats` size / new_voxel / evict | 12258 / 5065 / 0 | 12286 / 5138 / 0 | 12282 / 5074 / 0 |

- `MapInsert` 三组差 < 2 μs/帧，**代表点策略的计算代价可忽略**（B 分支只多一次 `++count`）。
- `Observe` 的 40 μs 级差异（< 1%）与匹配点集大小相关，不构成性能门槛。
- `Lidar->Odom` 含「等待 IMU 传播对齐」的时间，不是纯计算耗时，且 A 明显更大 —— 这反映的是**调度/等待**差异，本报告不据此下性能结论。
- `MapStats size` 是父体素数（~12.3 k），三组差 < 0.3%，`evict=0` ⇒ 容量淘汰全程未触发（与审计 §7 预测一致）。
- `user` 时间 A 14.78 s vs B/C 18.1–18.8 s，`sys` 相反（10.08 vs 6.6/5.9 s），wall 几乎相同 ⇒ **总 CPU 占用无实质差异**，差异来自 user/sys 划分，不作为结论。

## 5. 结果解读

### 5.1 为什么 A 的 `fit_res` 最低（且这不是优势证据）

`fit_res_mean` 只统计「平面拟合成功」点邻域内的最大偏差。A 的代表点是同一子体素内多个观测的**均值**，其位置天然接近邻域中心，因而「邻域最大偏差」被机械压低；B 保留首个观测、C 保留某个真实观测，两者都保留了单次观测的测距噪声与表面厚度。这与阶段一 M1/M2 的结论方向**完全一致**（阶段一：A 的留出残差最小，但 `off5cm` 是 B/C 的 3.6 倍，0.603% vs 0.168%/0.193%）。

⇒ **A 的「残差最低」与「表面最不准」同时成立**，只能说明该指标对 A 有方向性偏置。方案原文「不能简单把中心距离变小解释成几何精度提高」在此处的对应表述是：**不能把残差变小解释成地图更准或定位更准**。

### 5.2 C 的匹配率为何下降（与阶段一可交叉验证）

C 的代表点被约束到「离子体素几何中心最近」，因此更靠近 0.25 m 立方体的中心，而不一定靠近真实表面。后果：

- 8 邻域（`KNNHeap<8>`，来自 8 个父体素各 1 个子体素代表点）构成的局部点集更「厚」、法向更不稳定；
- `calc_plane_coeff` 的 `max_abs_dist > 0.15 m` 判据更易触发 ⇒ `plane_fail_frac` 23.4% → 30.1%，`match_ratio` 0.764 → 0.691；
- 这与阶段一 §5.4 的定量结果吻合：C 的「到子体素中心距离」中位数下降 23 mm，但「沿局部法向的位移」P90 由 24.9 mm **增大**到 40.8 mm（B→C）。

⇒ C 减少了「中心距离」，却增大了「沿表面法向的偏移」，闭环中表现为有效观测变少、残差变大。**这正是方案要求检出的空间选择偏差**，且两个阶段给出一致的方向。

### 5.3 为什么 `att_corr` 差异很小

`att_corr` 是 ESKF 每帧的姿态修正量，量级 0.1°。三组差异 < 0.02°，且不随 `fit_res` 单调（B 最大 0.144°，C 居中 0.141°，A 最小）。说明：

- 在 24.5 s、~10 m 的短窗内，IMU 预测占主导，地图差异只体现为**小幅修正**；
- 地图质量的差异**没有**线性转化为姿态修正量差异 ⇒ 不能据此判断谁更准。

### 5.4 与阶段一的对应关系

| 阶段一（离线几何，时间分块留出） | 阶段二（闭环） | 是否同向 |
|---|---|---|
| B 的留出残差显著劣于 A（p=3.0e-10 / 1.2e-14） | B 的 `fit_res` 显著高于 A（p=6.1e-42），匹配率不可区分 | 同向 |
| C 与 A 在留出残差上**统计不可区分**（p=0.644 / 0.575） | C 的 `fit_res` 显著高于 A，匹配率显著更低 | **不完全同向**（阶段一不可区分 → 闭环可区分） |
| A 的 `off5cm` 是 B/C 的 3.6 倍 | A 的 `fit_res` 最低 | 同向（均说明 A「规整但偏表面」） |
| C 的顺序敏感性低于 B（反序/随机下代表点变化 >1 mm 的比例：C 40.2%/31.9%，B 77.2%/63.7%） | C 无崩溃、无丢帧，`deg_cond` 仅高 5% | 同向（都稳定） |

不一致点必须如实说明：阶段一的 M2 用「时间分块留出 + A/B/C 配对交集」，闭环的 `fit_res` 是「单帧对当前地图的邻域拟合质量」，**两者不是同一个量**，且闭环指标对 A 有偏置。因此「阶段一 C≈A」与「闭环 C 劣于 A」并不矛盾，但也**不能**合并成一个更强的结论。

## 6. 限制（结论适用边界）

1. **无独立 GT**：所有位姿结论限于「相对一致性」与「系统稳定性」。轨迹互差只说明策略有影响，不说明谁更接近真实位姿。
2. **单序列、单次运行**：`Retail_Street` 一段，三组各 1 次；未做多次重复，因此不能给置信区间，只能给配对帧级统计（帧间不独立，p 值应视为「方向一致的强证据」而非严格概率）。
3. **指标偏置**：`fit_res_mean`、`plane_fail_frac`、`match_ratio` 三者同源（都来自 8 邻域拟合），且对 A 有利；三者相互印证不能消除该偏置。
4. **`merge_dist` 与代表点的耦合未解耦**：`|p−stored|>0.1 m` 的准入以 `stored` 为参照，A/B/C 的 `stored` 不同 ⇒ 三组「被接纳的观测集」不同。方案明确要求保留该机制，因此这是策略差异的一部分，但意味着**无法把差异单独归因于「代表点是否平均」**。
5. **调试开销**：三组均开 `lio.debug.level=2`，绝对耗时偏大；相对比较有效，绝对值不外推。
6. **回放速率 1.0×，CPU 17%**：进程未跑满单核，实时性余量大；本结果不代表高负载/多线程场景。
7. **3.2 s bag 空档**：全段指标受其影响（`step_max≈2.3 m`），窗内指标不受影响。
8. **未做彩色点云、未改 ESKF/残差/KNN/体素划分**（方案要求），因此不能回答「换匹配损失后结论是否改变」。
9. **D 组（全量观测）不参与闭环**：`OctVox` 每子体素只有 1 个代表点槽位，结构上不支持；D 仅在阶段一离线评价。

## 7. 复现与产物清单

### 7.1 复现

```bash
# 1) 编译（含 B 组 rep_mode=2 分支）
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO && colcon build --symlink-install

# 2) 三组闭环（约 7 min，含每组冷启动与 2 s 间隔）
bash /home/ywj/git/Super-LIVO/experiments/octvox_ablation/05_closed_loop.sh

# 3) 指标（阶段一同一时间窗，245 帧）
python3 /home/ywj/git/Super-LIVO/experiments/octvox_ablation/06_analyze_closedloop.py
# 4) 全段补充（1317 帧）
python3 /home/ywj/git/Super-LIVO/experiments/octvox_ablation/06_analyze_closedloop.py --full
```

### 7.2 产物（`/home/ywj/slam_data/octvox_closedloop_20261110/`）

```
rep_A/  rep_B/  rep_C/              每组一个独立运行目录
  ├── debug.csv                     45 列 × 1317 行（含表头），逐帧位姿/残差/匹配/退化签名
  ├── time.txt                      /usr/bin/time -v（wall/user/sys/CPU/RSS）
  ├── node.log                      节点日志（含 timer.h 阶段耗时、MapStats）
  ├── play.log  rec.log             bag 回放/录制日志
  └── bag/                          /lio/odom 录制（三组各自轨迹）
closedloop_metrics.csv              每组一行（窗内 245 帧）
closedloop_per_frame.csv            逐帧三组残差/匹配/姿态修正 + 三组位置
closedloop_summary.json             窗内汇总 + 配对检定 + 轨迹互差
closedloop_metrics_full.csv         全段 1317 帧版本
closedloop_per_frame_full.csv
closedloop_summary_full.json
```

### 7.3 脚本

- [05_closed_loop.sh](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/experiments/octvox_ablation/05_closed_loop.sh)：三组顺序运行、存活检查、odom 录制
- [06_analyze_closedloop.py](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/experiments/octvox_ablation/06_analyze_closedloop.py)：`debug.csv` 解析、指标、配对 Wilcoxon、轨迹互差；`--full` 切全段

### 7.4 代码改动

- [OctVoxMap.hpp](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/include/OctVoxMap/OctVoxMap.hpp#L126-L130)：新增 `rep_mode==2`（B 组），仅 4 行。
- 未改动：`super_lio.cpp`、`ESKF.*`、`VoxelGridFilter.h`、`r3live.yaml`。
