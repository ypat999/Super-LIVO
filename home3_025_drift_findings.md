# home3 / 0.25 体素漂移定位（阶段小结）

> 记录时间：2026-11-25
> 分支：main（Super-LIVO `third_party/Super-LIO`），对照：no-de-check（`home3_v7_025`）
> 目标：home3 全段 941.3 s 内不出现 `|v|>2 m/s`、`z<-5 m`，无清图重锚级联，终点在 (±15, ±15, 0~5)；且不回归 `gicp_aw`。

---

## 1. 结论摘要（TL;DR）

1. **漂移不是"观测错但自信"**。触发时刻的观测完全健康：`n_effect=417`、`n_plane_fail=0`、`fit_res_p95≈0.10`、`cond_yk≈313`、`deg_axes=0`、平移退化 `cond≈3`。平面拟合、内点数、残差、退化签名都没有异常。
2. **真正的时间线是两段**：
   - **A 段（515–555 s）**：main 与 no-de-check 开始系统性分离（`dpos>0.1 m @515.5`，`>1.0 m @546.7`；yaw 差 545 s 4.6° → 550 s 9.0°），期间**没有任何 Trust 干预**，观测指标健康。这是"估计器层面的既有差异"，不是守护机制造成的。
   - **B 段（555.9–557 s）**：`新点比例 + 状态速度` 的"早期前兆"判据命中（diag580: new=0.187 / |v|=1.49；early17: new=0.201 / |v|=1.79），进入 3 s 冻结抑制；GICP 无法做绝对校验（给出 1.2 m / 22° 的修正、inlier 0.40，被拒绝），抑制结束时扫描已出冻结图覆盖（new 0.6–0.7），放行后 `|v|` 被泵到 5 m/s，随后硬重锚 → 级联下沉。
3. **把早期门限从 1.2 提到 1.7 只是把触发从 555.9 s 推迟到 556.8 s，没有解决发散**（|v|>2 从 560.2 s→561.4 s，终点仍 z=-523）。
4. **根因已确认到具体参数：`intensity_filter_en=true`**。no-de-check 用的 `filter_rate=3` + `intensity_filter_en=false`，main 用 `filter_rate=6` + 过滤低反射点（详见 §6）。
   - `home3_early17_in3`（main 全部代码/守护机制 + 3/不过滤）：941 s **与 no-de-check 逐位相同**（9400 帧 max dpos = 0），无漂移，Trust 仅 7 条开阔区门控。
   - `home3_early17_noint`（main + 仅关强度过滤、保留 `filter_rate=6`）：**无 `|v|>2`、无 `z<-5`**，终点 (-3.73, -6.18, 0.69)，与 no-de-check 最大差 0.117 m。
   - 即守护机制本身没有问题（注：此处“无问题”仅指“健康输入下惰性、不误伤”；守护机制的放行路径本身是发散源，见 §11）；是强度过滤让有效点从 787 掉到 562（-29%），稀疏输入在走廊快速转向段产生 1 m/9° 系统偏差，守护机制再把已偏轨迹判成"前兆"并放大成发散。
5. **终态配置已定为** `intensity_filter_en=false` + `trust_vel_early=1.2`（恢复原灵敏度：关掉过滤后 home3 全程 98 帧 `|v|>1.2`，但新点比例无一超过 0.18，判据不再误触发）。**终态验证已全部通过**：
   - `home3_final`（941 s）：无 `|v|>2`、无 `z<-5`，`max|v|=1.54`，终点 (-3.73, -6.18, 0.69)，与 no-de-check 一致；Trust 14 条全为开阔区门控。
   - `gicp_aw_fix`（435 s）：无 `z<-5`，`max|v|=2.56`（基线 2.90），`|v|>2` 首次 278.7 s（基线 280.8 s）；与基线在 272–300 s 的重定位事件处产生一次 **0.89 m 的常数偏移**（峰值 4.12 m 是瞬态），400 s 后稳定在 0.85–0.89 m 且不再增长 → **判为可接受的恢复路径差异，回归通过**（详见 §9）。

---

## 2. 复现环境

```bash
# 编译（约 1–3 min）
bash /mnt/c/Users/ypat/AppData/Local/Temp/build_lio.sh   # 成功标志 BUILD_RC=0

# 单次回放（DURATION>0 只播前 N 秒；DIAG=1 注入 debug.csv 埋点）
export RUN=/home/ywj/slam_data/lio_runs/<run_name>
export BAG=/home/ywj/slam_data/lio_bag/home/build_map_record_home_3
export DURATION=941 SAVE=0 ROS_DOMAIN_ID=140 USE_FASTDDS=1 DIAG=1
bash /home/ywj/slam_data/lio_runs/home_test/run_home_nd.sh
```

产物：`$RUN/odom.csv`（10 Hz 位姿/速度）、`$RUN/debug.csv`（逐雷达帧观测指标）、`$RUN/node.log`、`$RUN/params.yaml`。

> 注意：WSL 前台退出会杀掉 `nohup ... &` 起的回放，必须用工具的 background 模式让整条命令阻塞执行。

## 3. 跑批结果对照

| 跑批 | 差异 | 首次 \|v\|>2 | 首次 z<-5 | 终点 (x,y,z) |
|---|---|---|---|---|
| `home3_v7_025` | no-de-check，filter_rate=3、不过滤强度 | 无 | 无 | (-3.73, -6.18, 0.70) |
| `home3_openguard` | main + 开阔区门控 | 558.7 s | 573.5 s | (-109.14, 77.21, -437.86) |
| `home3_diag580` | 同 openguard，DIAG=1（位姿逐帧一致） | 560.2 s | 581.7 s | (-93.38, 6.19, -600.61) |
| `home3_early17` | `trust_vel_early` 1.2→1.7，全段 | 561.4 s | 591.6 s | (107.11, -143.04, -523.15) |
| `home3_early17_in3` | **main 代码 + `filter_rate=3` + `intensity_filter=false`** | 无 | 无 | (-3.73, -6.18, 0.70) |
| `home3_early17_noint` | **main 代码 + 仅 `intensity_filter=false`** | 无 | 无 | (-3.73, -6.18, 0.69) |
| `gicp_aw_fix` | 终态配置（intensity=false, trust_vel_early=1.2），home 435 s 回归 | 278.7 s（基线 280.8 s） | 无 | (-1.54, -1.87, 5.32) |
| `home3_final` | 终态配置，home3 941 s 验收 | 无 | 无 | (-3.73, -6.18, 0.69) |

`home3_early17_in3` 与 `home3_v7_025` 的 odom **逐位相同**（9400 帧 max dpos=0），说明 main 的 Trust/Anchor/退化检测在输入足够稠密时完全惰性、不影响结果。`home3_early17_noint` 证明**只关强度过滤就够了**，`filter_rate` 保持 6 不会漂。

`home3_diag580` 与 `home3_openguard` 在 0–555 s 逐位一致，因此 diag580 的 `debug.csv` 可直接代表 main 的观测层。

## 4. 关键证据（debug.csv，逐帧）

触发帧（diag580，bag 时间 **555.9 s**，由 `map_novelty_ratio=0.186551` 与日志数值精确对齐）：

```
tr=555.90  nov=0.187  eff=417  plane_fail=0  p95=0.1005  deg_cond_t=3.1  deg_axes=0
           |v|=1.44  p=(3.24, -3.69, 2.76)  frame_dt=0.094
```

- 全段 941 s 中 `|v|>1.2` 只有 **8 帧**，且全部集中在 555.4–555.9 s（峰值 1.44 m/s），novelty 同时抬到 0.17–0.21 —— 这是走廊快速转向时的正常加速与新覆盖，不是漂移签名。
- 535–555 s 期间 `n_effect` 400–700、`n_plane_fail` 0–2、`dx_inf` 1e-4~1e-3、`cond_yk` 313–2300、`deg_axes` 全 0。

与 no-de-check 的分离（两跑批时间戳逐行相同，`dt=0`，排除采样错位）：

| 门限 | 首次达到（diag580 与 early17 完全一致） |
|---|---|
| dpos > 0.05 m | 334.7 s |
| dpos > 0.10 m | **515.5 s** |
| dpos > 0.20 m | 517.7 s |
| dpos > 0.30 m | 525.3 s |
| dpos > 0.50 m | 541.2 s |
| dpos > 1.00 m | **546.7 s** |

yaw 差：542.5 s 0.46° → 545 s 4.58° → 550 s 9.04° → 555 s 31.8°。

## 5. 触发后的失败链（日志）

```
[Trust] 早期前兆(新点比例+速度) 判据成立（new=0.186551, |v|=1.48783, 连续 2 帧,
        累计修正 0.118 m / 4.43 deg, effect_pts=417）
[GICP] 拒绝 dpos=1.19772 drot=22.1186 inlier=317/796 rmse=0.162333  （连续多次）
[Trust] 持续抑制 3 s 未重定位（new=0.46~0.56）：保留地图、恢复 Voxmap 观测
[Trust] 状态速度 5.05 m/s 超门限：钳到 1 m/s
[Trust] 平移锯齿 / 20 s 内第 3 次保图放行 → 硬重锚
```

根因是"判据误触发 → 抑制期钉死位姿 3 s（此时机身在快速转向/前飞）→ 扫描必然移出冻结图覆盖 → 放行时 Voxmap 观测把速度泵到 5 m/s"。

## 6. 对照污染：no-de-check 与 main 的 yaml 并不一致

`params.yaml` 逐项对比（`home3_v7_025` vs `home3_diag580`）：

| 参数 | no-de-check | main | 影响 |
|---|---|---|---|
| `lio.sensor.filter_rate` | **3** | **6** | 输入点密度 ×2 |
| `lio.sensor.intensity_filter_en` | **false** | **true** | 滤掉低反射点（暗走廊） |
| `lio.single_core` | false | true | 仅并行/确定性 |
| `lio.hash_map.hash_capacity` | 20 000 000 | 2 000 000 | 仅内存 |
| `lio.observe.lost_recover_frames` | 5 | 10 | 仅 LOST 恢复 |
| Trust / Anchor / 退化检测 | 无（分支无此机制） | 有 | 本轮主角 |

**结论（已由单因子实验证实）**：此前"A/B 参数一致"的前提不成立，真正的差异是**强度过滤**。只关 `intensity_filter_en`（保留 main 的 `filter_rate=6`）后，941 s 全程无漂移、终点与 no-de-check 一致；把 `filter_rate` 也改成 3 则轨迹与 no-de-check **逐位相同（9400 帧 max dpos=0）**。守护机制只是在稀疏输入下把已偏轨迹判成"前兆"并放大成发散。

有效点数对比（30–560 s 中位数）：main 562 / 关过滤后 787（+40%），p90 927 / 1649。

## 7. 已尝试并回退的改动

| 尝试 | 结果 | 处理 |
|---|---|---|
| 取消抑制期钉死位姿（仅 IMU 传播） | 更差：`|v|>2 @557.3`、`z<-5 @575.9`、终点 z=-795；拒绝观测 2.4 s 内 0.81→3.91 m/s | 已回退为 `RollbackToState` |
| `lio.observe.rot_weight = 4.0`（方案 A） | 明显更差：510 s 0.387 m / 3.36°，530 s 14 m；说明该段旋转观测"错但自信" | yaml 改回 1.0，代码保留（1.0 时与原行为逐位一致） |
| `trust_vel_early` 1.2→1.7 | 仅推迟触发（555.9→556.8 s），仍发散；根因修复后已无必要 | **已回退 1.2**（关过滤后全程 98 帧 `|v|>1.2` 但新点比例无一 >0.18，不再误触发） |

## 8. 当前代码/配置状态（main）

- `third_party/Super-LIO/src/super_lio/src/lio/super_lio.cpp`
  - L2090 开阔区门控；L2164 早期前兆触发日志；L1976 抑制超时（含 `moving_release` / `coverage_limited` 分支）；L2444 状态速度钳位。
  - L3154 附近 rot_weight 通道缩放（默认 1.0，等价原行为）。
- `config/livox_360_drone.yaml`
  - **`lio.sensor.intensity_filter_en: false`（本轮唯一行为改动，附证据注释）**；`filter_rate` 保持 6。
  - `vox_resolution 0.25`、`plane_knn_k 5`、`kf_max_iterations 8`、`huber 0.20`、`imu_na 0.01`、`normal_weight_en false`。
  - 退化检测：`threshold 60`、`degen_trans_only 1`、`hysteresis 1.5`、`enter 10`、`exit 20`、**truncate false**。
  - 开阔区门控：`open_guard_max_range 20`、`open_guard_z_span 25`。
  - Trust：`map_novelty_max 0.25`、`streak 3`、`map_novelty_early 0.18`、**`trust_vel_early 1.2`（已回退原值）**、`novelty_early_streak 2`。
  - `lio.observe.rot_weight: 1.0`。

## 9. 最终验证结论

1. **根因验证（决定性）**：`home3_early17_in3`（main 代码 + `filter_rate=3` + 不过滤）与 no-de-check **逐位相同**（9400 帧 max dpos=0）；`home3_early17_noint`（main + 仅关强度过滤，保留 `filter_rate=6`）941 s 无漂移（终点 (-3.73,-6.18,0.69)，与 no-de-check 最大差 0.117 m）。守护机制本身无问题。（注：该结论的精确边界见 §11.4——“无问题”仅指健康输入下惰性、不误伤，不代表守护机制的放行路径不会发散。）

2. **home3 全段验收（通过）**：`home3_final`（`intensity_filter_en=false` + `trust_vel_early=1.2`，941 s）
   - 无 `|v|>2`、无 `z<-5`、无清图重锚级联；
   - `max|v|=1.54 m/s`；
   - 终点 (-3.73, -6.18, 0.69)，落在 (±15, ±15, 0~5) 内；
   - Trust 仅 14 条，全部为开阔区门控（无前兆触发）。

3. **gicp_aw 回归（通过，判定说明）**：`gicp_aw_fix` vs 基线 `gicp_aw`（同为 435 s / 4472 帧）
   - `max|v|` 2.56（基线 2.90），`|v|>2` 首次 278.7 s（基线 280.8 s），无 `z<-5`；
   - 逐帧 dpos：0–272 s 均 ≤0.03 m；272–300 s 在基线自身的 GICP 重定位事件处出现瞬态分离（峰值 4.12 m @279.7 s）；
   - 300 s 后回落并收敛：300–435 s mean 0.99 / max 1.45；400 s 后稳定在 **0.850–0.891 m**，末帧 0.891 m，**不再增长**；
   - 即两条轨迹形状一致，仅因重定位落点不同产生一个**常数平移偏移**，不存在持续发散；终点分别 (-1.54, -1.87, 5.32) / (-1.66, -0.99, 5.26)，差 0.89 m。
   - 结论：属于恢复路径差异，非漂移回归。

4. 收尾事项（待用户决定）：清理 `home3_early17*`、`home3_diag580` 等实验产物；是否将 `intensity_filter_en: false` 的 yaml 改动整理为 git 提交。

## 10. 数据与脚本位置

- 跑批目录：`/home/ywj/slam_data/lio_runs/{home3_v7_025, home3_openguard, home3_diag580, home3_early17, home3_early17_in3, home3_early17_noint, gicp_aw, gicp_aw_fix, home3_final}`
- 实验脚本（Windows Temp，WSL 可直接 bash）：
  - `C:\Users\ypat\AppData\Local\Temp\exp_h3_early17.sh`
  - `C:\Users\ypat\AppData\Local\Temp\exp_h3_in3.sh`
  - `C:\Users\ypat\AppData\Local\Temp\exp_h3_noint.sh`
  - `C:\Users\ypat\AppData\Local\Temp\exp_final.sh`（gicp_aw_fix + home3_final 顺序执行）
- 分析脚本（Temp）：`ana_diag.py`、`cmp_abs.py`、`cmp3.py`、`diverge_stats.py`、`nov_stats.py`、`dbg_early17.py`、`find_trig.py`、`stats_in3.py`、`verify_in3.py`、`pts_cmp.py`
- debug.csv 关键列：`t, ptsize, n_knn_fail, n_plane_fail, n_res_fail, n_effect, fit_res_mean, fit_res_p95, iters, dx_inf, cond_yk, htvh_ev_min/max, px..rvz, frame_dt, deg_cond, deg_axes, deg_cond_t, deg_evt_min, deg_dir_*`
- 日志墙钟→bag 时间标定技巧：用唯一事件数值（如 `map_novelty_ratio=0.186551`）在 debug.csv 中定位真实 bag 时刻，再与 node.log 同一行数值对齐；直接按墙钟差换算会差 3–14 s（回放速度与启动抖动所致）。

---

## 11. 方向反思：反应式守护 vs 预防式降权（对照清单）

> 记录时间：2026-11-26，在 §1–§10 结论之后的二次审视。
> 触发问题：home3 失败链（§5）暴露的不是“某个参数错了”，而是 Trust 状态机这一类**反应式守护**的结构性缺陷——它问的是“这个结果**用不用**”（二值否决），手段是硬切换（否决/冻结/切源/重锚），放行路径（冻结→放行→泵速）本身就能制造发散。代码里其实已存在一整套**预防式降权**机制，只是默认关闭、被 Trust 盖住。

### 11.1 对立本质

| | Trust（反应式） | 预防式（已有，默认关） |
|---|---|---|
| 回答的问题 | 这个结果**用不用** | 这个约束**信几成、在哪个方向信** |
| 手段 | 否决观测 / 冻结位姿 / 切换观测源 / 重锚地图 | 逐点/逐方向降权、阻尼、切 na |
| 特性 | 二值、硬切换、有瞬态 | 连续、逐帧、无切换 |
| 发散风险 | 放行瞬态亲手制造发散 | 阻尼只让更新变小，天然不发散 |

### 11.2 逐动作对照（super_lio.cpp 行号）

**A. 否决观测类**（rollback_damped / 单帧跳变门 / 姿态限幅）

- Trust 现状：单帧跳变门 `L2403-L2505`（`jump>1m`/`>25°` 整帧 rollback）、姿态限幅 `L2251-L2267`。
- 问题：二值，阈值附近抖动，整帧否决丢信息。
- 软替代（已有）：Huber + 距离自适应 `L2905-L2931`、法向质量加权 `L2887-L2890`、rot_weight `L3154-L3162`、beam_err/dept_err 噪声建模。
- 差距：Huber 线性尾部对极端外点不够狠，可换 Cauchy/Geman-McClure；“单帧跳变”仍需帧级强核或 gating，但不要用“整帧否决”二值动作。

**B. 冻结/持续抑制类**（novelty_hold_ + 3 s 抑制 + GICP 重试）

- Trust 现状：判据评估 `L2067-L2172`（novelty+速度前兆）→ 冻结 3 s → 持续抑制 `L1826-L2065`。
- 问题：触发的是**症状**（新点比例+速度），不是**原因**（几何退化）；冻结→放行瞬态亲手制造发散（正是 home3 失败链 §5）。
- 软替代（已有）：退化检测 `L3182-L3212`（cond 触发）+ MotionTrunc 运动轴对齐截断 `L3120-L3152`——匹配不可信时在该方向少信雷达多信 IMU，而不是冻结。
- 差距：把触发源从“novelty+速度”换成“cond/特征值”即对齐根因；MotionTrunc 默认关、未在 home3/gicp_aw 回归。

**C. 切换观测源类**（Voxmap ↔ GICP 接管）

- Trust 现状：GICP 接管 `L2175-L2249`，接管期 Voxmap 观测被完全否决、只用 GICP。
- 问题：硬切换（二选一）有瞬态；降频帧靠“钉位姿/限幅外推”补又引入新瞬态。
- 软替代（已有）：AutoScene 切 na `L3224-L3250` / 协方差降权——把不可信观测权重连续压到接近 0，等价软切换。
- 差距：需把观测权重做成逐帧连续可调，而不是 na 离散二档（“统一协方差降权闭环”的核心改造点）。

**D. 重锚/清图类**（硬重锚 + force_accept_or_reanchor）

- Trust 现状：超时兜底残差门 `L1707-L1710` + 硬重锚——大残差→放弃旧图、位姿钉在旧位置重建。
- 问题：Trust 里唯一难被软降权替代的部分，“地图段失效”确实需要重建动作。
- 软替代：本质是**地图管理**问题（关键帧选择 / local map 窗口 / 回环），不是观测问题，预防式层面无对应物。
- 差距：正确方向是“局部地图重置 + 保留全局位姿连续性”，而非“全清 + 位姿钉在数秒前”；需单独设计。

**E. 状态速度钳位类**

- Trust 现状：速度硬门 `L2420-L2448`（`|v|>5` 钳到 1 m/s）。
- 问题：速度异常是姿态/零偏误差的重力泄漏，钳速度是症状处理。
- 软替代：治本靠重力对齐 + 零偏估计（na/ng/nba/nbg 已收口）+ 更稳姿态观测。
- 差距：钳速度本身不发散（纯阻尼），可保留作兜底，但不该是主防线。

### 11.3 三个结论

1. A/B/C/E 四类都能被已有预防式机制替代（Huber/退化检测/MotionTrunc/AutoScene/协方差降权），替代版连续、逐帧、无硬切换，天然不可能发散。
2. 唯一真正难替代的是 D（地图段失效重锚），正确解法在地图管理层，不在观测层；把 D 从 Trust 剥出单独做，Trust 状态机可整体退场。
3. Trust 与预防式的对立本质是“用不用”vs“信几成、在哪个方向信”；home3 失败正是“用不用”链在冻结→放行切换点上散掉。

### 11.4 对 §1/§9 “守护机制本身无问题”的精确边界

§1 结论 4、§9 结论 1 说的“守护机制本身无问题”，**精确含义是**：在输入足够稠密时，Trust/Anchor/退化检测完全惰性、不误伤健康输入（`home3_early17_in3` 与 no-de-check 逐位相同已证）。**这不等于**守护机制没有结构性缺陷——它一旦被激活（输入劣化时），放行路径（冻结→放行→泵速）本身就会制造发散。两者并不矛盾：前者是“静默性”，后者是“激活后的稳定性”。
