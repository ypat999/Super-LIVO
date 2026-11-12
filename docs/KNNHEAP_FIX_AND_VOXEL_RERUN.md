# KNNHeap 修复与体素复跑（A/B/C）

> 生成时间：2026-11-12
> 目的：先修正 `KNNHeap<8>` 的最远元素定位缺陷，再用**同一份修复后代码**复跑 voxel = 0.25 / 0.15 / 0.10，
> 用来判断此前"voxel 0.25→0.15 精度变好"的结论在修复邻域选择器之后是否仍然成立。

---

## 1. 为什么先修 KNNHeap

此前 `KNNHeap::update_worst_unrolled()` 只展开比较 `dist2_[0..4]`，而当前分支的堆容量是 `K = 8`。
堆填满后它找到的只是"前 5 个槽位里的最远点"，第 6~8 槽里的远点永远无法被淘汰，
`getTopK` 返回的邻居集合就不是全局最近的 8 个点。

后果：**旧的 voxel=0.15 结果是在一个可能错误的邻域选择器下得到的**。
它能说明"当前实现下指标改善"，但不能保证修好 KNN 后趋势仍然成立。
所以不能直接拿旧 0.15 和新 0.10 比，那会把"代码修复"和"体素变化"混在一起。

### 改动

| 文件 | 改动 |
|---|---|
| `src/super_lio/include/OctVoxMap/KNNHeap.hpp` | **新增**。`KNNHeap` 从 `OctVoxMap.hpp` 抽出，零重依赖，便于单测 |
| 同上 | `update_worst()` 遍历全部 `K` 个槽位（不再是 5 槽展开） |
| 同上 | 新增 `static constexpr int kSize = K` |
| `src/super_lio/include/OctVoxMap/OctVoxMap.hpp` | 删除内联类定义，改为 `#include "KNNHeap.hpp"` |
| 同上 | `getTopK` 的 `count == 8` 改为 `KNNHeapType::kSize`，去掉硬编码 |
| `test/test_knn_heap.cpp` | **新增**。与暴力 Top-K 逐点对照的回归测试 |

未改动（刻意保留原语义）：未填满时按到达顺序占槽、`dist2 < max_dist2_` 的严格替换判据、
等距时保留哪一个点（等距时保留项本来就任意，不影响 Top-K 集合）。

---

## 2. 单元测试

```bash
cd <repo>/third_party/Super-LIO
g++ -std=c++17 -O2 -I src/super_lio/include -I /usr/include/eigen3 \
    test/test_knn_heap.cpp -o /tmp/test_knn_heap && /tmp/test_knn_heap
```

覆盖：候选数 `< K` / `== K` / `> K`；先让 8 个远点占满堆、再逐个插入更近点（正序 + 逆序）；
200 组随机点集 × 随机插入顺序；全部等距 / 近乎等距；同时跑 `KNNHeap<5>` 防回归；
额外校验 `worst_` / `max_dist2_` 确实指向槽位内最大值。

判据是**返回的邻居集合与暴力 Top-K 是否完全一致**（距离多重集 + 点集），不是条件数或残差。

已验证结果：

| 实现 | 结果 |
|---|---|
| 修复后 | `ALL PASS` |
| 同一份测试套用旧 5 槽逻辑 | **169 处 FAIL** |

即：缺陷真实存在，且该测试能抓住它。

---

## 3. 实验矩阵

三组都使用**修复后的同一份代码**，固定 `K = 8`、`filter_rate = 6`、`imu_na/ng = 0.1`、`DEBUG_LEVEL=2`，
只改体素尺寸（`voxel_fliter_size` 与 `hash_map.vox_resolution` 同步改）。

| 实验名 | voxel / vres | 目的 |
|---|---|---|
| `knnfix_vox025` | 0.25 / 0.25 | 修复后的基准 |
| `knnfix_vox015` | 0.15 / 0.15 | 确认此前的改善是否仍存在 |
| `knnfix_vox010` | 0.10 / 0.10 | 检查进一步细化 |

已写入 `exp/run_all.sh` 的 `CASES`，无需临时改脚本。

---

## 4. 在另一台机器上的操作

```bash
REPO=~/git/Super-LIVO
EXP=$REPO/third_party/Super-LIO/exp

# (1) 构建（必须先做，修复在头文件里，不改代码等于没修）
cd $REPO/third_party/Super-LIO
source /opt/ros/humble/setup.bash
colcon build --packages-select super_lio --cmake-args -DCMAKE_BUILD_TYPE=Release

# (2) 单测
g++ -std=c++17 -O2 -I src/super_lio/include -I /usr/include/eigen3 \
    test/test_knn_heap.cpp -o /tmp/test_knn_heap && /tmp/test_knn_heap

# (3) 三组跑批（BAG 换成那台机器上的实际路径；RESULT_ROOT 必须是绝对路径，见第 6 节）
cd $EXP
BAG=/absolute/path/to/<bag_dir> \
RESULT_ROOT=$EXP/results_knnfix \
WS_SETUP=$REPO/third_party/Super-LIO/install/setup.bash \
JOBS=1 \
./run_all.sh knnfix_vox025 knnfix_vox015 knnfix_vox010 2>&1 | tee /tmp/knnfix_full.log

# (4) 汇总
python3 $EXP/analyze.py --root $EXP/results_knnfix
# 输出 $EXP/results_knnfix/summary.csv 与 summary.md
```

`run_one.sh` 会拼接 `lio.hash_map.vox_resolution = <vres>`、`lio.sensor.voxel_fliter_size = <voxel>`、
`lio.sensor.filter_rate`、`DEBUG_LEVEL` 等参数，其余取 `config_template.yaml` 默认值。
若那台机器上没有 `/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash`，
`run_one.sh` 会跳过（有 `-f` 判断）；需要 Livox 消息包时自行改 `LIVOX_SETUP`。

---

## 5. 需要采集的指标与判据

主判据（几何精度）：

- 最低停留层的 `dxy` / `dyaw`（`analyze.py` 的分层墙面配准结果）
- 跨帧墙面法向偏移、墙面弯曲幅度 `p95-p5`
- 有效平面约束数量 `n_effect` / `effect_ratio`
- KNN 邻居数量分布（`n_knn_fail` 与之互补）

辅助（局部优化质量，不能替代几何指标）：

- `cond(Yk)`、`fit_res_mean` / `p95`

预期读法：先看 A（0.25）与 B（0.15）在**修复后**是否仍复现"变小体素更好"；
若 B 仍优于 A，再看 C（0.10）是否继续改善、还是因为单帧点变多 / 地图点变多而回退。

---

## 6. 已知坑与注意事项

1. **`$PWD` 不能出现在传给 `wsl -d ... bash -lc` 的命令里**。
   从 Windows 侧调用时 `$PWD` 会被 PowerShell 展开成
   `Microsoft.PowerShell.Core\FileSystem::\wsl.localhost\...`，
   导致 `params.yaml` 路径失效、节点退回默认参数、结果写到错误目录。
   实测已踩过：现象是 `node.log` 里没有参数相关日志、`results_knnfix` 目录为空。
   `RESULT_ROOT` 一律写绝对路径。

2. **bag 版本**。本机原先 1274 s 的 A 楼 bag 已不在磁盘上，只剩
   `~/slam_data/livox_record_noshell`（683.8 s / 6789 帧，约原来一半）。
   同一台机器上 A/B/C 内部对比有效，但绝对值不能和旧 1274 s 表混用。
   到另一台机器上请确认用的是哪一份 bag 并在 `meta.txt` 里记下来。

3. **`hash_capacity = 2,000,000`**。voxel = 0.10 时地图点数会明显上升，
   若逼近上限会触发淘汰（`n_evict`），那一点的数据要单独标注，不能当作纯体素效应。

4. **20k 截断与本次实验无关，但别忘**。`kMaxObservePoints = 20000` 在源码里确实存在，
   但楼梯间 bag 每帧只有 719~1172 个点，从未触发。
   在 `Retail_Street`（Avia，单帧点更多）上跑时，仍要
   `grep '超过配准上限' node.log` 确认是否截断——两个数据集不能混为一谈。

5. **脚本行尾**。`exp/run_all.sh`、`exp/run_one.sh` 在工作区里可能是 CRLF，
   直接 `./run_all.sh` 会报 `bad interpreter`。已转成 LF；若再次从 Windows 侧编辑请复查 `file`。

---

## 7. 本机已完成的部分

- 修复 + 单测：完成，`ALL PASS` / 旧逻辑 169 FAIL。
- 三组整包跑批：用 `livox_record_noshell` 跑在
  `exp/results_knnfix/`（`knnfix_vox025` → `knnfix_vox015` → `knnfix_vox010`，顺序执行）。
  该结果可作为交叉参考，但由于 bag 与另一台机器不同，**不能替代**在目标 bag 上的复跑。

### 实测结果（`livox_record_noshell`，674 s / 6744 帧）

**先说前提**：这份 bag 不是楼梯间。z 只从 -1.0 到 2.18（旧 A 楼是 -41 到 +9，约 50 m），
`low_rel = 0` 即全程只停留在一层，因此 `low_dxy/low_dyaw` 恒为 0，**分层墙面指标在这份数据上无效**。
且该段路径首尾不重合（非闭环），下表只反映这段开放路径上的局部行为。

| 指标 | 0.25 | 0.15 | 0.10 |
|---|---|---|---|
| 墙面弯曲 `bow_pp` (m) | 0.257 | 0.256 | **0.254** |
| 单帧点数 `ptsize_mean` | 957 | 1082 | 1138 |
| `fit_res_mean` (m) | 0.0561 | 0.0439 | **0.0369** |
| `cond_yk_median` | 5219 | 4905 | **4293** |
| `htvh_cond_median` | 316 | 272 | **241** |
| `effect_ratio` | **0.9375** | 0.9333 | 0.8862 |
| `knn_fail_ratio` | **0.0252** | 0.0560 | 0.1114 |
| `plane_fail_ratio` | 0.0366 | 0.0106 | **0.0024** |
| 地图点数 | 837 k | 850 k | 880 k |
| 耗时 `stage_total_ms` | **5.90** | 7.61 | 8.88 |

**累计精度在这份 bag 上不可评**：该 bag 的起点与终点不在同一物理位置（不闭环），
`closure_3d` 比的是两个不同地点，`revisit_min_xy` 只是"离起点最近的一帧"（`revisit_t ≈ 567 s`，
是中途路过而非闭环）。两者都不构成闭环真值。
因此 `analyze.py` 的 `drift_score = closure_3d + low_dxy + 0.1×|low_dyaw|` 排名在本 bag 上无效，
`summary.md` 里"以 knnfix_vox025 作为默认配置"的结论同样作废；
**上一版文档据此写的"0.25→0.15→0.10 单调变差、旧 0.15 优势来自错误堆"整段已撤回**。

读法（只保留不依赖闭环的判据）：

1. **局部优化质量随细化单调变好**：`fit_res` 0.0561→0.0439→0.0369（−34%），
   `cond_yk` 5219→4905→4293，`htvh_cond` 316→272→241，`plane_fail_ratio` 3.7%→0.24%。
   这是"体素变小 ⇒ 邻域点更贴平面"的直接结果，与 KNN 是否修好无关，两种实现都会出现。
2. **邻域可得性随细化单调变差**：`knn_fail_ratio` 2.5%→5.6%→11.1%，`effect_ratio` 0.938→0.886。
   体素变小 ⇒ 物理邻域缩小 ⇒ 凑不满 K 个邻居，对应排查意见第 2 条。
3. **地图自一致性几乎不变**：`bow_pp` 0.257→0.254（差 1%）。该 bag 平面结构简单，
   这项只能算弱证据，不足以区分三组。
4. **三者都在稳定跟随同一物理轨迹**：`path_m` 704.86 / 705.29 / 706.02（差 <0.2%），
   `big_jumps = 0`，`diverged = 0`。但这只说明没有发散，不能说明谁更准。
5. **没有容量污染**：地图点数只涨 5%，离 `hash_capacity = 2,000,000` 很远，0.10 这一组的数据是干净的。
6. **`loop1_*` / `t_ascend` 不可跨组比**：`t_ascend` 分别是 674.3 / 359.2 / 324.4 s，
   A 组的搜索窗被迫覆盖整段（`loop1_t = 667.6`），B/C 在 102.8 s，不是同一物理事件。
7. **无截断告警**：三组 `grep -c '超过配准上限' node.log` 均为 0，又一次确认 20k 与本问题无关。

**结论**：在这份非楼梯间、不闭环的 bag 上，唯一能确定的是
"体素变细 ⇒ 局部拟合更准、邻域更稀、更慢"，**累计/全局精度无法判定**，
也不能据此对 0.25 / 0.15 / 0.10 排序。要回答"修复 KNN 后 0.15 的优势是否还在"，
必须用闭环的楼梯间 bag 取 `low_dxy/low_dyaw` 与分层墙面证据，
即明天在另一台机器上跑第 4 节的命令。

---

## 8. `livox_record_drone` / `livox_record_drone_1f` 复跑（2026-11-12）

同一份已修复代码（`KNNHeap<8>`，`update_worst()` 遍历全部 K 槽），
`filter_rate=6`、`imu_na/ng=0.1`、`DEBUG_LEVEL=2`、整包 1× 播放、顺序执行、`JOBS=1`。
结果目录 `exp/results_livox_record_drone{,_1f}/`。

| bag | lidar 帧 | 轨迹时长 | 帧数 | 路径长 | 起终点关系 |
|---|---|---|---|---|---|
| `livox_record_drone` | 4701 | 466.9 s | 4670 | ~297 m | z 0.1 → −34.6 → 0.13，**回到起点** |
| `livox_record_drone_1f` | 6405 | 637.4 s | 6375 | ~775 m | **两次闭环**：两趟往返均回到起点 |

### 8.1 先判闭环性，再决定哪些指标能用

- **`livox_record_drone` 是真闭环**：下到 z≈−34.6 m 再原路返回，末 20 帧均值与首 20 帧均值相差
  `(−0.21, −0.28, 0.02) m`（模长 0.35 m，0.10 组）。
- **`livox_record_drone_1f` 是两次闭环（修正上一版"不闭环"的误判）**：
  第一趟 0 s 出发、~140 s 到达 153 m 远端、~285 s 回到起点附近停留到 ~366 s；
  第二趟随即再出发、~470 s 到达 127 m 远端、~567 s 回到起点直到结尾。单趟约 290 / 300 m。
  上一版只比了"起点 vs 最末帧"，而结尾段残差 3–6 m、包尾并未精确停回，被误判成"终点在别处"。
  正确判据是**两个闭环停留窗口内轨迹相对起点的残差**（见 8.3），`closure_3d`（首尾差）单独看仍无意义。
- **两 bag 都没有楼层停留**：`detect_floor_levels` 靠 z 速度平台段（|vz|<0.035 m/s 且 ≥4 s）分楼层，
  这两段几乎是连续下降/平移，只分出 1 层（1f 的 0.10 组分出 2 层但最低层无墙面匹配）。
  因此 `low_dxy/low_dyaw` 仍不可用 —— 不是算法失效，是数据里没有"逐层停留"。
- `loop1_*` 两 bag 都无效：`t_ascend` 落在包尾（467.1 / 23.3 s），首圈搜索窗不成立。

### 8.2 `livox_record_drone`（闭环，35 m 竖向）实测

| 指标 | 0.25 | 0.15 | 0.10 |
|---|---|---|---|
| **终点闭环 `closure_3d` (m)** | 3.30 | 1.67 | **0.35** |
| 终点水平偏差 `final_xy` (m) | 3.10 | 1.52 | **0.35** |
| 终点高度偏差 `final_z` (m) | −1.13 | −0.69 | **+0.03** |
| 终点偏航 `final_yaw` (°) | 1.42 | 0.43 | **−0.17** |
| 路径长 `path_m` (m) | 297.13 | 296.56 | 296.38 |
| 单帧点数 `ptsize_mean` | 651 | 990 | **1275** |
| `fit_res_mean` (m) | 0.0339 | 0.0222 | **0.0170** |
| `cond_yk_median` | 401.5 | 427.2 | 437.7 |
| `htvh_cond_median` | 45.6 | 32.5 | **27.3** |
| `effect_ratio` | 0.9781 | **0.9826** | 0.9739 |
| `knn_fail_ratio` | **0.0075** | 0.0146 | 0.0255 |
| `plane_fail_ratio` | 0.0127 | 0.0019 | **0.0003** |
| 地图点数 | 122 k | 123 k | 113 k |
| 耗时 `stage_total_ms` | **4.36** | 6.48 | 8.39 |

读法：

1. **闭环误差随体素细化单调下降**：3.30 → 1.67 → 0.35 m，且三个分量（水平/高度/偏航）
   方向一致：0.25 组结束时低了 1.13 m、偏了 1.42°，0.10 组基本回到原点（3 cm / 0.17°）。
   这与此前在非闭环 bag 上得到的"越细越差"相反 —— 那条结论已经撤回，这里不再受其影响。
2. **这个排序不是容量假象**：三组 `[MapStats] evict` 全程为 0，地图规模 113–123 k，
   远低于 2 000 000 上限。
3. **局部质量同向变好**：`fit_res` 0.0339→0.0170（−50%），`plane_fail_ratio` 0.0127→0.0003。
4. **代价是邻域更稀 + 更慢**：`knn_fail_ratio` 0.75%→2.55%，`effect_ratio` 0.978→0.974（轻微下降），
   `stage_total_ms` 4.36→8.39 ms（仍在 100 ms 预算的 8.4% 内）。
5. `cond_yk_median`（401→438）和 `htvh_cond`（45.6→27.3）方向相反，说明"条件数"不能单独当判据，
   与上一轮的建议一致。

### 8.3 `livox_record_drone_1f`（两次闭环；0.10 组已用 4M capacity 复跑）

上一版把这份 bag 判为"不闭环 + 0.10 组被 2M 容量污染、整组作废"，两处都已修正：
它实际有**两次闭环**；0.10 组用 `HASH_CAPACITY=4000000` 重跑（组名 `knnfix_vox010_cap4m`），
全程 `evict=0`（体素需求峰值 2 829 887，确实超过旧 2M 上限，旧组后半程在持续淘汰属实）。

**闭环残差**（停留窗口中位数，相对首 20 帧均值；窗口内机体自身散布 ~1 m）：

| 闭环点（累计行程） | 0.25 | 0.15 | 0.10 cap4m | 0.10 旧(2M) |
|---|---|---|---|---|
| 闭环1 285–366 s（~380 m）`dxy / dz / d` | 2.75 / −1.35 / **3.06** | 2.20 / −2.21 / 3.12 | 2.57 / −2.45 / 3.55 | 2.57 / −2.45 / 3.55 |
| 闭环2 567 s–结尾（~775 m）`dxy / dz / d` | 2.70 / −2.54 / **3.71** | 2.18 / −3.45 / 4.08 | 3.44 / −4.48 / 5.65 | 3.53 / −4.57 / 5.78 |

读法：

1. **cap4m 与旧 010 组到闭环 1 逐位相同**（旧组的 `evict` 从 bag t≈366–420 s 才开始，
   在闭环 1 之后），闭环 2 残差只从 5.78 回收 0.13 m 到 5.65 m。
   **即：0.10 组在 1f 上闭环最差不是容量假象，是体素细化在这个场景的真实结果。**
2. **排序与 drone bag 相反**：这份 bag 上 0.25 两个闭环点都最好，0.10 都最差；
   竖向漂移随细化单调变差（闭环 2 平均 dz：−2.03 / −2.95 / −3.98 m）。
3. 绝对量级：3–6 m / 775 m（0.4–0.8%），三组都远差于 drone bag 的最好成绩；
   停留窗内散布 ~1 m，说明排序可信、绝对值精度有限。

代价与局部指标（0.10 列取 cap4m 组）：

| 指标 | 0.25 | 0.15 | 0.10 cap4m |
|---|---|---|---|
| 单帧点数 `ptsize_mean` | 847 | 1006 | 1085 |
| `fit_res_mean` (m) | 0.0530 | 0.0431 | **0.0358** |
| `cond_yk_median` | 3058 | 2674 | **2068** |
| `effect_ratio` | **0.8959** | 0.8874 | 0.8236 |
| `knn_fail_ratio` | **0.0462** | 0.0955 | 0.1732 |
| `plane_fail_ratio` | 0.0568 | 0.0170 | **0.0031** |
| 体素需求峰值（MapStats size） | 0.30 M | 1.73 M | 2.83 M（evict=0） |
| 耗时 `stage_total_ms` | **7.43** | 9.53 | 9.75 |

局部质量随细化变好的方向与 drone bag 一致；`bow_pp` 0.331 / 0.155 / 0.273 仍无单调性，不作判据。

### 8.4 结论

1. **两份闭环 bag 方向相反，"缩小体素"不是普适改善**：
   - `livox_record_drone`（单闭环）：闭环 3.30 → 1.67 → 0.35 m，**越细越好**；
   - `livox_record_drone_1f`（双闭环，0.10 组已用 4M capacity 复跑排除淘汰）：
     闭环 1 3.06 / 3.12 / 3.55、闭环 2 3.71 / 4.08 / 5.65 m，**越粗越好**。
   修复 KNN 后，"缩小体素是否改善累计精度"**依赖场景**，单份 bag 不足以做全局参数决策。
2. **两份 bag 一致的代价**：体素越小 `fit_res` 越好、`knn_fail_ratio` 越差、`stage_total_ms` 越大；
   局部质量方向不随场景变，闭环精度方向会变。
3. **容量教训**：voxel 0.10 在 1f 上体素需求峰值 2.83 M，超过默认 2 M 上限。
   对比实验必须先查 `MapStats evict`。`run_one.sh` 已支持 `HASH_CAPACITY` 环境变量
   （默认 2 000 000），`knnfix_vox010_cap4m` 组已加入 `run_all.sh`。
4. **分层墙面证据仍然缺失**：两份 bag 都没有楼层停留，`low_dxy/low_dyaw` 不可得。
   这一项只能等有逐层停留的 bag。
5. 七次运行 `grep -c '超过配准上限' node.log` 全为 0，20k 截断与本次结论无关。

### 8.5 复现注意

- `exp/build_ws.sh` 里的 `WS=/home/ywj/git/xtd2_ws` 已经不能直接用：该工作区没有 `src/`，
  且 `XTDrone2_ego_planner/lio/Super-LIO` 是与本仓库**不同 inode 的旧拷贝**（不含 KNN 修复）。
  本次直接用下面的命令从本仓库编译：

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ywj/git/xtd2_ws/install/setup.bash   # 只为 livox_ros_driver2 / basic 等依赖
  cd /home/ywj/git/Super-LIVO/third_party/Super-LIO
  colcon build --packages-select super_lio \
    --build-base /tmp/sl_build --install-base /tmp/sl_install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
- `/tmp` 每次 WSL 重启都会清空，跑批前必须重新编译（约 3 分钟）。
- 跑批脚本：两 bag 顺序执行，`RESULT_ROOT` 分别为
  `exp/results_livox_record_drone` / `exp/results_livox_record_drone_1f`，`BAG` 指向对应目录。
