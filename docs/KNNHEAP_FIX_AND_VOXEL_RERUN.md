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
