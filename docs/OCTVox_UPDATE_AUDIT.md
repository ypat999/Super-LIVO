# OctVox 地图代表点更新机制源码审计（OCTVox_UPDATE_AUDIT）

- 日期：2026-11-10
- 审计范围：Super-LIO 的 OctVox 增量地图实现，只关注"代表点如何被更新"，不评估算法优劣。
- 对象：`third_party/Super-LIO`（commit `67c0acd4`），与 `r3live.yaml` 默认配置。
- 方法：逐行读码 + 参数链路核对（YAML → declare/get → 全局变量 → 使用点）。

## 0. 结论速览（含三处与实验假设不符的事实）

| # | 事实 | 对实验的影响 |
|---|---|---|
| 1 | **C 组（最近中心点）已经实现**：`rep_mode=1`，默认关闭 | 阶段二闭环不需新写代码，只需改 YAML |
| 2 | **B 组（首次观测）未实现**；A、C 已实现，D 无 | B 必须新增 `rep_mode=2`，D 只能离线做 |
| 3 | **进入 OctVox 的点已是"最近子体素中心"的真实点**（`VoxelGridClosest`，0.5 m） | "保留真实观测点"的对照组差异被前级降采样部分抹平；B/C 的差别只发生在 0.25 m 子体素内 |

补充两条关键机制：

- **合并门槛是"到当前代表点"的距离**（`merge_dist=0.1 m`），不是到子体素中心 ⇒ 存在点顺序依赖（用户 §5.1 要求的"单独记录这种依赖"见 §7）。
- 降采样网格用 `round`、OctVox 用 `floor`，两者**相差半个格子（0.25 m）**，同一物理点所属的"0.5 m 父体素"在前级与地图层并不相同（不影响正确性，但影响"子体素中心"的物理含义，见 §6）。

## 1. 源码位置与调用链

| 环节 | 文件 | 位置 |
|---|---|---|
| 地图容器 | [OctVoxMap.hpp](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/include/OctVoxMap/OctVoxMap.hpp#L91-L164) | `OctVox` 类 L91-164；`OctVoxMap` 类 L168-324 |
| 代表点更新 | 同上 | `OctVox::AddPoint` L113-149 |
| 插入/淘汰 | 同上 | `OctVoxMap::insert` L328-369 |
| 查询 | 同上 | `getTopK` L373-447、`getTopK_VN` L451-471 |
| 导出 | 同上 | `getMap` L475-495、`saveMap` L521-562 |
| 前级降采样 | [VoxelGridFilter.h](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/include/OctVoxMap/VoxelGridFilter.h#L48-L86) | `VoxelGridClosest::filter` L48-86 |
| 地图创建 | [super_lio.cpp](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/lio/super_lio.cpp#L199) | L199 `new OctVoxMapType(Options{g_ivox_resolution, g_ivox_capacity})` |
| 每帧写图 | 同上 | `UpdateMap` L2306-2331；`DownSample` L1586-1617 |
| 参数声明 | [ROSWrapper.cpp](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/ros/ROSWrapper.cpp#L389-L399) | L389-399 `lio.hash_map.*` |
| 参数默认值 | [params.cpp](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/lio/params.cpp#L178-L181) | L178-181 |
| 配置值 | [r3live.yaml](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/r3live.yaml#L34-L35) | L34-35 |

每帧写图流程（[super_lio.cpp L2306-L2331](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/lio/super_lio.cpp#L2306-L2331)）：

```
undistortCloud(pc → scan_undistort_full_)
  → delta correction（按校正后位姿重写 scan_undistort_full_ 与 ds_undistort_，L1564-1566）
  → DownSample()：VoxelGridClosest 以 0.5 m 叶子降采样 → ds_undistort_（L1586-1617，超 20000 点截断）
  → Observe()：用 ds_undistort_ 做 HKNN + 点面残差 → ESKF 更新得到后验位姿
  → UpdateMap()：ds_undistort_ 逐点 R·p + t（后验位姿）→ ivox_->insert()
```

## 2. 父/子体素几何

`Options`（L180-188）→ `SetOptions`（L218-237）：

```cpp
resolution_        = 0.5              // 父体素边长（lio.hash_map.vox_resolution）
sub_resolution_    = resolution_ / 2   // = 0.25 m（子体素边长，代码派生，不可配）
sub_inv_resolution_ = 4.0
```

插入时的分格（L339-348）——**floor 约定**：

```cpp
fine_key  = floor(p / 0.25)               // 子体素索引
key       = fine_key >> 1                 // 父体素索引（算术右移 = floor(x/2)，负数一致）
local_idx = (dz<<2)|(dy<<1)|dx ,  d* = fine_key[*] & 1
```

- 父体素 `key` 的实际空间范围：`[key*0.5, (key+1)*0.5)`，8 个边长 0.25 m 的子体素。
- 新建父体素时记录 `origin_ = key * 0.5`（L352），仅用于推算子体素中心。
- 子体素中心（L106-111）：

```
SubCenter(local_idx) = origin_ + ( (idx&1)?3:1 ) * 0.125   // x
                              + ( (idx&2)?3:1 ) * 0.125   // y
                              + ( (idx&4)?3:1 ) * 0.125   // z
```

即子体素**几何中心**（0.125 / 0.375 偏移），与用户 §四 C 组要求的 `c_s` 完全一致，且是子体素中心而非父体素中心。

## 3. 代表点更新规则（`AddPoint`，L113-149）

每个子体素保存 **1 个代表点 + 1 个计数**（`std::array<Point,8> points_`、`std::array<uint8_t,8> counts_`，L159-160）。`counts_` 用 `0` 表示未占用（`UNINIT_MASK=0x00`，L157）。

逐条对应源码：

```cpp
// (a) 首次占用
if (count == UNINIT_MASK) { stored = pt; count = 1; return; }          // L116-120

// (b) 计数上限
if (count >= g_map_max_points) return;                                 // L122  默认 20

// (c) 合并门槛：到"当前代表点"的距离
if ((pt - stored).squaredNorm() > g_map_merge_dist^2) return;          // L123-124 默认 0.1 m

// (d) 代表点模式
if (g_map_rep_mode == 1) {                                             // L126-134
  if (|pt - center|^2 < |stored - center|^2) stored = pt;              // 严格小于，无 ε
  ++count; return;
}
stored = (stored * count + pt) / (count + 1);                          // L137 增量均值
++count;                                                                // L138
if (g_map_clamp_mu && sub_res > 0) 逐轴夹回 [center-0.125, center+0.125];  // L140-148 默认 false
```

要点：

1. **A 组（增量均值）**：`μ ← (μ·n + p)/(n+1)`，等权，无遗忘因子；`n` 上限 20（到 20 后不再变化）。浮点为 float（`Point` = `BASIC::V3`，与 `pcl::PointXYZI` 同精度链路）。
2. **C 组（最近中心点）**：已有实现；`ε = 0`（严格 `<`），即距离相等时不替换。
3. **B 组（首次观测）不存在**；语义上等于"跳过 (d) 的两条分支、只 `++count`"，需新增分支（建议 `rep_mode=2`）。
4. **D 组（全量累积）不存在**；`points_` 固定 8 个槽位，结构上不支持多代表点。
5. `(c)` 是**唯一**的几何准入条件，它比较的是 `pt` 与 `stored`——A 组下 `stored` 是均值（会漂移），C 组下 `stored` 是某个真实点，B 组下是首点 ⇒ **三种模式的"可接受点集"并不相同**，这本身是策略差异的一部分，不能关闭该项来"公平化"（用户 §四 明确要求保留）。

## 4. 现有可调参数（已有实验钩子）

| YAML 键 | 全局量 | 默认 | 作用 |
|---|---|---|---|
| `lio.hash_map.vox_resolution` | `g_ivox_resolution` | 0.5 | 父体素边长（子体素 = 一半） |
| `lio.hash_map.hash_capacity` | `g_ivox_capacity` | 1e6（r3live: 2e6） | 父体素容量上限 |
| `lio.hash_map.rep_mode` | `g_map_rep_mode` | 0 | 0=均值，1=最近中心真实点 |
| `lio.hash_map.merge_dist` | `g_map_merge_dist` | 0.1 m | 合并距离门限（对 stored） |
| `lio.hash_map.max_points` | `g_map_max_points` | 20 | 每子体素最多融合次数 |
| `lio.hash_map.clamp_mu` | `g_map_clamp_mu` | false | 均值后夹回子体素包围盒 |
| `lio.map.freeze` | `g_map_freeze` | false | 冻结地图（只读） |

`r3live.yaml` 只显式设置了 `hash_capacity: 2000000` 与 `vox_resolution: 0.5`，其余全部取默认值 ⇒ **历史 sl 运行为 A 组（均值、无 clamg、merge 0.1 m、max 20）**。

## 5. 代表点如何参与建图查询

- `getTopK`（L373-447）：由查询点所在 0.25 m 子体素索引，按 `HKNN_neighbor_voxel` 查 8 个父体素（镜像轴选择），每个父体素**只读取该 local_idx 对应的那 1 个代表点**（`getPoint(local_idx)`，L415/L433）。
- `KNNHeapType = KNNHeap<8, Point>`（L177）⇒ 平面拟合用 8 邻域代表点，与 `Observe()` 一致（[super_lio.cpp L1755](file:///wsl.localhost/Ubuntu-22.04/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/lio/super_lio.cpp#L1755), L1880）。
- 结论：**代表点直接决定点面残差与平面法向**，不存在"另存一套精确点"的备份。因此代表点误差会原样进入 ESKF 观测。

## 6. 进入 OctVox 前的降采样（必须与跨帧融合区分）

`DownSample()`（L1586-1617）用 `VoxelGridClosest`（leaf = `lio.sensor.voxel_fliter_size` = 0.5 m）：

```cpp
idx    = round(p / leaf)                  // 注意是 round（格子中心在 leaf 整数倍）
center = leaf * idx
每格保留 |p - center| 最小的那个真实点         // 不是质心、不是首点
```

要点：

1. 该级输出**仍是真实测量点**（`points_.push_back(pt)`），未被平均。
2. 因为是 `round`，格子中心在 0.5 的整数倍；而 OctVox 用 `floor`，父体素为 `[k·0.5, (k+1)·0.5)` ⇒ **两级网格错开 0.25 m**。同一帧内，一个 0.5 m 降采样格里的点，进入地图后可能落在相邻两个父体素里。
3. 每帧进入 OctVox 的点数已被截断到 20000（L1605-1616）；同时上游 ROS 层 `filter_rate=3` 抽稀 + 0.5 m 降采样已先执行一次 ⇒ `ds_undistort_` 实际是"抽稀→0.5 m 最近中心→0.5 m 最近中心"的结果，每 0.5 m 格每帧**至多 1 个点**。

推论（对实验很关键）：

- 用户 §四 C 组要求的"每帧每子体素若存在多个候选点，先取距中心最近的一个"，在本实现中**天然满足**（前级降采样已保证每 0.5 m 父格每帧 ≤1 点 ⇒ 每个 0.25 m 子格每帧 ≤1 点）。仍需在离线实验中统计验证（见 §9）。
- "保留真实观测点"（B/C）与 A 的差异，只体现为"同一子体素内、跨帧多个候选点如何取舍"，而每个候选点本身已是"最近中心"的真实点 ⇒ C 相对 B 的增益上限较小，这是先验预期，不是结论。

## 7. 其他替换/裁剪/顺序机制

| 机制 | 位置 | 说明 |
|---|---|---|
| 容量淘汰 | L359-363 | `data_.size() >= capacity_` 时删除链表尾（LRU，L366 每次访问 `splice` 到表头）。本数据集体素量 ≪ 2e6 ⇒ 不会触发 |
| `reset_map_` 跳过 | L329-336 | `resetMap()` 后连续 10 次 insert 被跳过（重定位初始化用）；本实验不触发 |
| 地图冻结 | L2308 | `g_map_freeze` 时整个 `UpdateMap` 直接返回 |
| 均值夹取 | L140-148 | 默认关闭 |
| 点过滤 | 上游（ROS 层） | `blind 2 m / maxrange 100 m / tag`，对所有组相同 |

**顺序依赖（必须记录）**：由于 (c) 以 `stored` 为参照，A/B/C 的最终代表点都依赖点的进入顺序：

- A：均值与顺序无关，但"哪些点被 (c) 接纳"与顺序有关 ⇒ 最终也有轻微顺序依赖；
- B：首点即定值 ⇒ 对顺序最敏感（等价于"谁先到"）；
- C：每次可能替换，且 (c) 的参照点在变 ⇒ 存在顺序依赖，但收敛方向是"离中心更近"，理论上比 B 稳定（需实验验证，见阶段一 §7.1）。

## 8. 并发与线程

- `insert` / `getTopK` 全部在 LIO 主线程（`UpdateMap` L2306、`Observe` L1755/1880）；`DownSample` 同线程 ⇒ 无写-写竞争。
- 唯一跨线程访问：`OutputThread` 打印 `ivox_->mapSize()/bucketCount()/takeStats()`（L2501-2503），只读 + 统计自增，存在良性数据竞争（不影响点坐标）。
- `DenseThread`（`cloud_body_dense` 产线）**不访问** `ivox_`，只做去畸变 + delta correction ⇒ 与本实验相互独立。
- `super_lio_reloc.cpp` L133/L314 另有 insert（重定位/回环子图路径），本实验不启用。

## 9. 对 A/B/C/D 分组的实现影响

| 组 | 现有实现 | 需要的改动 | 备注 |
|---|---|---|---|
| A 增量均值 | ✅ `rep_mode=0` | 无 | 与历史 sl 一致 |
| B 首次观测 | ❌ | 新增 `rep_mode=2`：仅 `++count`，不动 `stored` | 约 3 行 |
| C 最近中心 | ✅ `rep_mode=1` | 无（ε=0，如需 ε 需加判断） | 已可用 |
| D 全量累积 | ❌ | 结构不支持；离线另建 | 仅阶段一离线做 |

离线实验（阶段一）不依赖上述 C++ 改动：在 Python 中按 §2/§3 精确复刻语义（同一输入点集、同一插入顺序）即可实现四组；闭环实验（阶段二）再按上表改 `rep_mode`。

> **实施记录（2026-11-10）**：已按上表在 `OctVox::AddPoint` 新增 `rep_mode == 2` 分支
> （`third_party/Super-LIO/src/super_lio/include/OctVoxMap/OctVoxMap.hpp` L126-130，
> 仅 `++count; return;`），用于阶段二串联 B 组闭环实例。该分支只影响代表点是否更新，
> 不触碰 ESKF、残差、KNN、体素划分。A/C 未改动。

## 10. 审计问题逐条回答

| # | 问题 | 结论 |
|---|---|---|
| 1 | 父体素尺寸、8 子体素边界与中心 | 父 0.5 m（可配），子 0.25 m；父 `[k·0.5,(k+1)·0.5)`；子中心 = origin + {0.125/0.375}（L106-111） |
| 2 | 每子体素是否最多 1 个代表点 | 是（`array<Point,8>` + `array<uint8_t,8>`，L159-160） |
| 3 | 增量平均公式、计数更新与上限 | `μ←(μn+p)/(n+1)`；`n++`；上限 `max_points=20`（L122/L137-138） |
| 4 | 合并阈值/拒绝/更新条件 | 拒：`|p−μ|>0.1 m` 或 `n≥20`；更新：模式 1 需更近才替换，模式 0 直接均值（L122-138） |
| 5 | 其他替换/滤波/裁剪 | LRU 容量淘汰、reset 跳过、freeze、可选均值夹取（§7） |
| 6 | 地图点是否直接参与 KNN/点面 | 是，`getTopK` 只读代表点，K=8（§5） |
| 7 | 进入 OctVox 前是否已 center-based 降采样 | 是，双层：ROS 层 `filter_rate=3`+0.5 m，`DownSample()` 再 0.5 m；降采样 = 保留离格中心最近的真实点（§6） |
| 8 | 多线程竞争/异步/顺序依赖 | 无写竞争（主线程）；存在良性只读竞争；顺序依赖见 §7 |

## 11. 复现与文件清单

```bash
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO
git rev-parse HEAD            # 67c0acd4...
sed -n '91,170p;328,370p' src/super_lio/include/OctVoxMap/OctVoxMap.hpp
sed -n '48,87p'            src/super_lio/include/OctVoxMap/VoxelGridFilter.h
sed -n '2306,2331p;1586,1617p' src/super_lio/src/lio/super_lio.cpp
```

配置基线：`r3live.yaml`（sha256 `632e5721…e1d2a1`），`hash_capacity=2e6`、`vox_resolution=0.5`，其余 `lio.hash_map.*` 取默认（§4）。
