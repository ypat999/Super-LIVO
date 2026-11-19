# Super-LIO 视觉支路单体化整改方案（A + B）

> 状态：部分实施（2026-11-19）。A 的 S1 单体接线已完成；B 已完成节点线程化与 odom 源切换（10 Hz `/lio/odom`），尚未 mailbox 化。
> 范围：`solution_a_lightweight` + `solution_b_tight_coupled` 两条视觉支路，以及公共库 `third_party/Super-LIO`（`liblio`）。
> 目标：把"一个进程内两个 ROS 节点靠 DDS 回环取 LIO 输出"改成"单节点、进程内推式共享缓冲"，视觉逻辑退化为 ROS 无关的流水线类。ROS 只保留外部输入（LiDAR/IMU/图像）与对外输出（可视化/录制）。
> 当前进度与实测数据见 [monolith_refactor_progress.md](monolith_refactor_progress.md)。
> **参数命名（2026-11-19 统一）**：本文涉及的支路开关已全部收进 params 的 `super_livo_visual`
> 分支（顶层 `/**` + `ros__parameters`，摊平后真实名带 `super_livo_visual.` 前缀），旧的 `app.*`
> 一族改名为 `super_livo_visual.embed` / `.inject` / `.inject_max_dt` / `.lio_source`；
> 规则与改名对照表见根 [README「参数文件规范」](README.md#参数文件规范ros-2-params-yaml)。

---

## 1. 目标与非目标

### 目标
1. 进程内 LIO → 视觉链路**不再走 DDS**，改为内部线程通信（推式 mailbox）。
2. 视觉支路从 `rclcpp::Node` 抽成 **ROS 无关的 `VisualPipeline` 类**，可脱离 ROS 单测。
3. A、B 共用同一套公共库钩子 + mailbox 抽象，改动最小、行为逐位一致。
4. 保留一个运行时开关，作为"内部直连 vs DDS 回环"的**实验观察窗口**，稳定后删除对照路径。
5. A 预留**可选视觉回灌**（默认关闭保持轻量），在 Super-LIO 强退化场景经 `ExternalObserve` 写回 ESKF，回灌比例由配置参数控制（§6.5）。

### 非目标
- **不**把 LiDAR/IMU/图像这些外部源拉进进程（它们仍走 ROS 订阅，除非将来嵌入驱动）。
- **不**改动任何算法模块（KLT 前端、深度关联、重投影、退化门控、视觉更新、彩色上色）。
- **不**改变滤波/约束的数学口径（回归要求逐位一致）。

---

## 2. 现状架构与视觉写回 ESKF 机制

### 2.1 方案 A（轻量、后处理式，不写回 ESKF）
`visual_a_app` 是**单进程单节点**（只有 `ROSWrapper`），唯一模式（旧 `lio_source=topic`
对照路径与 `VisualANode` 已在 S2 删除）：

- `VisualASuperLIO` 覆写 `onLioOutput` / `onDenseCloud`，把位姿、
  世界云、机体系全量云直推 `VisualPipeline`（ROS 无关类，**回调入队 / 工作 / 落盘三线程**，
  参照原版 Super-LIO 的 IO 与计算隔离）；
  退化签名取自 `degeneracySnapshot()` 内存快照。图像订阅（bgr8 零拷贝浅拷贝 Mat +
  消息 keepalive 入队）、彩色云 / odom / path 发布、落盘定时器都建在 `ROSWrapper` 上。
  进程内**无 DDS 回环**。
- `super_livo_visual.embed=false`：只跑纯 LIO。

视觉修正**不写回 ESKF**：用 `pose_offset_` 左乘（带泄漏衰减）或 `update.reinject`
写回 `own_pose_cache_`，发布 `super_livo_visual_a/odom`。另提供 TUM 轨迹落盘
（`evo.pose_output_en / evo.save_dir / evo.seq_name / evo.interval`）与按 LIO 时域
命名的 PNG 图像落盘（`image_save.img_save_en / dir / interval`）。
结论：A 现状是后处理，滤波器不感知视觉；本方案为其预留**可选视觉回灌**（§6.5），
默认关闭保持轻量，强退化场景可逐步打开。

### 2.2 方案 B（紧耦合，代码级写回 ESKF）
写回链路（**已非 DDS**）：

```
工作线程 VisualBNode::publishVisualConstraint()   // 线程化后不再在回调线程
  → 白化 + 符号翻转 + 世界左扰动→机体系右扰动 (H18 = JwT·R_wi)
  → ConstraintBridge::push()                     // 单槽互斥信箱
Super-LIO 处理线程 VisualBSuperLIO::ExternalObserve()
  → bridge_->take() + 时间戳新鲜度检查
  → eskf()->UpdateObserveFull(H, r, R)           // 18 维误差状态迭代更新
```

- 视觉约束由 B 自带的 `UnifiedEstimator`（30 维影子滤波器）产出：IMU 传播直订 `/livox/imu`，
  位姿先验订 10 Hz `/lio/odom`（激光帧后验、base_link 系，用 app 注入的 `odom_robo⁻¹`
  换回 IMU 系）做 `lidarUpdate`，再 `visualUpdate`，输出 6×6 `appliedVisual()`。
  注入侧还有 `super_livo_visual.inject_max_dt` 时间门控（本机定为 0.4 s，见 §9.3）。
- 部署形态（**过渡态，不是终点**）：`visual_b_app` 在同一个进程内建
  `ROSWrapper`(LIO) + `VisualBNode`(视觉) 两个节点，两者仍进同一个
  `MultiThreadedExecutor`；视觉→LIO 的写回走 `ConstraintBridge` 直接函数调用
  （`super_livo_visual.inject`），**只有 LIO→视觉这一侧还是 DDS 订阅**。
  B 的目标形态与 A 相同：**单进程单节点**，`VisualBNode` 最终被 `VisualPipeline`
  取代并删除（§7）。
- 节点内部已线程化（commit `b0f5adc`），拆成三条线程：
  1. **回调线程**（rclcpp executor）：只把消息 `ConstSharedPtr` 投进 `input_queue_`，
     不解析、不搬大数据，耗时 O(µs)；
  2. **工作线程**：按到达顺序 FIFO 执行原 `onImage/onOdom/onCloud/onBodyCloud/onImu`，
     全部算法状态仍由这一条线程独占，因此不给它们加锁；
  3. **落盘线程**：只做彩色地图序列化写盘。工作线程每次只移交"上次以来新增的那一段"
     （彩色地图 append-only），落盘线程并进常驻缓冲后写 PCD。

  起因：原先 6 个订阅 + 1 个落盘定时器同在默认互斥回调组串行，一次 47 MB PCD 落盘
  实测阻塞 855~980 ms，期间图像/odom 回调全部停摆，压爆 `pending_visual_`（上限 8）
  白丢 293 帧；第一版改为移交整图快照后仍会拖停（随地图增长到 1267 ms），故必须增量。
- 结论：B 的"写回"已经是直接调用，**要改的是它的输入侧**（odom/imu/cloud/degen 仍走
  DDS）。线程化只解决了"回调被阻塞"，没有消除 DDS 拷贝与时序对齐成本；
  它与 A 的 `VisualPipeline` 已经同构（单线程独占状态，而非多回调组 + 细粒度锁）。

### 2.3 共性痛点
- A 的 `topic` 分支与 B 全线的 LIO 输出仍走 DDS 回环（A 的 `internal` 分支已消除）。
- `/lio/degeneracy` 原树无发布者，B 的 `degen_sev` 恒为 0、"强退化才加强回灌"整条守卫是
  死代码。**已补发布端**（本地未提交，`ROSWrapper::pub_degeneracy()`）：随 10 Hz `pub_odom()`
  把 `super_lio_->degeneracySnapshot()` 转成 8 元素 `Float64MultiArray`
  `[cond6, axes, active, cond_t, evt_min, ux, uy, uz]` 发出（`cond6=cond_t=snap.cond`，
  `evt_min=0`，方向取 `dir_w` 归一化），QoS best_effort 以匹配 B 的 `SensorDataQoS`。
  实测四包均能收到 10 Hz、`cond6` 随场景变化（2.8–5.4，`move_ros2` 出现一次 1e12 截断），
  但健康场景 `active=0`、`sev` 仍为 0——门控不再是死代码，只是还没遇到真退化样本。
  A 的 `internal` 分支不受影响，因为它直接读 `degeneracySnapshot()` 内存快照。

---

## 3. 目标架构

```
外部 ROS（不变）
  LiDAR ─┐
  IMU   ─┼─► ROSWrapper(唯一 rclcpp::Node) ─► SuperLIO(主干, liblio)
  图像  ─┘        │  │                          │
                  │  │ 进程内推式 mailbox         │ OutputThread / DenseThread / imuOutputThread
                  │  │ (mutex ring-buffer)       ▼
                  │  └──────────────────► VisualPipeline(ROS 无关类)
                  │                              │
                  └──(可选对外输出)──────────────┘ 彩色云 / 修正 odom / path
```

> 上图是**目标态**。当前实现：A 已达到该形态（`internal`）；B 仍是 2 节点、
> 且 LIO→视觉走 DDS，差异见下表。

- 内部数据单向：`SuperLIO` 各输出线程 → mailbox → `VisualPipeline`。
- B 额外保留 `ConstraintBridge`（写回方向，已经是代码级，不动）。
- 对外输出（可视化/录制）仍经 `ROSWrapper` 发布，属输出而非内部通信。

现状与目标的差距（2026-11-19）：

| 环节 | A | B | 目标 |
| --- | --- | --- | --- |
| LIO→视觉数据 | `internal` 已进程内直推 | 仍 DDS 订阅 + 输入事件队列 | mailbox 直供 |
| 视觉→LIO 写回 | 未做（后处理） | 已代码级 `ConstraintBridge` | 同 B |
| 退化签名 | 内存快照 `degeneracySnapshot()` | 订阅 `lio/degeneracy`（发布端已补，过渡） | 内存快照 |
| 线程模型 | 单工作线程 + 任务队列 | 回调 / 工作 / 落盘三线程 | 统一到 `VisualPipeline` |
| 进程/节点数 | 1 进程 1 节点 | 1 进程 2 节点（同 executor，**过渡态**） | 1 进程 1 节点 |

---

## 4. 公共库 `liblio` 改动（不引入视觉概念）

文件：`third_party/Super-LIO/src/super_lio/include/lio/super_lio.h`、`src/lio/super_lio.cpp`

**已实现**（子模块 commit `f9b77d6`）：

1. 两个 `protected virtual` 空钩子（风格同现有 `ExternalObserve()`）：
   - `virtual void onLioOutput(const OutputData&)` —— odom / body / world 云三路（10 Hz，`OutputThread` 末尾调用）。
   - `virtual void onDenseCloud(const BASIC::CloudPtr&, double stamp)` —— dense 体扫描（`DenseThread` 发布前调用）。
2. 退化 + 观测信息快照：新增 `DegeneracySnapshot{ok, cond, axes, active, dir_w, has_info, info}`
   与 mutex 保护的 `degeneracySnapshot() const`。`Observe()` 中把 `last_HTVH`（原始
   6×6 位姿观测 Hessian，布局 `[rot(0-2), pos(3-5)]`）与退化状态机结果一起写入快照。

**尚未实现**：

3. `virtual void onImuOdom(const NavState&, const V3& body_omega, double stamp)` ——
   IMU 频率 odom。**当前不需要**：B 的传播直订原始 `/livox/imu`，位姿先验用 10 Hz
   `/lio/odom`，已不再订阅 `/lio/imu/odom`。只有后续 mailbox 化时决定把 200 Hz
   NavState 也直供视觉侧，才需要补这个钩子。

> 约束：以上只加空钩子与成员，公共库不出现"视觉"字样，A/B 均可复用，纯 LIO 行为不变。

---

## 5. 公共进程内 mailbox（推模式）

**已实现**：`common/super_livo_common/include/super_livo_common/latest_sample_buffer.hpp`
（header-only 的通用样本缓冲，`push / clear / empty / size / latest / nearest / bracket`）。

**尚未实现**：原计划的聚合型 `lio_output_mailbox.hpp`（单生产者多消费者的 ring buffer，
元素 `{stamp, NavState, body_omega, body_pc, world_pc, dense_pc, degeneracy_sig, lidar_info}`）。
它本来是给 B 的输入侧替换 DDS 订阅用的，但 A 的 S1 最终没有走 mailbox，而是在
`VisualPipeline` 内部用"任务队列 + 单工作线程"直接承接钩子推送：

- 生产者：`OutputThread` / `DenseThread`（10 Hz），只做数据拷贝入队，不阻塞滤波；
- 消费者：`VisualPipeline` 的唯一工作线程，串行处理图像/位姿/点云/退化任务；
- 位姿插值仍沿用各自原有的 `pose_cache_` 逻辑。

结论：通用 mailbox 先做成了最小可用件（`latest_sample_buffer`），聚合型 mailbox 等
B 的输入侧改造时再决定是否需要；不要在 A 已经跑通的路径上再插一层。

---

## 6. 方案 A 改动清单

1. **已新建** `super_livo_visual_a/include/super_livo_visual_a/pipeline/visual_pipeline.hpp` +
   `src/pipeline/visual_pipeline.cpp`（编成独立共享库 `super_livo_visual_a_pipeline`）：
   - 把 `VisualANode` 除订阅/发布外的逻辑搬入；接口改为普通类型（`submitImage` /
     `submitOdom` / `submitWorldCloud` / `submitBodyCloud` / `submitDegeneracy`），
     输出走 `std::function` 回调（彩色云 / odom / 日志）。
   - 内部为**回调入队 / 工作 / 落盘三线程**：`SuperLIO` 的 `OutputThread`/`DenseThread` 与
     图像回调只做数据拷贝入队（有界队列 cap 2048，满则丢最旧并计数），不会被视觉处理或
     写盘阻塞；彩色地图分段移交独立 save 线程，退出统一走幂等 `shutdownAndSave()`。
   - 算法模块（KltFrontend/DepthAssociation/ReprojectionConstraint/DegeneracyGate/
     VisualUpdate/ColorizerPipeline）**一行未改**。
2. **已实现** `VisualASuperLIO`（app 内）覆写 `onLioOutput` / `onDenseCloud` 直推 pipeline；
   `submitDegeneracy` 由 `degeneracySnapshot()` 转换而来（不走 DDS）。
3. **已实现** `visual_a_app.cpp`：不建 `VisualANode`、不
   `executor.add_node(visual)`；图像订阅、彩色云 / odom / path 发布、落盘定时器
   都建在 `ROSWrapper` 上（`setupMonolithIo()`）。
4. ~~观察窗口：参数 `super_livo_visual.lio_source`（`internal` 默认 / `topic` 对照）~~ ——
   S2 已删除 `topic` 对照路径与 `lio_source` 开关，单体为唯一模式。
5. 新增输出能力（commit `d66beb6`）：TUM 轨迹 `evo.*`（`timestamp tx ty tz qx qy qz qw`，
   每 LiDAR 帧一行）与 PNG 图像 `image_save.*`（文件名用 LIO 时域时间戳，固定 6 位小数），
   两者都默认关闭。
6. **可选视觉回灌到 ESKF（尚未接线，默认关闭）**：`VisualASuperLIO::ExternalObserve()`
   目前仍是空实现，正好是注入点。强退化场景下，视觉约束经
   `eskf()->UpdateObserveFull(H, r, R)` 写回（与 B 同机制），由
   `DegeneracyGate::GateDecision` + `DegeneracySnapshot` 的 `severity/dir` 门控
   （`has_info=false` 时必须先过滤，见 §9）。

   预留回灌参数（后续观察 Super-LIO 退化参数 cond/sev 量级再调比例）：

   | 参数 | 默认 | 含义 |
   | --- | --- | --- |
   | `update.inject_to_eskf` | `false` | 总开关（false=纯后处理，与现状逐位一致） |
   | `update.inject_severity_min` | `0.6` | 退化严重度门槛，sev≥此值才允许回灌 |
   | `update.inject_weight` | `1.0` | 视觉权重全局缩放（回灌比例主旋钮） |
   | `update.inject_info_ratio` | `1.0` | 视觉信息量上限 ≤ ratio·λmax(当前 LiDAR 先验) |
   | `update.inject_rot_sigma_floor` | `0.005` | 视觉隐含姿态精度下限（rad） |
   | `update.inject_pos_sigma_floor` | `0.02` | 视觉隐含位置精度下限（m） |
   | `update.inject_max_dt` | `0.2` | 视觉帧↔雷达帧时间差上限（过期丢弃） |
   | `update.inject_dir_gated` | `true` | 是否只在退化方向放开视觉权限（各向异性） |

   > 口径与 B 的 `UnifiedEstimator` 对齐（`visual_info_ratio` / `visual_rot_sigma_floor` / `visual_pos_sigma_floor` / `visual_pose_only`），避免 A/B 两套"视觉可信度"定义打架。

---

## 7. 方案 B 改动清单

**目标形态（与 A 一致）：1 进程 1 节点。** `VisualBNode` 不是最终形态，只作为过渡与
S0 对照保留；最终由 ROS 无关的 `VisualPipeline` 承接它的全部逻辑（图像前端、
`UnifiedEstimator`、彩色落盘），`visual_b_app` 只剩 `ROSWrapper` 一个节点。
下面 1~3 是已完成的过渡步骤，4~8 是收敛到目标形态的剩余工作。

**已完成（commit `b0f5adc`，过渡态）**：节点线程化。

1. `VisualBNode` 拆三条线程：回调线程只 `pushInputEvent`（`input_queue_`，上限 2048，
   超限丢最旧）；工作线程 `workerLoop` 按 FIFO 串行分派 `onImage/onOdom/onCloud/
   onBodyCloud/onImu/onSaveSnapshot`；落盘线程 `saveLoop` 只写彩色 PCD，
   工作线程按"新增段"增量移交（`ColorSegment` + `save_handed_points_`）。
2. 视觉帧调度保持原语义：图像先入 `pending_visual_`（上限 8），等 `stamp +
   img_time_offset` 时刻的位姿到齐再 `processVisualFrame`；`drainPendingVisualFrames`
   在 odom 事件里触发。
   > 已知不一致：`degen_sub_` 仍用 `std::bind(&VisualBNode::onDegeneracy)` 直接绑在
   > executor 上，是唯一没有进 `input_queue_` 的回调。`degen_sig_` 有独立 mutex，
   > 不会崩，但它与工作线程之间没有顺序保证（迟到/早到的签名可能配到错误的帧）。
   > 改造时随"直吃 `degeneracySnapshot()`"一起收敛。
3. 局部修正（未提交）：`odom_topic` 默认改为 10 Hz 的 `lio/odom`（与 A 一致），并在
   `onOdom` 里用 app 注入的 `odom_robo⁻¹` 换回 IMU 系（`setOdomBodyExtrinsic()`）；
   `poseFromSamples()` 改为在 `pose_cache_` 上二分插值、拒绝超出最新采样；
   彩色排空加 0.1 s 节流；新增 `noteSkippedInterval()` 处理 IMU 空洞，
   配合 `UnifiedEstimator` 的 `freeze_bias` 与分块限幅，避免伪零偏把状态泵飞。

**尚未完成**：

4. **新建** `super_livo_visual_b/include/.../pipeline/visual_pipeline.{hpp,cpp}`
   （同理抽离 `VisualBNode`），输入侧从 DDS 订阅换成 mailbox 直供：
   - `/lio/odom`（10 Hz）→ 位姿先验 + 轨迹（`odom_robo` 外参由 app 注入）
   - `/livox/imu` → `UnifiedEstimator::propagate`（原始 IMU，不需要 `onImuOdom` 钩子）
   - `/lio/cloud_world` / `/lio/cloud_body_dense` → world / dense 云
   - `/lio/degeneracy` → 改为直吃 `degeneracySnapshot()`（话题已补发布端，但进程内再绕 DDS 是冗余）
5. **写回方向不动**：`ConstraintBridge` + `ExternalObserve` + `UpdateObserveFull` 保持原样。
6. `visual_b_app.cpp` 改为**单节点**：`VisualBSuperLIO` 覆写 `onLioOutput` /
   `onDenseCloud` 直推 `VisualPipeline`；图像订阅、彩色云 / 相机 odom /
   path 发布、落盘线程都挂在 `ROSWrapper` 上；不再 `add_node(visual)`。
7. ~~观察窗口保留：`super_livo_visual.lio_source` 开关 + `VisualBNode` 对照~~ —— S2 已删除
   （B 无 `lio_source`，直接删 `VisualBNode`；A 同步删 `topic` 分支与 `VisualANode`），
   B 与 A 的差异只剩"写回 ESKF"。

8. **直接吃内部观测信息**：B 的 `UnifiedEstimator::lidarUpdate` 不再从 `lio/odom`
   位姿合成强先验，改为直接消费 `degeneracySnapshot().info`（即 `Observe()` 内的
   6×6 `last_HTVH`），由 B 侧映射到 30 维状态。这是数值口径变更，需用 S0 基线做
   逐位对比确认不劣化。

---

## 8. 实验观察窗口（可拆分阶段）

| 阶段 | A | B | 状态 |
|---|---|---|---|
| S0 基线 | `super_livo_visual.lio_source=topic`（`VisualANode` + DDS） | `VisualBNode` 直接订阅 | A 可用；B 即当前形态 |
| S1 单体 | `super_livo_visual.lio_source=internal`（钩子直推 + `VisualPipeline`） | 单节点 `VisualPipeline` + 内部通信 | A **已完成**；B 未开始 |
| S1.5 过渡 | — | 进程内注入 + 回调/工作/落盘三线程（commit `b0f5adc`） | B **已完成**，仅作过渡，输入仍走 DDS |
| S2 收敛 | 删除 `topic` 分支与 `VisualANode` | 删除 `VisualBNode` 及其 DDS 订阅 | **已完成**（B 先行；A 于本轮删除，`lio_source` 一并移除） |

- A 的 S0/S1 共用同一套算法模块，差异只在数据源（DDS 订阅 vs 钩子直推）；
  同一 bag 下前端指标一致，额外开销约 1 ms/帧（实测见 §9）。
- B 的 S1.5 只解耦了"回调线程"与"计算/落盘"，DDS 订阅本身仍在，**不是设计终点**；
  它证明了三线程模型跟得上 10 Hz 输入，但 B 的目标仍是单进程单节点（S1/S2）。

---

## 9. 验证与回归口径

- `super_livo_visual.embed=false`：纯 LIO 逐位回归（现成保留）。
- S0 vs S1：`tools/latency_probe*`、`tools/odom_vs_gt.py` 比对轨迹与端到端延迟。
- 退化门控：发布端已补（`ROSWrapper::pub_degeneracy()`），S1 后退化签名由内存快照直供
  或经 `lio/degeneracy` 转发，均可验证 A/B 的退化分支不再恒假。见 §9.2 实测：
  四个 bag 都能收到 10 Hz 签名，但场景健康（`active=0`），"强退化才加强回灌"的
  强度分支仍缺真实退化样本，需要构造退化场景（长走廊/隧道/空旷地）再验证。
- 观测信息为空时（`has_info=false`）`snap.cond` 会到 `1e12`、`deg` 到 `1.0`，
  回灌前必须先按 `has_info` 过滤（发布端已按 `snap.ok && snap.has_info` 过滤，
  不满足时发全 0，B 侧 `valid()` 为 false ⇒ `sev=0`）。

### 9.1 已实测（Retail_Street，135.5 s / 1355 帧 LiDAR / 10 Hz 图像）

B 的 S1.5（线程化 + 进程内注入），视觉注入 off / on 两轮，同 bag、同参数：

| 项 | OFF | ON |
| --- | --- | --- |
| ERROR / 输入队列溢出 | 0 / 0 | 0 / 0 |
| odom 行数 / 时长 | 1305 / 133.5 s | 1306 / 133.6 s |
| 轨迹长度 / 回环残差 | 66.82 m / 0.02 m | 66.84 m / 0.03 m |
| 最大速度 | 1.14 m/s | 1.49 m/s |
| 视觉帧处理 | 1352（严格 10 Hz） | 1353 |
| 单帧 avg / max | 21.1 / 34.9 ms | 20.9 / 30.3 ms |
| 注入生效 | — | `push=1347 take=1308 生效=1308 被拒=0 过期=0` |

- 单帧预算：前端≈8.9 ms、约束≈8.7~8.9 ms、更新≈0.6 ms、彩色≈2.2~2.9 ms；
  100 ms 帧间隔有约 5 倍余量，`pend_vis=0, pend_cloud=0`。
- ON vs OFF 时间轴对齐（1304 点）：中位偏差 2.62 cm、p95 4.32 cm、最大 5.55 cm、
  终点差 0.28 cm ⇒ 视觉只是微调，没有把本已很好的解带偏。
- 单帧注入量 `dxp` 多数 0~0.1 mm（最大 1.0 mm），`λ≈5.9e6 → scale≈0.0068`；
  即当前注入强度只由 LIO 观测信息 λ 决定，与退化严重度无关（sev 恒 0）。
- 局限：Retail_Street 全段结构化良好，本来就不该触发退化，本轮只能证明
  "非退化场景不损伤"，不能证明"退化场景能救回来"。

### 9.2 已实测（livo_bag 四包，按已知标定）

形态：单进程 B（`visual_b_app`，`super_livo_visual.inject=true`），参数 `lio_move.yaml` +
`visual_b_move.yaml`；图像时钟按已知标定换算到雷达时钟：
（下表跑批在参数改名前执行，当时命令行写的是 `app.visual_inject` / `app.visual_inject_max_dt`）

| bag | 图像 → 雷达时钟偏移 | 时长 | degen 条数 | cond6 p50（min–max） | active | 视觉帧处理 | push / 生效 | ERROR |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| all_ros2 | −204262819.04 | 41 s | 392 | 4.78（4.43–5.43） | 0 | 406 | 386 / 383 | 0 |
| move_ros2 | −204262819.03 | 77 s | 754 | 4.04（2.81–1e12） | 0 | 737 | 690 / 673 | 0 |
| move_livoxclk | 0（yaml） | 77 s | 765 | 4.05（2.82–5.24） | 0 | 741 | 688 / 681 | 0 |
| test4_move | −204701439.32 | 90 s | 868 | 1.88（1.44–2.92） | 0 | 778 | 769 / 331（过期 308） | 0 |

- 发布端按 10 Hz 出数，`cond6` 随场景变化（test4_move 明显更"好"：1.4–2.9），
  但四包都 `active=0`，`退化 sev` 仍为 0 ⇒ 门控只验证了"不误触发"，强度分支仍缺样本。
- `move_ros2` 有一帧 `cond6=1e12`（平移块 min eig≈0 的截断值），此时 B 由 cond
  对数映射出的 severity 会到 1.0；因 LIO 状态机 `active=0`、下一帧即恢复，10 s 采样日志
  没抓到。若要更稳，B 侧应 require `active` 或对 `cond=1e12` 做白名单处理。
- A（`internal`，同四包）仍 `n=0`（tracked 86–150、合计 4.8–16 ms），而 B 同包
  `obs=133–179`。A 的深度关联在这四个包上没出约束，属既有问题，与本次发布端无关，
  需单独排查（相机-LiDAR 外参/时延与外参校验）。
- `test4_move` 是 mcap；本机 ROS 环境没有 rosbag2 的 mcap storage 插件，
  先用 `rosbags-convert` 转 sqlite3，并把 `metadata.yaml` 改写为 rosbag2 的 v4 结构
  （`offered_qos_profiles: ""`）才能回放。转出的 3.4 GB 在
  `/home/ywj/slam_data/output/bag_convert/test4_move`。

### 9.3 已实测（test20261118，odom 切 10 Hz 与注入门控）

形态：单进程 `visual_b_app`，`lio_move.yaml` + `visual_b_move.yaml`，四轮同 bag 对照。
详细表格见 [monolith_refactor_progress.md](monolith_refactor_progress.md) §3.5，结论：

- `odom_topic: lio/odom`（10 Hz）配合 `odom_robo⁻¹` 换回 IMU 系后，该 bag 上漂移消失
  （此前 200 Hz `/lio/imu/odom` 那版 `_fix12` 仍有漂移）。
- odom 相邻间隔 87.6–94.5 ms，输入队列深度 max 8 / 溢出 0，`[PROBE-D]` 位姿失败 0 /
  空洞 13，着色 1898 帧。
- 取用时差是结构性的：等位姿 avg≈164 ms + 前端 28–29 ms + 约束 12 ms + 等下一次
  雷达更新（≤100 ms），实测 avg≈0.32–0.35 s。`[年龄]` 直方图显示 0.2 s 门限只能接
  34.4%、0.4 s 能接 71.4%；两档生效数 521 vs 1089，与直方图一致。
- 两档成图逐格对比一致（5 cm 栅格 IoU 0.88、点数 8.289M vs 8.298M、范围相同、无重影），
  故 `super_livo_visual.inject_max_dt` 定为 `0.4`，不再上调。降时差应减结构延迟，而不是放门限。
- 跨运行的"过期数"不可比（bag 回放抖动会让时差整体后移），判据固定用同一趟数据的
  `[年龄]` 分布。

---

## 10. 风险与回滚

- **公共库改动是加法**：空钩子 + 成员 + 访问器，纯 LIO 路径不受影响，可独立编译验证。
- **内存上界**：A/B 的 pipeline 输入队列上限 2048 条事件（一帧原图 4 MB，不设限会吃几十 GB），
  超限丢最旧并计数；dense 云走共享指针避免拷贝。
- **线程安全**：A/B 均为"单工作线程独占全部视觉状态"+ 独立落盘线程；
  仅 `degen_sig_` 有独立 mutex（退化回调仍直接绑在 executor 上，见 §7 已知不一致）。
- **回滚**：S2 已完成，`lio_source=topic` 对照路径与两个旧节点均已删除，无进程内回滚开关；
  只能靠 `super_livo_visual.embed` / `super_livo_visual.inject` 关闭视觉，或回退 git 历史。

---

## 11. 建议实施顺序

分两条线：**主线（去 DDS 单体化）** 与 **功能线（回灌/直吃内部观测）**。功能线互相独立、且都依赖主线 S1 单体稳定后再开，避免混在一起难以回归定位。

### 主线：去 DDS 单体化
1. ~~公共库钩子 + 退化/观测信息快照（`liblio`）~~ —— **已完成**（`f9b77d6`）。
2. ~~公共 mailbox（common）~~ —— **最小件已完成**（`latest_sample_buffer.hpp`）；
   聚合型 mailbox 未做，A 改用 pipeline 内部任务队列。
3. ~~方案 A：抽 `VisualPipeline` + 单节点接线 + `super_livo_visual.lio_source` 开关；S0/S1 对比~~ —— **已完成**（`b284f93`、`d66beb6`）。
4. 方案 B：抽 `VisualPipeline` + 输入侧改为内部直推（保留 `ConstraintBridge`）；
   目标是**单进程单节点**，S0/S1 对比。
   - 已完成中间态（`b0f5adc`）：进程内注入 + 回调/工作/落盘三线程（过渡，不是终点）。
   - 已完成（未提交）：odom 源切到 10 Hz `/lio/odom` + `odom_robo` 外参换算，
     `poseFromSamples()` 二分插值、彩色排空节流、`visual_inject_max_dt: 0.4`
     与 `[年龄]` 探针（§9.3）。
   - 未完成：抽 `VisualPipeline`、输入侧去 DDS、退化签名直吃快照、删除 `VisualBNode`。
5. ~~稳定后进入 S2：删除 `topic` 对照路径与两个旧节点~~ —— **已完成**（B 先行；A 的
   `visual_a_node.{hpp,cpp}`、`visual_a_node_main.cpp` 与 `lio_source` 已删，A pipeline
   同步线程化为回调入队/工作/落盘三线程）。

### 功能线 A：可选视觉回灌（§6.5，依赖主线 S1 后）
6. 先观察 S1 单体内退化参数（cond/sev）实际量级，标定 `inject_severity_min` / `inject_info_ratio`。
   - 注意：本机 Retail_Street 全段健康、`sev=0`，拿不到退化样本；
     A 有快照可直接采；B 所需的 `/lio/degeneracy` 发布端已补（§2.3）。
7. 移植 B 的"视觉增量 → 信息形式约束"映射（白化 + 世界左扰动→机体系右扰动），在 `ExternalObserve` 接 `UpdateObserveFull`。
8. 默认 `inject_to_eskf=false` 逐位回归；再逐参数打开，跑强退化场景验证纠正力。

### 功能线 B：直吃内部观测信息（§7.6，依赖主线 S1 后）
9. `UnifiedEstimator::lidarUpdate` 由"lio/odom 合成强先验"改为直吃 `degeneracySnapshot().info`，先 S0 基线逐位对比确认不劣化，再进 S2。

> 主线 3 已交付；主线 4 与功能线 6–9 各自独立，可按需穿插，但都必须在对应方案 S1 单体稳定之后。
> B 的 `/lio/degeneracy` 发布端已补（`ROSWrapper::pub_degeneracy()`，8 元素
> `Float64MultiArray`，布局见 `DegeneracySignature`），这只是过渡；
> 最终 B 应与 A 一样直读内存快照。
