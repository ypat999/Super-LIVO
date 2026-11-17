# Super-LIO 视觉支路单体化整改方案（A + B）

> 状态：规划稿（未实施）
> 范围：`solution_a_lightweight` + `solution_b_tight_coupled` 两条视觉支路，以及公共库 `third_party/Super-LIO`（`liblio`）。
> 目标：把"一个进程内两个 ROS 节点靠 DDS 回环取 LIO 输出"改成"单节点、进程内推式共享缓冲"，视觉逻辑退化为 ROS 无关的流水线类。ROS 只保留外部输入（LiDAR/IMU/图像）与对外输出（可视化/录制）。

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
- `visual_a_app.cpp` 同进程创建 `ROSWrapper`（主干）+ `VisualANode`（视觉），两者都进同一个 `MultiThreadedExecutor(6)`。
- `VisualANode` 订阅 `lio/odom`、`lio/cloud_world`、`lio/cloud_body_dense`、`lio/degeneracy`、图像。
- 视觉修正**不写回 ESKF**：用 `pose_offset_` 左乘（带泄漏衰减）或 `update.reinject` 写回 `own_pose_cache_`，发布 `super_livo_visual_a/odom`。
- 结论：A 现状是后处理，滤波器不感知视觉；本方案为其预留**可选视觉回灌**（§6.5），默认关闭保持轻量，强退化场景可逐步打开。

### 2.2 方案 B（紧耦合，代码级写回 ESKF）
写回链路（**已非 DDS**）：

```
图像线程 VisualBNode::publishVisualConstraint()
  → 白化 + 符号翻转 + 世界左扰动→机体系右扰动 (H18 = JwT·R_wi)
  → ConstraintBridge::push()                     // 单槽互斥信箱
process 线程 VisualBSuperLIO::ExternalObserve()
  → bridge_->take() + 时间戳新鲜度检查
  → eskf()->UpdateObserveFull(H, r, R)           // 18 维误差状态迭代更新
```

- 视觉约束由 B 自带的 `UnifiedEstimator`（30 维影子滤波器）产出：它订阅 `/lio/imu/odom`（IMU 传播）、把 `/lio/odom` 当强先验做 `lidarUpdate`、再 `visualUpdate`，输出 6×6 `appliedVisual()`。
- 结论：B 的"写回"已经是直接调用，**要改的是它的输入侧**（odom/imu/cloud/degen 仍走 DDS）。

### 2.3 共性痛点
两条支路都存在"**取 LIO 输出仍走 DDS 回环**"的问题；且 `/lio/degeneracy` 在当前树里**根本未被 Super-LIO 发布**（退化状态是 `SuperLIO` 的 protected 成员，从未 pub 成话题），退化门控实为死代码。

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

- 内部数据单向：`SuperLIO` 各输出线程 → mailbox → `VisualPipeline`。
- B 额外保留 `ConstraintBridge`（写回方向，已经是代码级，不动）。
- 对外输出（可视化/录制）仍经 `ROSWrapper` 发布，属输出而非内部通信。

---

## 4. 公共库 `liblio` 改动（不引入视觉概念）

文件：`third_party/Super-LIO/src/super_lio/include/lio/super_lio.h`、`src/lio/super_lio.cpp`

1. 新增三个 `protected virtual` 空钩子（风格同现有 `ExternalObserve()`）：
   - `virtual void onLioOutput(const OutputData&)` —— odom / body / world 云三路（10 Hz，来自 `OutputThread`）。
   - `virtual void onDenseCloud(const CloudPtr&, double stamp)` —— dense 体扫描（来自 `DenseThread`）。
   - `virtual void onImuOdom(const NavState&, const V3& body_omega, double stamp)` —— IMU 频率 odom（来自 `imuOutputThread`，B 的 `UnifiedEstimator` 传播用）。
2. 退化签名暴露：新增成员 `Eigen::Vector3d degeneracy_dir_w_` + mutex 保护的 `degeneracySignature()` 返回 `{cond, axes, active, dir_w}`。
3. 观测信息矩阵暴露：新增成员/访问器，把 `Observe()` 内结算出的 6×6 位姿观测信息矩阵 `last_HTVH`（原始平面 Hessian，未经截断/正则化，布局 `[rot(0-2), pos(3-5)]`）暴露给派生类，供 B 的 `UnifiedEstimator::lidarUpdate` 直吃。
4. 调用点：
   - `OutputThread` 取出 `data` 后 → `onLioOutput(data)`（推）。
   - `pub_cloud_body_dense` 处 → `onDenseCloud(...)`（推）。
   - `imuOutputThread` 高频路 → `onImuOdom(...)`（推，仅 B 需要）。
   - 退化结算处（`degeneracy_cond_/axes_/active_` 写入处）→ 存 `deg.eV.col(0)` 为 `degeneracy_dir_w_`。
   - `Observe()` 结算 `last_HTVH` 处 → 存入观测信息成员（供 §7 直吃）。

> 约束：以上只加空钩子与成员，公共库不出现"视觉"字样，A/B 均可复用，纯 LIO 行为不变。

---

## 5. 公共进程内 mailbox（推模式）

文件：`common/super_livo_common/include/super_livo_common/lio_output_mailbox.hpp`

- 单生产者多消费者的 mutex 保护 ring buffer，元素：
  `{stamp, NavState, body_omega, body_pc, world_pc, dense_pc, degeneracy_sig, lidar_info}`。
- 三个生产者：`onLioOutput`（10 Hz 位姿+云+退化+观测 Hessian）、`onDenseCloud`（dense 云）、`onImuOdom`（IMU 频 odom，仅 B）。
- 保留最近 K 帧 + 时间戳，供视觉端按图像时刻插值（复用现有 `pose_cache_` 插值逻辑）。
- 接口：`pushXxx(...)` / `sampleAt(stamp, ...)` / `latest(...)`。

---

## 6. 方案 A 改动清单

1. **新建** `super_livo_visual_a/include/.../pipeline/visual_pipeline.{hpp,cpp}`：
   - 把 `VisualANode` 除订阅/发布外的逻辑搬入；接口改为普通类型：`onImage(gray, stamp)`、`setLioSource(mailbox)`，输出走 `std::function` 回调。
   - 算法模块（KltFrontend/DepthAssociation/ReprojectionConstraint/DegeneracyGate/VisualUpdate/ColorizerPipeline）**不动**。
2. `VisualASuperLIO`（app 内）覆写 `onLioOutput` / `onDenseCloud` → 推入 mailbox。
3. `visual_a_app.cpp`：**不建 `VisualANode`、不 `executor.add_node(visual)`**；相机订阅建在 `ROSWrapper` 上，回调直调 `pipeline.onImage`；输出经回调转发到 `ROSWrapper` 发布。
4. 观察窗口保留：参数 `app.lio_source`（`internal`/`topic`），`topic` 分支保留 `VisualANode` 作对照（见 §8）。
5. **可选视觉回灌到 ESKF（新增，默认关闭）**：`VisualASuperLIO::ExternalObserve()` 目前是空实现，正好是注入点。强退化场景下，视觉约束经 `eskf()->UpdateObserveFull(H, r, R)` 写回（与 B 同机制），由 `DegeneracyGate::GateDecision` + `DegeneracySignature` 的 `severity/dir` 门控。

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

1. **新建** `super_livo_visual_b/include/.../pipeline/visual_pipeline.{hpp,cpp}`（同理抽离 `VisualBNode`）。
2. 输入侧全部改从 mailbox 拉取，替代现有订阅：
   - `/lio/odom` → mailbox odom（强先验 + 轨迹）
   - `/lio/imu/odom` → mailbox imu odom（`UnifiedEstimator::propagate`）
   - `/lio/cloud_world` → mailbox world cloud
   - `/lio/cloud_body_dense` → mailbox dense cloud
   - `/lio/degeneracy` → mailbox degeneracy（随 odom 一起推，见 §5）
3. **写回方向不动**：`ConstraintBridge` + `ExternalObserve` + `UpdateObserveFull` 保持原样。
4. `visual_b_app.cpp`：单节点接线，`VisualBSuperLIO` 覆写 `onLioOutput`/`onDenseCloud`/`onImuOdom` 推 mailbox；相机订阅在 `ROSWrapper` 上。
5. 观察窗口保留：`app.lio_source` 开关，`topic` 分支保留 `VisualBNode` 作对照。

6. **直接吃内部观测信息（本期做）**：B 的 `UnifiedEstimator::lidarUpdate` 不再从 `lio/odom` 位姿合成强先验，改为直接消费 Super-LIO `Observe()` 内的 6×6 观测信息矩阵 `last_HTVH`（§4 暴露，随 `onLioOutput` 推入 mailbox），由 B 侧映射到 30 维状态。这是数值口径变更，需在 §9 用 S0 基线做逐位对比确认不劣化。

---

## 8. 实验观察窗口（可拆分阶段）

| 阶段 | 数据源 | 用途 |
|---|---|---|
| S0 基线 | `app.lio_source=topic`（原 DDS 回环） | 保持现状，可回滚对照 |
| S1 单体 | `app.lio_source=internal`（mailbox） | 目标形态，A/B 对比 |
| S2 收敛 | 删除 `topic` 分支与 `VisualANode`/`VisualBNode` | 稳定后清理 |

- 两套数据源共用同一个 `VisualPipeline`，仅数据源适配器不同（`MailboxSource` vs `TopicSource`）。
- S1 与 S0 跑同一 bag，比对 ATE / 延迟，确认耦合化不引入数值差异后再进 S2。

---

## 9. 验证与回归口径

- `app.embed_visual=false`：纯 LIO 逐位回归（现成保留）。
- S0 vs S1：`tools/latency_probe*`、`tools/odom_vs_gt.py` 比对轨迹与端到端延迟。
- 退化门控：S1 后 `/lio/degeneracy` 由 mailbox 直供，验证 A/B 的退化分支不再恒假。

---

## 10. 风险与回滚

- **公共库改动是加法**：空钩子 + 成员 + 访问器，纯 LIO 路径不受影响，可独立编译验证。
- **mailbox 内存**：dense 云较大，ring buffer 需设上限并按需持有 `CloudPtr`（shared_ptr，避免拷贝）。
- **线程安全**：生产者多线程（Output/Dense/imuOutput），mailbox 用单锁；消费者只读，不做跨线程写回。
- **回滚**：任何阶段可切回 `app.lio_source=topic`，`VisualANode`/`VisualBNode` 在 S2 前保持可用。

---

## 11. 建议实施顺序

分两条线：**主线（去 DDS 单体化）** 与 **功能线（回灌/直吃内部观测）**。功能线互相独立、且都依赖主线 S1 单体稳定后再开，避免混在一起难以回归定位。

### 主线：去 DDS 单体化
1. 公共库钩子 + 退化/观测信息访问器（`liblio`，编译验证纯 LIO 不变）。
2. 公共 mailbox（common）。
3. 方案 A：抽 `VisualPipeline` + 单节点接线 + `app.lio_source` 开关；S0/S1 对比。
4. 方案 B：抽 `VisualPipeline` + 输入侧 mailbox 化（保留 ConstraintBridge）；S0/S1 对比。
5. 稳定后进入 S2：删除 `topic` 对照路径与两个旧节点。

### 功能线 A：可选视觉回灌（§6.5，依赖主线 S1 后）
6. 先观察 S1 单体内 Super-LIO 退化参数（cond/sev）实际量级，标定 `inject_severity_min` / `inject_info_ratio`。
7. 移植 B 的"视觉增量 → 信息形式约束"映射（白化 + 世界左扰动→机体系右扰动），在 `ExternalObserve` 接 `UpdateObserveFull`。
8. 默认 `inject_to_eskf=false` 逐位回归；再逐参数打开，跑强退化场景验证纠正力。

### 功能线 B：直吃内部观测信息（§7.6，依赖主线 S1 后）
9. `UnifiedEstimator::lidarUpdate` 由"lio/odom 合成强先验"改为直吃 `last_HTVH`，先 S0 基线逐位对比确认不劣化，再进 S2。

> 主线 1–5 可独立交付；功能线 6–9 各自独立，可按需穿插，但都必须在对应方案 S1 单体稳定之后。
