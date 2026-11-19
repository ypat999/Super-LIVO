# 视觉支路单体化（方案 A / B）：设计与实施

> 状态：**已完成**（2026-11-19）。A、B 两条视觉支路均为 **单进程单节点**：
> 一个 `rclcpp::Node`（`ROSWrapper`）+ 一个 ROS 无关的 `VisualPipeline`，
> LIO 输出经进程内钩子直推，不再走 DDS 回环。
> 本文合并原 `monolith_refactor_plan.md`（设计）与 `monolith_refactor_progress.md`（实施/验证）。

---

## 1. 目标与非目标

### 目标
1. LIO → 视觉链路**不再走 DDS**，改为同进程函数调用/入队。
2. 视觉支路从 `rclcpp::Node` 抽成 **ROS 无关的 `VisualPipeline`**，可脱离 ROS 单测。
3. A、B 共用同一套公共钩子与公共件，算法模块一行不改。
4. 不得改变滤波/约束的数学口径（同一 bag 轨迹逐位可比）。

### 非目标
- 不把 LiDAR/IMU/图像这些外部源拉进进程（仍走 ROS 订阅）。
- 不动 KLT 前端、深度关联、重投影、退化门控、视觉更新、着色等算法。
- 不改 `ConstraintBridge` 的写回数学（白化、符号、扰动系）。

---

## 2. 最终形态

### 2.1 进程与数据流

```
外部 ROS（不变）
  LiDAR ─┐
  IMU   ─┼─► ROSWrapper（唯一 rclcpp::Node）─► SuperLIO（主干，liblio）
  图像  ─┘        │  │                            │ OutputThread / DenseThread
                  │  │ 进程内钩子 + 任务队列        ▼
                  │  └──────────────────► VisualPipeline（ROS 无关类）
                  │                              │
                  └── 对外发布 ◄──────────────────┘ 彩色云 / 修正 odom / path
```

- 内部单向：`SuperLIO` 输出线程 → `onLioOutput` / `onDenseCloud` / `degeneracySnapshot()`
  → `VisualPipeline` 入队。
- 对外输出（彩色云、odom、path、落盘）只经 `ROSWrapper` 发布，属输出而非内部通信。
- B 额外保留 `ConstraintBridge`（视觉 → LIO 的写回方向，本来就是函数调用）。

### 2.2 代码结构

| 层 | 方案 A | 方案 B |
| --- | --- | --- |
| 可执行入口 | `solution_a_lightweight/super_livo_visual_a/src/app/visual_a_app.cpp` | `solution_b_tight_coupled/super_livo_visual_b/src/app/visual_b_app.cpp` |
| 流水线 | `.../include|src/pipeline/visual_pipeline.*`（不依赖 rclcpp） | `.../include|src/pipeline/visual_pipeline.*`（不依赖 rclcpp） |
| 算法 | `frontend/klt_frontend`、`constraint/{depth_association,reprojection_constraint,degeneracy_gate}`、`update/visual_update` | `frontend/{patch_tracker,feature_lifecycle}`、`constraint/{lidar_visual,msckf,photometric}_constraint`、`estimator/{unified_estimator,...}`、`feature_map/*`、`bridge/constraint_bridge.hpp` |
| 参数源 | ROS 参数（`vp()` 前缀 helper） | `ParamSource` 抽象 + `RosParamSource` + `CalibrationOverlayParamSource` |
| 节点数 | 1 进程 1 节点 | 1 进程 1 节点 |

已删除：`visual_a_node.{hpp,cpp}`、`visual_a_node_main.cpp`、`visual_b_node.{hpp,cpp}`、
`super_livo_visual.lio_source` 开关。

### 2.3 A 与 B 的唯一差异

| 环节 | A（轻量） | B（紧耦合） |
| --- | --- | --- |
| 视觉结果去向 | 后处理：`pose_offset_` 左乘 + 缓变衰减，发布自己的 odom | 白化后经 `ConstraintBridge` → `ExternalObserve()` → `UpdateObserveFull` 写回 ESKF |
| 前端 | KLT 光流 + 深度关联 + 重投影 | Patch 跟踪 + 30 维影子滤波器（`UnifiedEstimator`） |
| 位姿先验 | 钩子直推的 LIO 输出 | IMU 传播（`/livox/imu`）+ 10 Hz `/lio/odom` 先验 |
| 单帧预算（test4） | ~10 ms | ~50 ms |

---

## 3. 公共库与公共件

### 3.1 `liblio` 钩子与退化快照（子模块 `third_party/Super-LIO`，`f9b77d6`）

纯加法，未覆写时纯 LIO 逐位不变：

| 位置 | 内容 |
| --- | --- |
| `include/lio/super_lio.h` | `virtual void onLioOutput(const OutputData&)`（`OutputThread` 末尾，10 Hz，含 odom/body/world 云）<br>`virtual void onDenseCloud(const CloudPtr&, double stamp)`（`DenseThread` 发布前）<br>`DegeneracySnapshot{ok, cond, axes, active, dir_w, has_info, info}` + `degeneracySnapshot() const` |
| `src/lio/super_lio.cpp` | 两处钩子调用；`Observe()` 把 `last_HTVH`（6×6 位姿观测 Hessian，布局 `[rot(0-2), pos(3-5)]`）与退化状态机结果写入快照（mutex 保护） |

未实现 `onImuOdom`：B 的传播直订原始 IMU、先验用 10 Hz odom，不需要 200 Hz `NavState`。

### 3.2 公共件 `super_livo_common`

| 文件 | 用途 |
| --- | --- |
| `latest_sample_buffer.hpp` | header-only 时序样本缓冲（`push/latest/nearest/bracket`） |
| `calibration_source.hpp` | A/B 共用标定 loader：`loadCameraIntrinsic`（`cv::FileStorage`）、`loadFastCalib`（`key: value` 文本，Rcl/Pcl 跨行）、`mergeCalibration` |
| `degeneracy_signature.hpp` | `/lio/degeneracy` 8 元素数组 ↔ 结构体（过渡期发布端使用） |
| `latency_timer.hpp` | 端到端延时统计（`[Img->VisCloud]` / `[Img->VisOdom]`），默认关闭 |
| `anisotropic_prior.hpp` / `measurement_noise.hpp` | 各向异性先验与噪声模型 |

原计划的聚合型 `lio_output_mailbox.hpp` **未建**：A/B 都在 pipeline 内用"有界任务队列 + 单工作线程"，
再插一层 mailbox 属冗余。

---

## 4. 方案 A：单体后处理

### 4.1 接线

- `visual_a_app` 只建 `ROSWrapper`，`VisualASuperLIO` 覆写 `onLioOutput` / `onDenseCloud` 直推 pipeline；
  退化签名由 `degeneracySnapshot()` 转换后入队。
- 图像订阅（bgr8 零拷贝浅拷贝 + 消息 keepalive）、彩色云/odom/path 发布、周期落盘都挂在 `ROSWrapper`。
- 视觉修正不写回 ESKF：`pose_offset_`（带泄漏衰减）或 `update.reinject` 写入 `own_pose_cache_`，
  对外发布 `/super_livo_visual_a/odom`。

### 4.2 三线程与退出

| 线程 | 职责 |
| --- | --- |
| 回调线程 | 只拷贝入队 `InputEvent`（cap 2048，满则丢最旧并计数），O(µs) |
| 工作线程 | 串行执行位姿/图像/点云/退化任务，独占全部视觉状态（无锁） |
| 落盘线程 | 只写彩色地图；工作线程按 `ColorSegment` 增量移交，常驻缓冲合并后写 PCD |

退出：`flush()` barrier（30 s）+ 幂等 `shutdownAndSave()`（`call_once`：停线程 → 排空 → 同步存盘 → 关轨迹）。

### 4.3 输出

彩色 PCD、`/super_livo_visual_a/odom`、path；可选 TUM 轨迹（`evo.*`）、PNG 图像（`image_save.*`），
默认关闭。

### 4.4 未做：视觉回灌 ESKF

`VisualASuperLIO::ExternalObserve()` 仍是空实现，预留参数（尚未进入 `VisualPipelineOptions`）：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `update.inject_to_eskf` | `false` | 总开关（false=纯后处理） |
| `update.inject_severity_min` | `0.6` | 退化严重度门槛 |
| `update.inject_weight` | `1.0` | 视觉权重全局缩放 |
| `update.inject_info_ratio` | `1.0` | 视觉信息量上限 ≤ ratio·λmax(LiDAR 先验) |
| `update.inject_rot_sigma_floor` | `0.005` | 视觉隐含姿态精度下限（rad） |
| `update.inject_pos_sigma_floor` | `0.02` | 视觉隐含位置精度下限（m） |
| `update.inject_max_dt` | `0.2` | 视觉帧↔雷达帧时差上限 |
| `update.inject_dir_gated` | `true` | 只在退化方向放开视觉权限 |

口径与 B 的 `UnifiedEstimator` 对齐。注意 `has_info=false` 时快照 `cond` 会截断到 `1e12`，
回灌前必须按 `has_info` 过滤（发布端已按 `snap.ok && snap.has_info` 过滤）。

---

## 5. 方案 B：紧耦合写回

### 5.1 写回链路（已是函数调用）

```
工作线程 publishVisualConstraint()
  → 白化 + 符号翻转 + 世界左扰动→机体系右扰动（H18 = JwT·R_wi）
  → ConstraintBridge::push()                      // 单槽互斥信箱
Super-LIO 处理线程 VisualBSuperLIO::ExternalObserve()
  → bridge_->take() + |obs.stamp − ESKF 时刻| ≤ inject_max_dt
  → eskf()->UpdateObserveFull(H, r, R)            // 18 维迭代更新
```

### 5.2 输入与时间对齐

- 位姿先验：10 Hz `/lio/odom`（base_link 系），app 注入 `odom_robo⁻¹` 换回 IMU 系；
  `poseFromSamples()` 在 `pose_cache_` 上二分插值，拒绝超出最新采样。
- IMU 传播：直订 `/livox/imu`；IMU 空洞时 `freeze_bias` + 分块限幅，避免伪零偏。
- 退化签名：`degeneracySnapshot()` 直推（不再订 `/lio/degeneracy`）。
- 图像按 `stamp + img_time_offset` 等位姿到齐后处理（`pending_visual_` 上限 8）。

### 5.3 三线程

回调入队 / 工作线程 FIFO（`pending_visual_` 调度 + 着色）/ 落盘线程（增量 PCD）。
起因：原 6 订阅 + 落盘同在一个互斥回调组，一次 47 MB PCD 落盘阻塞 855~980 ms，
压爆 `pending_visual_` 白丢 293 帧。

### 5.4 参数源

`ParamSource` 接口（`getBool/getInt/getDouble/getFloat/getString/getDoubleArray`），
`RosParamSource` 统一注入 `super_livo_visual.` 前缀；`CalibrationOverlayParamSource`
在命中相机内参/畸变/外参时返回 `camera_intrinsic.yaml` / FAST-Calib 的值，其余委托数据集 yaml。

---

## 6. 参数与配置规范

全仓 params YAML 顶层统一：

```yaml
/**:
  ros__parameters:
    lio: ...                 # 与原版 Super-LIO 完全一致，可直接喂 super_lio_node
    super_livo_visual: ...   # 视觉支路（A/B 共用同类键名）
```

- rcl 会摊平成 `super_livo_visual.<name>`，代码侧一律带前缀读取。
- A/B 同类参数已合并改名：B 的 `lv.*` → `reproj./update./depth.*`；
  `est.enable_visual` → `update.enable`；`app.embed_visual` → `embed`；
  `app.visual_inject` → `inject`；`app.visual_inject_max_dt` → `inject_max_dt`。
- 每个 app 吃两份 yaml（LIO 主干 + 视觉支路），两份都必须满足上面的顶层格式。
  同一份文件也能直接起官方 `super_lio_node` 跑纯 LIO。
- 标定覆盖：`calib.camera_intrinsic_yaml`、`calib.fast_calib_result` 留空则沿用 yaml 值。

---

## 7. 实施阶段

| 阶段 | A | B | 状态 |
| --- | --- | --- | --- |
| S0 基线 | `VisualANode` + DDS 订阅 | `VisualBNode` 直接订阅 | 历史 |
| S1 单体 | 钩子直推 + `VisualPipeline` | 单节点 + 内部通信 | **完成** |
| S1.5 过渡 | — | 进程内注入 + 三线程（输入仍 DDS） | **完成**（`b0f5adc`） |
| S2 收敛 | 删 `VisualANode` / `lio_source`、pipeline 三线程化 | 删 `VisualBNode` 与 DDS 订阅 | **完成**（2026-11-19） |

---

## 8. 验证

### 8.1 编译

```bash
colcon build --packages-select super_livo_common super_lio super_livo_visual_a \
  super_livo_visual_b pointcloud_colorizer --cmake-args -DCMAKE_BUILD_TYPE=Release
```

2026-11-19 拉取上游后（`588cbc2`）实测：`super_livo_common` + A + B 全通过，约 1 min 18 s，0 error。

### 8.2 A：S0 vs S1（同 bag 同配置）

| 项 | S0（DDS） | S1（钩子直推） |
| --- | --- | --- |
| 跟踪特征 | `tracked=115/54` | `tracked=115/45` |
| 单帧耗时 | 5.2 ms | 6.4 ms |
| 上色 | 1463/17790 点，帧 #425 | 1477/17816 点，帧 #425 |
| 错误 | 无 | 无 |

结论：功能等价，额外开销约 1 ms/帧。该 bag 上 A 的 `n=0`（无有效深度约束）是既有外参/时延问题，
非单体化引入（B 同包 `obs=133–179`，见 §8.4）。

### 8.3 B：Retail_Street 全量（135.5 s / 1355 帧）

注入 OFF / ON 两轮，同 bag 同参数：

| 项 | OFF | ON |
| --- | --- | --- |
| ERROR / 队列溢出 | 0 / 0 | 0 / 0 |
| odom 行数 / 时长 | 1305 / 133.5 s | 1306 / 133.6 s |
| 轨迹长度 / 回环残差 | 66.82 m / 0.02 m | 66.84 m / 0.03 m |
| 视觉帧处理 | 1352（严格 10 Hz） | 1353 |
| 单帧 avg / max | 21.1 / 34.9 ms | 20.9 / 30.3 ms |
| 注入 | — | `push=1347 take=1308 生效=1308 被拒=0 过期=0` |

ON vs OFF 时间轴对齐（1304 点）：中位 2.62 cm、p95 4.32 cm、最大 5.55 cm、终点差 0.28 cm
⇒ 视觉只做微调，未把好解带偏。单帧注入 `dxp` 多数 0~0.1 mm。

### 8.4 退化发布端 + livo_bag 四包

`ROSWrapper::pub_degeneracy()`（过渡）随 10 Hz odom 发 8 元素 `Float64MultiArray`，
`snap.ok && snap.has_info` 不成立时发全 0。

| bag | 时长 | degen 条数 | cond6 p50（min–max） | active | 视觉帧 | push / 生效 |
| --- | --- | --- | --- | --- | --- | --- |
| all_ros2 | 41 s | 392 | 4.78（4.43–5.43） | 0 | 406 | 386 / 383 |
| move_ros2 | 77 s | 754 | 4.04（2.81–1e12） | 0 | 737 | 690 / 673 |
| move_livoxclk | 77 s | 765 | 4.05（2.82–5.24） | 0 | 741 | 688 / 681 |
| test4_move | 90 s | 868 | 1.88（1.44–2.92） | 0 | 778 | 769 / 331（过期 308） |

四包 `active=0`、`sev=0`：只验证了"健康场景不误触发"，强度分支仍缺真实退化样本。
`move_ros2` 有一帧 `cond6=1e12`（平移块 min eig≈0 被截断），B 侧应以 `active` 优先或对 1e12 做白名单。

### 8.5 B：odom 切 10 Hz 与注入门控（test20261118）

- odom 间隔 87.6–94.5 ms，输入队列 max 8 / 溢出 0，位姿失败 0 / 空洞 13，成功着色 1898 帧。
- 单帧：等位姿 avg≈164 ms、前端 28–29 ms、约束 12 ms、更新 1 ms、彩色 8–12 ms。
- 门控 0.2 vs 0.4（同 bag）：生效 521 vs 1089（34.4% vs 72.4%），与 `[年龄]` 直方图一致；
  两档成图逐格对比一致（5 cm 栅格 IoU 0.88，点数 8.289M vs 8.298M，无重影）。
- 结论：0.4 只是把已算出的约束用上；再放宽无收益（尾部含积压离群帧）。
  降时差应从结构延迟下手（等位姿 164 ms / 前端 28 ms），不是继续放门限。
- 跨运行的"过期数"不可比（回放抖动），判据固定用同一趟数据的 `[年龄]` 分布。
- 当前配置：`super_livo_visual.inject_max_dt: 0.4`。

### 8.6 test4 全 bag 双臂（2026-11-19，1x 实时，FastDDS）

bag `test4_move_ros2`（sqlite3，图像偏移 −204701439.4937），`run_one.sh test4 {a,b}`：

| 项 | A | B |
| --- | --- | --- |
| 彩色 PCD | 57 MB / 3,686,694 点 | 56 MB / 3,643,719 点 |
| odom | 829 行（bag 90.5 s，≈9.2 Hz） | 502 行（≈5.5 Hz） |
| 单帧视觉合计（采样行） | avg 10.5 ms / max 28.4 | avg 44.0 ms / max 69.9 |
| 端到端 `[Img->VisOdom]`（末段窗口） | **avg 11.4 ms** / max 82.2 | **avg 83.8 ms** / max 436.8 |
| 端到端 `[Img->VisCloud]`（末段窗口） | avg 59.5 ms / max 402.3 | avg 102.6 ms / max 517.4 |
| 总墙钟（含 5 s 预热 + 落盘） | 115 s | 113 s |

两臂标定覆盖均生效：内参取 `config/camera_intrinsic.yaml`（1280×1024, fx=708.63），
外参取 FAST-Calib `output/calib_result.txt`。端到端计时由 `super_livo_visual.timing` 门控，
`run_one.sh` 已统一打开。

---

## 9. 运行与换机

```bash
# 编译（仓库根）
source /opt/ros/humble/setup.bash
(cd third_party/Super-LIO && colcon build --packages-select super_lio --cmake-args -DCMAKE_BUILD_TYPE=Release)
source third_party/Super-LIO/install/setup.bash
colcon build --packages-select super_livo_common pointcloud_colorizer \
  super_livo_visual_a super_livo_visual_b --cmake-args -DCMAKE_BUILD_TYPE=Release

# A
ros2 run super_livo_visual_a visual_a_app --ros-args \
  --params-file common/super_livo_common/config/lio_move.yaml \
  --params-file solution_a_lightweight/super_livo_visual_a/config/visual_a_move.yaml

# B
ros2 run super_livo_visual_b visual_b_app --ros-args \
  --params-file common/super_livo_common/config/lio_move.yaml \
  --params-file solution_b_tight_coupled/super_livo_visual_b/config/visual_b_move.yaml \
  -p super_livo_visual.inject:=true
```

- 推荐的统一跑批：`experiments/scripts/paper_matrix/run_one.sh <seq> <arm>`
  （四臂 `fl2/a/b/sl`，1x 实时，内置互斥与残留清理）。
- 退出请用 SIGINT/Ctrl-C：两臂都是 `shutdownAndSave()` 同步落盘，`kill -9` 会丢最后的彩色地图。
- 正常判据：A 每秒一行 `视觉帧:`（tracked 80–150、n>0、合计 5–10 ms）；
  B 每 2 s 一组 `[PROBE-*]`、`pend_vis=0, pend_cloud=0`，退出打印 `push/take/生效/过期`。

---

## 10. 已知限制与待办

| 项 | 说明 |
| --- | --- |
| A 的深度关联 | 四个 livo bag 上 `n=0`，B 同包 `obs=133–179`；需单独排查 A 的外参取值/时延/坐标系约定 |
| A 视觉回灌 | 未编码（8 个 `update.inject_*` 参数仅预留） |
| B 直吃观测信息 | `UnifiedEstimator::lidarUpdate` 仍用 `lio/odom` 合成先验，未消费快照 `info`（数值口径变更，需 S0 逐位对比） |
| 退化强度分支 | 缺真实退化样本（长走廊/隧道/空旷地），`active=0` 未触发过 |
| 单测 | `VisualPipeline` 已 ROS 无关，但尚未补脱离 ROS 的单元测试 |
| `/lio/degeneracy` | 发布端为过渡保留，A/B 均已直读内存快照 |

---

## 11. 踩坑记录

- **落盘阻塞回调**：47 MB PCD 序列化 855–980 ms，整图快照移交仍会拖到 1267 ms；
  最终按"新增段"增量移交 + 独立落盘线程。
- **孤儿进程**：`ros2 run` 只是 wrapper，`kill` wrapper 带不走真身；残留进程占 FastDDS 共享内存端口，
  下一臂会 `open_and_lock_file failed` → 0 odom。跑批必须按命令行正则 pkill 并清 `/dev/shm/fastrtps_*`。
- **mcap**：本机无 rosbag2 mcap 插件（无 root 装不了），需 `rosbags` 转 sqlite3 并改写
  `metadata.yaml` 为 v4（`offered_qos_profiles: ""`）。
- **WSL 路由**：`127.0.0.1 via … dev loopback0` 会让 DDS 只剩多播发现，跑批前删掉该路由。
- **RMW**：本机 WSL 用 CycloneDDS 回放 move 会被压到 4.5 Hz，统一 FastDDS。
- **图像时钟**：`move_ros2` / `test4_move` 的图像 header 是主机时钟，必须用
  `super_livo_visual.img_time_offset` 覆盖（两包分别 −204262818.9974 / −204701439.4937）。
