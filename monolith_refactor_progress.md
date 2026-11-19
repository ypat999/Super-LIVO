# 视觉支路单体化：实施进度与交接清单

> 配套方案文档：[monolith_refactor_plan.md](monolith_refactor_plan.md)
> 本文只记录"做到哪儿了、验证过什么、换机器要做什么"。
> 最后更新：2026-11-19（B 完成 S2：抽出 `VisualPipeline`、删除 `visual_b_node`，`visual_b_app` 单进程单节点；
> 退出改为同步落盘 `shutdownAndSave()`，跑批脚本与配置顶层键同步；A/B 启动时间对齐在离线回放下按设计回退配置值）

---

## 1. 一句话状态

- **方案 A**：进程内单体接线（S1）已完成、编译并跑通。Super-LIO 通过
  `onLioOutput` / `onDenseCloud` 两个虚钩子把位姿、世界云、机体系全量云、退化快照
  直推 `VisualPipeline`，视觉支路不再有任何 DDS 回环。旧 `visual_a_node`（S0 对照）保留。
- **方案 B**：**S2 已完成（2026-11-19）**——`VisualPipeline`（ROS 无关）已抽出，
  `visual_b_app` 单进程单节点：`ROSWrapper` + `VisualPipeline`，LIO→视觉改由
  `onLioOutput` / `onDenseCloud` / `degeneracySnapshot` 直推，旧 `visual_b_node` 与其
  DDS 订阅已删除。节点内部保持回调 / 计算 / 落盘三线程；odom 源为 10 Hz `/lio/odom`
  （用 `odom_robo` 外参换回 IMU 系），视觉注入时间门控 `app.visual_inject_max_dt` 定为 `0.4`。
  配套接线：三份数据集配置顶层键改 `/**`、`run_one.sh` b 臂改起单进程、退出改为
  **同步落盘**（`shutdownAndSave()`），详见 [方案 B README §9.11](solution_b_tight_coupled/README.md)。
- **未开始**：A 的视觉回灌 ESKF（§6.5 的 8 个参数）、B 直吃 `last_HTVH`、单测。

---

## 2. 已完成的改动

### 2.1 公共库 `liblio`（纯加法，未覆写时纯 LIO 逐位不变）

| 文件 | 改动 |
| --- | --- |
| `third_party/Super-LIO/src/super_lio/include/lio/super_lio.h` | 新增 `virtual onLioOutput(const OutputData&)`、`virtual onDenseCloud(const CloudPtr&, double)`；新增 `DegeneracySnapshot` 结构与 `degeneracySnapshot()` 线程安全访问器 |
| `third_party/Super-LIO/src/super_lio/src/lio/super_lio.cpp` | `OutputThread` 末尾调用 `onLioOutput`；`DenseThread` 发布前调用 `onDenseCloud`；`Observe()` 中把 `last_HTVH` 与退化状态机结果写入快照；新增 `degeneracySnapshot()` 实现 |

`DegeneracySnapshot` 字段：`ok / cond / axes / active / dir_w / has_info / info`，
其中 `dir_w` 为最弱平移方向（世界系），`info` 为未截断的位姿观测 Hessian（`[rot, pos]` 布局）。

> 原计划里的 `onImuOdom` 钩子**没有实现**——B 现在直订 `/livox/imu` 做传播、订 10 Hz
> `/lio/odom` 做位姿先验，不再需要 200 Hz 的 `/lio/imu/odom`。
> 只有 B 输入侧真正 mailbox 化、且决定把 200 Hz NavState 也直供时才需要补。
> 子模块另有远端提交 `5b4ccf0`（新增 `lio.kf.dynamic_start` 参数），与本工作无关。

### 2.2 公共 mailbox

| 文件 | 说明 |
| --- | --- |
| `common/super_livo_common/include/super_livo_common/latest_sample_buffer.hpp` | header-only，`push/clear/empty/size/latest/nearest/bracket` |

A 最终没有用它——`VisualPipeline` 内部是"任务队列 + 单工作线程"。聚合型
`lio_output_mailbox.hpp` 未建，等 B 输入侧改造时再定。

### 2.3 方案 A：单体流水线（S1）

| 文件 | 说明 |
| --- | --- |
| `solution_a_lightweight/super_livo_visual_a/include/super_livo_visual_a/pipeline/visual_pipeline.hpp` | `VisualPipelineOptions` + `VisualPipeline`，**不依赖 rclcpp** |
| `.../src/pipeline/visual_pipeline.cpp` | 原 `VisualANode` 的逻辑逐项迁移：去畸变、KLT、曝光时刻插值、深度关联、重投影法方程、退化门控、`VisualUpdate`、持久偏移/回灌、上色、落盘 |
| `.../src/app/visual_a_app.cpp` | `VisualASuperLIO` 覆写两个钩子直推 pipeline；`setupMonolithIo()` 只保留图像订阅 + 彩色云/odom/path 发布 + 落盘定时器；`app.lio_source` 开关 |

- 线程模型：`VisualPipeline` 内**单工作线程 + 任务队列**，`OutputThread`/`DenseThread`
  只做拷贝入队，不会被视觉阻塞；退化快照读取单独加锁。
- 远端新增输出（commit `d66beb6`）：TUM 轨迹 `evo.*`、PNG 图像 `image_save.*`，默认关。
- `app.lio_source`：`internal`（默认，单体）/ `topic`（S0 双节点对照）。

### 2.4 方案 B：进程内注入 + 三线程（commit `b0f5adc` + 本地未提交修正，过渡形态）

> 目标形态是**单进程单节点**：`VisualBNode` 最终被 ROS 无关的 `VisualPipeline`
> 取代并删除，`visual_b_app` 只保留 `ROSWrapper`。本节的三线程改造只是过渡，
> 它解决了回调阻塞，但没有消除 DDS 订阅本身。
>
> **2026-11-19（S2 已完成）**：上表 `VisualBNode` 侧的全部改造已整体搬进
> `super_livo_visual_b/{include,src}/pipeline/visual_pipeline.{hpp,cpp}`，`visual_b_node.*`
> 已删除；本节的文件名字面保留为历史记录。

| 改动 | 说明 |
| --- | --- |
| `visual_b_app.cpp` | 一个进程内建 `ROSWrapper`(LIO) + `VisualBNode`(视觉)，同 executor；`app.embed_visual` / `app.visual_inject` 控制；写回仍走 `ConstraintBridge` 直接调用 |
| `visual_b_node.hpp/.cpp` | 回调只 `pushInputEvent` 入 `input_queue_`（上限 2048，超限丢最旧）；`workerLoop` 单线程 FIFO 执行 `onImage/onOdom/onCloud/onBodyCloud/onImu/onSaveSnapshot`；`saveLoop` 独立写彩色 PCD |
| 彩色落盘 | 工作线程按"新增段"增量移交（`ColorSegment` + `save_handed_points_`），落盘线程并进常驻缓冲 |
| odom 输入 | `odom_topic` 默认 `lio/odom`（10 Hz、base_link 系、带协方差）；`setOdomBodyExtrinsic()` 由 app 从 `LI2Sup::g_odom_robo` 注入外参，`onOdom` 里 `T_wl = T_wl * T_imu_robo⁻¹` 换回 IMU 系。IMU 传播直订 `/livox/imu`，不再订阅 `/lio/imu/odom` |
| 位姿查询 | `poseFromSamples()` 直接在 `pose_cache_` 上二分 + 插值，不再逐帧重建 `LidarPoseInterpolator`；拒绝 `stamp > cache.back().stamp` |
| 彩色排空 | `drainPendingColorization()` 加 `kColorDrainMinIntervalSec=0.1` 节流，避免积压时无解重试吃满工作线程 |
| 视觉注入门控 | `app.visual_inject_max_dt`（默认 0.2，本机配置 0.4）；`ExternalObserve()` 中 `|obs.stamp - ESKF时刻|` 超限即计"过期"丢弃 |

起因：原先 6 个订阅 + 落盘定时器同在一个互斥回调组，一次 47 MB PCD 落盘实测阻塞
855~980 ms，压爆 `pending_visual_`（上限 8）白丢 293 帧；第一版"整图快照移交"仍会
拖停（1267 ms），最终改成增量移交。

**已知不一致**：`degen_sub_` 仍直接 `std::bind(&VisualBNode::onDegeneracy)` 绑在
executor 上，是唯一没进 `input_queue_` 的回调。`degen_sig_` 有独立 mutex 不会崩，
但签名与帧之间没有顺序保证。

### 2.5 本地未提交改动（换机器前注意）

> **2026-11-19 更新**：下列改动均已随 `bb712d6`（单体化 S2）与 `bf916ef`（退出同步落盘 +
> 跑批接线）提交并推送，`git pull` 可取；本节保留为当时的改动清单。

以下改动在**本机工作区尚未提交**，`git pull` 不会带过去：

- `visual_b_node.cpp/.hpp`：`odom_topic` 默认改为 10 Hz 的 `lio/odom`，并用
  `odom_robo⁻¹` 换回 IMU 系（`setOdomBodyExtrinsic()`）；`poseFromSamples()` 重写为
  缓存上二分 + 插值、拒绝超出最新采样；彩色排空 0.1 s 节流；新增
  `noteSkippedInterval()` 处理 IMU 中断/空洞。
- `visual_b_app.cpp`：`setOdomBodyExtrinsic()` 注入 `g_odom_robo`；`[年龄]` |age|
  直方图探针（<0.1 / 0.1–0.2 / 0.2–0.3 / 0.3–0.4 / ≥0.4 s），用于同一趟数据内判断
  门限的边际收益（跨运行的"过期数"因 bag 抖动不可比）。
- `visual_b_move.yaml`：`app.visual_inject_max_dt: 0.4`。
- `unified_estimator.cpp/.hpp`：IMU 空洞时 `freeze_bias` + 更新步长分块限幅。
- `visual_a_move.yaml`：`gate.max_reprojection_rmse: 6.0`、`pcd.save_interval_sec: 0.0`。
- `config/retail_lio.yaml`、`visual_b_params.yaml`、`experiments/`、`tools/` 下的若干调参。

> 如果这些改动需要同步到另一台机器，先提交再 push；否则另一台机器只拿到 `b0f5adc` 的版本。

---

## 3. 已验证内容

### 3.1 编译

```bash
colcon build --packages-select super_livo_common super_lio super_livo_visual_a \
  super_livo_visual_b basic pointcloud_colorizer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

全部通过（liblio 导出 `degeneracySnapshot`，`visual_a_app` 链接 `super_livo_visual_a_pipeline`）。

### 3.2 方案 A：S0 vs S1（同一 bag、同一配置）

| 项 | S0（`app.lio_source:=topic`） | S1（`internal`） |
| --- | --- | --- |
| logger 名 | `super_livo_visual_a` | `super_lio` |
| 跟踪特征 | `tracked=115/54` | `tracked=115/45` |
| 单帧耗时 | 合计 5.2 ms | 合计 6.4 ms |
| 上色 | 1463/17790 点，帧#425 | 1477/17816 点，帧#425 |
| 退化签名 | 无（当时 `/lio/degeneracy` 无发布者） | 内存快照驱动，`deg=0.00`（健康） |
| 错误 | 无 | 无 |

结论：功能等价，额外开销约 1 ms/帧。
注意：该 bag 上 S0/S1 **都是 `n=0`**（有效深度数为 0），是相机-LiDAR 外参/时延与
`visual_a_params.yaml` 不匹配，不是本次接线的回归。

### 3.3 方案 B：Retail_Street 全量（135.5 s / 1355 帧 LiDAR / 1355 图像 / 10 Hz）

数据：`/home/ywj/slam_data/Retail_Street`（`Retail_Street.db3` + `avia.yaml`），
话题 `/livox/lidar`、`/livox/imu`、`/left_camera/image`，无 ground truth。

视觉注入 OFF / ON 两轮，同 bag、同参数、`ROS_DOMAIN_ID=86` 隔离：

| 项 | OFF | ON |
| --- | --- | --- |
| ERROR / 队列溢出 | 0 / 0 | 0 / 0 |
| odom 行数 / 时长 | 1305 / 133.5 s | 1306 / 133.6 s |
| 轨迹长度 / 回环残差 | 66.82 m / 0.02 m | 66.84 m / 0.03 m |
| 最大速度 | 1.14 m/s | 1.49 m/s |
| 视觉帧处理 | 1352（严格 10 Hz） | 1353 |
| 单帧 avg / max | 21.1 / 34.9 ms | 20.9 / 30.3 ms |
| 注入生效 | — | `push=1347 take=1308 生效=1308 被拒=0 过期=0` |

- 单帧预算：前端≈8.9 ms、约束≈8.7~8.9 ms、更新≈0.6 ms、彩色≈2.2~2.9 ms；
  100 ms 帧间隔约 5 倍余量，`pend_vis=0, pend_cloud=0`。
- 前端一致：`obs≈148, tracks≈131, used≈230, quality≈0.21`，地图 60000 点；
  位姿空洞丢弃各 7 帧（OFF 另有"待定队列超限"2 次，ON 为 0）。
- ON vs OFF 时间轴对齐（1304 点）：中位偏差 2.62 cm、均值 2.31 cm、p95 4.32 cm、
  最大 5.55 cm、终点差 0.28 cm ⇒ 视觉只是微调，未把本已很好的解带偏。
- 单帧注入 `dxp` 多数 0~0.1 mm（最大 1.0 mm），`λ≈5.9e6 → scale≈0.0068`。

> 该轮 B 仍使用 200 Hz `/lio/imu/odom` 做位姿源；切到 10 Hz `/lio/odom` 后的口径
> 与注入门控结论见 §3.5。

**这轮没验证到的**：当时 `/lio/degeneracy` 全仓库无发布者（`ROSWrapper.cpp` 只有
`lio.observe.degeneracy_*` 参数），实测 `退化 sev` 恒 0、`dir=(0,0,0)`；
"强退化→加强回灌"整条分支是死代码。Retail_Street 全段结构化良好、本来也不该触发
退化，所以结论仅限于"非退化场景不损伤"。发布端已于后续补齐，见 §3.4。

### 3.4 退化发布端 + livo_bag 四包（按已知标定）

发布端：`ROSWrapper::pub_degeneracy()`，随 10 Hz `pub_odom()` 把
`super_lio_->degeneracySnapshot()` 发到 `lio/degeneracy`，8 元素
`[cond6, axes, active, cond_t, evt_min, ux, uy, uz]`，best_effort（兼容 B 的
`SensorDataQoS`）；`snap.ok && snap.has_info` 不成立时发全 0（B 侧 `valid()` 为 false）。

形态：单进程 `visual_b_app` + `lio_move.yaml` + `visual_b_move.yaml`。图像时钟按已知标定
换算到雷达时钟：`all_ros2` −204262819.04、`move_ros2` −204262819.03、
`move_livoxclk` 用 yaml 的 0、`test4_move` −204701439.32（`freqcmd.txt` 已标定值）。

| bag | 时长 | degen 条数 | cond6 p50（min–max） | active | 视觉帧处理 | push / 生效 | ERROR |
| --- | --- | --- | --- | --- | --- | --- | --- |
| all_ros2 | 41 s | 392 | 4.78（4.43–5.43） | 0 | 406 | 386 / 383 | 0 |
| move_ros2 | 77 s | 754 | 4.04（2.81–1e12） | 0 | 737 | 690 / 673 | 0 |
| move_livoxclk | 77 s | 765 | 4.05（2.82–5.24） | 0 | 741 | 688 / 681 | 0 |
| test4_move | 90 s | 868 | 1.88（1.44–2.92） | 0 | 778 | 769 / 331（过期 308） | 0 |

- 发布端按 10 Hz 稳定出数，`cond6` 随场景变化；但四包 `active=0`、`退化 sev=0`，
  只验证了"不误触发"，强度分支仍缺真实退化样本。
- `move_ros2` 有一帧 `cond6=1e12`（平移块 min eig≈0 被截断）：B 按 cond 对数映射的
  severity 会到 1.0，但 LIO 状态机 `active=0`、下一帧即恢复，10 s 采样日志没抓到。
  若要更稳，B 侧应以 `active` 优先或对 1e12 做白名单。
- A（`internal`）同四包仍 `n=0`（tracked 86–150、合计 4.8–16 ms），而 B 同包
  `obs=133–179`。结合 §3.2，A 的 `n=0` 不是某个 bag 的外参特例，需单独排查
  A 的深度关联（外参取值/时延/坐标系约定）。
- `test4_move` 是 mcap，本机没有 rosbag2 的 mcap storage 插件：先用
  `rosbags-convert --src ... --dst ... --dst-storage sqlite3 --dst-typestore copy` 转
  sqlite3，再把 `metadata.yaml` 改写成 rosbag2 v4 结构（`offered_qos_profiles: ""`）
  才能 `ros2 bag play`。产物 3.4 GB 在 `/home/ywj/slam_data/output/bag_convert/test4_move`。
- 该轮 B 仍用 200 Hz `/lio/imu/odom`；odom 切到 10 Hz `/lio/odom` 后，等位姿/取用时差
  变长（见 §3.5），`push/生效` 的绝对值不可直接与本节比较。

### 3.5 方案 B：odom 切 10 Hz 与注入门控（test20261118）

数据：`/home/ywj/slam_data/livo_bag/test20261118`（move 场景），单进程 `visual_b_app`，
`lio_move.yaml` + `visual_b_move.yaml`，产物在 `output/livo_bag_test20261118/b/`。
四轮同 bag 对照：`_odom10`（0.2 门限）、`_dt04`（0.4）、`_dt02b`（0.2 复跑）、
`_hist02`（带 `[年龄]` 直方图）。

| 项 | 结果 |
| --- | --- |
| odom 相邻间隔 | 87.6–94.5 ms（10 Hz，抖动 < 7 ms） |
| 输入队列深度 / 溢出 | max 8 / 0 |
| `[PROBE-D]` 位姿失败 / 空洞 | 0 / 13 |
| 成功着色帧 | 1898 |
| 单帧预算 | 等位姿 avg≈164 ms、前端≈28–29 ms、约束≈12 ms、更新≈1 ms、彩色≈8–12 ms，合计≈50–58 ms |

注入门控 0.2 vs 0.4（同一份 bag）：

| 项 | `visual_inject_max_dt=0.2` | `=0.4` |
| --- | --- | --- |
| push / take | 1806 / 1515 | 1807 / 1504 |
| 生效 / 过期 | 521 / 994 | 1089 / 415 |
| 生效 ÷ take | 34.4% | 72.4% |
| 取用时差 avg | 323.6 ms | 338.3 ms |

- 同一趟数据的 `[年龄]` 直方图：0.1–0.2 s 34.4%、0.2–0.3 s 9.7%、0.3–0.4 s 27.3%、
  ≥0.4 s 28.6%（<0.1 s 约 0）。0.2 门限只能接 34%，0.4 能接 71%，与"生效 ÷ take"
  的 34.4% / 72.4% 互相印证。
- 两档成图逐格对比一致：5 cm 栅格 IoU 0.88、点数 8.289M vs 8.298M、
  范围同为 x[-5.57,10.94] y[-2.30,2.55]，无重影。
- 结论：10 Hz odom 下 0.32–0.35 s 的取用时差是结构性的（要等激光位姿覆盖 `t_img`），
  门限放宽到 0.4 只是把已经算出来的约束用上，不改变成图；再往上调没有收益
  （≥0.4 s 那 29% 含 1.3 s 级积压离群帧）。降时差应从结构延迟下手
  （前端 28–29 ms、等位姿 164 ms），不是继续放门限。
- 跨运行的"过期数"不可比（同门限 0.2 在不同轮次出现 994 / 1009，bag 回放抖动会让
  取用时差整体后移），判据固定用同一趟数据的 `[年龄]` 分布。

---

## 4. 换机器操作

### 4.1 拉取与编译

```bash
cd ~/git/Super-LIVO
git pull
git submodule update --init --recursive      # 钩子改动在 third_party/Super-LIO 里

source /opt/ros/humble/setup.bash
colcon build --packages-select super_livo_common super_lio super_livo_visual_a \
  super_livo_visual_b --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4.2 跑方案 A

```bash
# S1：单体（默认 app.lio_source=internal）
ros2 run super_livo_visual_a visual_a_app --ros-args \
  --params-file config/retail_lio.yaml \
  --params-file solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml

# S0：旧双节点对照，追加 -p app.lio_source:=topic
```

### 4.3 跑方案 B

> 当前 B 是**过渡形态**（进程内 2 节点 + LIO 输出走 DDS）。S1 收敛为单进程单节点后
> 启动命令不变，但进程内只剩 `ROSWrapper` 一个节点。

```bash
ros2 run super_livo_visual_b visual_b_app --ros-args \
  --params-file config/retail_lio.yaml \
  --params-file solution_b_tight_coupled/super_livo_visual_b/config/visual_b_params.yaml \
  -p app.visual_inject:=true
```

同时另开终端回放：

```bash
ros2 bag play /home/ywj/slam_data/Retail_Street --rate 1.0
```

注意：

- `config/retail_lio.yaml` 里 `app.visual_inject: false`，B 必须在命令行覆盖为 `true`。
- `visual_b_params.yaml` 用 `/**` 通配，换机器要改 `pcd.save_dir`。
- Retail_Street 的 lidar 是 `livox_ros_driver2/CustomMsg`，`lio.sensor.lidar_type: 1` 已对。
- `experiments/scripts/retail_ab/run_ab_retail.sh` 默认 `BAG=.../Retail_Street_d2`，
  本机该路径不存在，要么改脚本、要么直接 `ros2 bag play .../Retail_Street`。

### 4.4 判断是否正常

- A：每秒一行 `视觉帧:`，`tracked` 80~150、`n>0`、`deg` 随场景变化、`合计` 5~10 ms。
- B：`probeReport()` 每 2 s 一组 `[PROBE-IO]/[PROBE-T]/[PROBE-S]/[PROBE-U]/[PROBE-X]/[PROBE-Q]`；
  `处理 Δ=x/y` 每 2 s 应约 20 帧（10 Hz）；`pend_vis=0, pend_cloud=0`；
  1 Hz 日志另有一行 `[年龄]`，给同一趟数据的 `|age|` 分布；
  退出时打印 `push/take/生效/被拒/过期` 计数。
- B 的门限口径：`visual_inject_max_dt` 取 0.4 时，`[年龄]` 中 `<0.4 s` 的占比应与
  `生效/take` 接近（test20261118 实测 71.4% vs 72.4%）。两者差很多说明回放抖动或
  前端/等位姿耗时变了；不要拿跨运行的"过期数"直接比较（见 §3.5）。

---

## 5. 待办

### 5.1 需要你决策的一步

让 B 的退化门控活起来，两条路：

1. **补发布者（过渡）— 已完成**：`ROSWrapper::pub_degeneracy()` 随 10 Hz `pub_odom()`
   把 `super_lio_->degeneracySnapshot()` 转成 8 元素 `Float64MultiArray` 发到
   `lio/degeneracy`（布局同 `DegeneracySignature::fromArray`）。实测见 §3.4。
2. **直吃快照（终态）**：B 与 A 一样直接读 `degeneracySnapshot()`，同时把 `degen_sub_`
   收进工作线程队列。

1 只是让门控"不再恒假"，最终仍要做 2。

### 5.2 尚未编码

1. **A 的视觉回灌 ESKF**（`monolith_refactor_plan.md` §6.5）：8 个参数目前**不存在于**
   `VisualPipelineOptions`，需要先接线；`VisualASuperLIO::ExternalObserve()` 仍是空实现。
2. ~~**B 收敛为单进程单节点**~~ **已完成（2026-11-19）**：`VisualPipeline` 已抽出
   （`super_livo_visual_b/{include,src}/pipeline/`，承接图像前端、`UnifiedEstimator`、
   着色与落盘）；位姿 / 世界云 / 机体系云 / 退化快照改内部直推；`visual_b_app` 只建
   `ROSWrapper` 一个节点，图像与 IMU 订阅、输出发布与周期落盘都挂上去；
   `VisualBNode` 及其 DDS 订阅已删除（S2）。
3. **B 直吃 `last_HTVH`**：`UnifiedEstimator::lidarUpdate` 由合成强先验改为消费快照 `info`。
4. **S2 清理（剩余）**：删除 `app.lio_source=topic` 对照路径与 A 的旧 `visual_a_node`
   （B 的 `visual_b_node` 已随 S2 删除）。

### 5.3 标定前置数据（A 已具备，B 仍缺）

在 S1 下跑三段数据（正常 / 走廊长直 / 楼梯开阔），记录每秒 `deg=` 与 LIO 侧 `cond`：

| 场景 | cond 范围（典型/峰值） | deg 范围 | 备注 |
| --- | --- | --- | --- |
| 正常室内 | 待填 | 待填（Retail_Street 全段≈0） | |
| 走廊 / 长直通道 | 待填 | 待填 | |
| 楼梯 / 开阔退化 | 待填 | 待填 | |

结论用于 `update.inject_severity_min`（默认 0.6）与 `update.inject_info_ratio`（默认 1.0）。

---

## 6. 坑与注意事项

1. **`/lio/degeneracy` 原树中无发布者**，旧节点（A 的 S0、B 全线）的退化门控曾是死代码。
   **已补发布端**（`ROSWrapper::pub_degeneracy()`，best_effort QoS 以匹配 B 的
   `SensorDataQoS`）：A 的 S1 走内存快照，B 现在走 DDS 也能拿到真实签名（§3.4）。
   注意发布端只在 `snap.ok && snap.has_info` 时填值，否则发全 0 ⇒ B 侧 `valid()` 为 false。
2. **LIO 观测信息为空时** `snap.cond` 会到 `1e12`、`deg` 到 `1.0`。回灌/门控前必须按
   `has_info` 过滤，否则会把"没有观测"误判成"强退化"。
3. **`/lio/odom` 是 base_link 系**（施加了 `odom_robo` 外参）。B 默认订阅它
   （10 Hz，与 A 一致），必须在 `onOdom` 里用 `odom_robo⁻¹` 换回 IMU 系，否则带 30°
   pitch 的数据集会重力写歪、伪加速度泵飞速度。外参由 app 从 `LI2Sup::g_odom_robo`
   注入（`setOdomBodyExtrinsic()`），不需要在两份 yaml 里重复标定；只有刻意绕开外参
   换算时才把 `odom_topic` 填成 `lio/imu/odom`（200 Hz、IMU 系）。
4. **bag 回放与时钟**：`/livox/imu` 的 header 时间若落后墙钟会出现大量
   `imu loop back, clear buffer`。用 `--clock` / `use_sim_time` 或回放前统一时间。
5. **`third_party/Super-LIO` 是子模块**：钩子改动要先提交并推送子模块，父仓库再更新指针，
   否则另一台机器 `git pull` 只拿到旧指针。
6. **`pcd.save_dir` 默认 `/home/ywj/slam_data/pcd`**，换机器记得改。
7. **彩色 PCD 落盘必须走独立线程且增量移交**：周期落盘或整图移交都会拖停回调，
   实测分别丢 293 帧 / 阻塞 1267 ms。
8. **退出时的最终落盘必须同步**（2026-11-19 修）：投给异步 save 线程会随进程退出被杀，
   长序列（test4 全量 96 s）表现为 `pcd/` 只有目录没有文件，15 s 短跑因图小反而"看着正常"。
   统一走 `VisualPipeline::shutdownAndSave()`（`std::call_once` 幂等，shutdown 钩子 /
   `main` 收口 / 析构任一路径调用都会阻塞到写盘完成）。
9. `ros2 run` 的 Python 包装器不转发 SIGINT；脚本里收尾请
   `pkill -f 'lib/super_livo_visual_b/visual_b_app'`，否则会被 SIGKILL。

---

## 7. 复查脚本（本机临时脚本，未入库）

```bash
#!/usr/bin/env bash
# 用法: smoke.sh internal|topic
cd ~/git/Super-LIVO
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
pkill -f 'lib/super_livo_visual_a/visual_a_app' >/dev/null 2>&1
sleep 1

MODE="${1:-internal}"
EXTRA=""
[ "$MODE" = "topic" ] && EXTRA="-p app.lio_source:=topic"

ros2 run super_livo_visual_a visual_a_app --ros-args \
  --params-file config/retail_lio.yaml \
  --params-file solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml \
  -p image_topic:=/camera/color/image_raw \
  -p lio.sensor.lidar_type:=0 \
  -p lio.ros.lidar_topic:=/livox/lidar \
  -p lio.ros.imu_topic:=/livox/imu \
  $EXTRA > "/tmp/a_${MODE}.log" 2>&1 &
APP=$!
sleep 6

timeout -s INT 45 ros2 bag play <你的 bag 路径> --rate 1.0 > "/tmp/bag_${MODE}.log" 2>&1
sleep 2
kill -INT "$APP" 2>/dev/null; sleep 3; pkill -f 'lib/super_livo_visual_a/visual_a_app'

grep -E '视觉帧' "/tmp/a_${MODE}.log" | tail -3
grep -E '彩色统计' "/tmp/a_${MODE}.log" | tail -1
grep -iE 'error|terminate|what\(\)' "/tmp/a_${MODE}.log" | head -3
```

对拍时固定同一份 bag、同一份参数，只改一个开关。
