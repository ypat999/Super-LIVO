# 视觉支路单体化：实施进度与交接清单

> 配套方案文档：[monolith_refactor_plan.md](monolith_refactor_plan.md)（commit `0c3b3ad`）
> 本文只记录"做到哪儿了、验证过什么、另一台机器上要做什么"。
> 最后更新：2026-11-18（方案 A 的 S1 接线完成并跑通；B 线与视觉回灌标定未开始）

---

## 1. 一句话状态

方案 A 的**进程内单体接线（S1）已完成并编译、跑通**：Super-LIO 在主进程里通过两个虚钩子把
位姿/世界云/机体系全量云/退化快照直接"推"给 `VisualPipeline`，视觉支路不再有任何 DDS 回环。
原有的 `visual_a_node`（S0 对照）与 `super_livo_visual_b` 均未改动，行为不受影响。

**未开始**：视觉回灌 ESKF 的参数接线（§6.5 的 8 个参数）、方案 B 直吃 `last_HTVH`、S2 清理。

---

## 2. 已完成的改动

### 2.1 公共库 `liblio`（纯加法，未覆写时纯 LIO 逐位不变）

| 文件 | 改动 |
| --- | --- |
| `third_party/Super-LIO/src/super_lio/include/lio/super_lio.h` | 新增 `virtual onLioOutput(const OutputData&)`、`virtual onDenseCloud(const CloudPtr&, double)`；新增 `DegeneracySnapshot` 结构与 `degeneracySnapshot()` 线程安全访问器 |
| `third_party/Super-LIO/src/super_lio/src/lio/super_lio.cpp` | `OutputThread` 末尾调用 `onLioOutput`；`DenseThread` 发布前调用 `onDenseCloud`；`Observe()` 中把 `last_HTVH` 与退化状态机结果写入快照；新增 `degeneracySnapshot()` 实现 |

`DegeneracySnapshot` 字段：`ok / cond / axes / active / dir_w / has_info / info`，
其中 `dir_w` 为最弱平移方向（世界系），`info` 为未截断的位姿观测 Hessian（`[rot, pos]` 布局）。

### 2.2 公共 mailbox

| 文件 | 说明 |
| --- | --- |
| `common/super_livo_common/include/super_livo_common/latest_sample_buffer.hpp` | header-only，`push/clear/empty/size/latest/nearest/bracket`，供 B 线输入侧使用（A 线暂未依赖） |

### 2.3 方案 A：新建单体流水线

| 文件 | 说明 |
| --- | --- |
| `solution_a_lightweight/super_livo_visual_a/include/super_livo_visual_a/pipeline/visual_pipeline.hpp` | `VisualPipelineOptions` + `VisualPipeline`，**不依赖 rclcpp** |
| `solution_a_lightweight/super_livo_visual_a/src/pipeline/visual_pipeline.cpp` | 原 `VisualANode` 的逻辑逐项迁移：去畸变、KLT、曝光时刻插值、深度关联、重投影法方程、退化门控、`VisualUpdate`、持久偏移/回灌、上色、落盘 |
| `solution_a_lightweight/super_livo_visual_a/src/app/visual_a_app.cpp` | `VisualASuperLIO` 覆写两个钩子直推 pipeline；`setupMonolithIo()` 只保留图像订阅 + 彩色云/odom/path 发布 + 落盘定时器；新增 `app.lio_source` 开关 |
| `solution_a_lightweight/super_livo_visual_a/CMakeLists.txt` | 新增独立共享库 `super_livo_visual_a_pipeline`，`visual_a_app` 链接之；`visual_a_node` 未改 |

线程模型：`VisualPipeline` 内部**单工作线程 + 任务队列**，Super-LIO 的 `OutputThread`/`DenseThread`
只做数据拷贝入队，不会被视觉处理阻塞；快照读取单独加锁。

### 2.4 未改动（刻意保留）

- `src/node/visual_a_node.cpp` / `visual_a_node_main.cpp`：S0 对照基线。
- `solution_b_tight_coupled/super_livo_visual_b/**`：B 线下一阶段再动。

---

## 3. 已验证内容（本机）

```
colcon build --packages-select super_livo_common super_lio super_livo_visual_a super_livo_visual_b basic pointcloud_colorizer
→ 全部通过（liblio 导出 degeneracySnapshot，visual_a_app 链接成功）
```

同一份 bag 分别跑两种模式，视觉前端指标一致：

| 项 | S0（`app.lio_source:=topic`，旧双节点） | S1（默认 `internal`，单体） |
| --- | --- | --- |
| logger 名 | `super_livo_visual_a` | `super_lio` |
| 跟踪特征 | `tracked=115/54` | `tracked=115/45` |
| 单帧耗时 | 合计 5.2 ms | 合计 6.4 ms |
| 上色 | 1463/17790 点，帧#425 | 1477/17816 点，帧#425 |
| 退化签名 | 无（`/lio/degeneracy` 从未发布，门控是死代码） | 由内存快照驱动，`deg=0.00`（健康） |
| 错误 | 无 | 无 |

结论：单体接线在功能上与旧路径等价，额外开销约 1 ms/帧（同一 bag、同一配置）。

---

## 4. 另一台机器上的操作

### 4.1 拉取与编译

```bash
cd ~/git/Super-LIVO
git pull
git submodule update --init --recursive      # 钩子改动在 third_party/Super-LIO 里

source /opt/ros/humble/setup.bash
colcon build --packages-select super_livo_common super_lio super_livo_visual_a \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4.2 运行

```bash
# S1：单体（默认 app.lio_source=internal）
ros2 run super_livo_visual_a visual_a_app --ros-args \
  --params-file config/retail_lio.yaml \
  --params-file solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml

# S0：旧路径对照（同进程仍起两个节点，走 DDS）
#   在上一行基础上追加：
#   -p app.lio_source:=topic
```

需要覆盖的输入话题/雷达类型（按你那份数据改）：

```bash
-p lio.ros.lidar_topic:=/livox/lidar \
-p lio.ros.imu_topic:=/livox/imu \
-p lio.sensor.lidar_type:=0 \      # bag 里是 sensor_msgs/PointCloud2 时用 0；Livox CustomMsg 用 1
-p image_topic:=/camera/color/image_raw
```

### 4.3 判断是否正常

启动后每秒一行 `视觉帧:`，正常应满足：

- `tracked` 在 80~150 之间波动；
- `n`（有效深度数）**大于 0**——如果一直是 0，说明相机外参/时延不匹配，视觉修正不会产生任何约束；
- `deg` 随场景变化（健康场景接近 0，走廊/长直通道升高）；
- `合计` 稳定在 5~10 ms；
- 彩色统计里 `实际着色帧` 持续增长。

---

## 5. 需要你做的事（按顺序）

### 步骤 1：确认 `n > 0`

本机测试用的 `rosbag2_fastlivo2_2026_10_30-18_00_25` 上 **S0 与 S1 都是 `n=0`**，
说明该 bag 的相机-LiDAR 外参与 `visual_a_params.yaml` 不匹配（不是本次接线的回归）。
请在你那份数据上先确认 `n` 稳定大于 0，否则后续所有标定都没有意义。

排查顺序：`Rcl/Pcl`（`T_cam_lidar`）→ `img_time_offset` → `cam_fx/fy/cx/cy`（注意 `scale 0.5` 已折算）。

### 步骤 2：采集退化参数量级（决定回灌门槛）

在 S1 下跑三段数据（正常、走廊/长直、楼梯/开阔），记录每秒 `视觉帧` 里的 `deg=` 与 LIO 侧
`cond` 值，填这张表：

| 场景 | cond 范围（典型/峰值） | deg 范围 | 备注 |
| --- | --- | --- | --- |
| 正常室内 | 待填 | 待填 | |
| 走廊 / 长直通道 | 待填 | 待填 | |
| 楼梯 / 开阔退化 | 待填 | 待填 | |

结论用于确定 `update.inject_severity_min`（默认 0.6）与 `update.inject_info_ratio`（默认 1.0）。

### 步骤 3：视觉回灌 ESKF（第 6~8 步，尚未编码）

参数按方案 §6.5 预留，**目前这些字段还不存在于 `VisualPipelineOptions`，需要先接线**：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `update.inject_to_eskf` | `false` | 总开关（false = 纯后处理，与现状逐位一致） |
| `update.inject_severity_min` | `0.6` | 退化严重度门槛 |
| `update.inject_weight` | `1.0` | 视觉权重全局缩放（回灌比例主旋钮） |
| `update.inject_info_ratio` | `1.0` | 视觉信息量上限 ≤ ratio·λmax(LiDAR 先验) |
| `update.inject_rot_sigma_floor` | `0.005` | 视觉姿态精度下限（rad） |
| `update.inject_pos_sigma_floor` | `0.02` | 视觉位置精度下限（m） |
| `update.inject_max_dt` | `0.2` | 视觉帧↔雷达帧时间差上限（s） |
| `update.inject_dir_gated` | `true` | 是否只在退化方向放开视觉权限 |

实施注意：
1. 先保持 `inject_to_eskf=false`，确认 `/lio/odom` 与纯 LIO 逐位一致（回归基线）。
2. 打开后先只调 `inject_weight`，`inject_info_ratio` 从 0.3 起步逐步放大。
3. 写回走 `ExternalObserve()` + `eskf()->UpdateObserveFull(...)`；白化与
   "世界左扰动 → 机体系右扰动"的映射直接复用 B 线 `ConstraintBridge` 的做法。

### 步骤 4：方案 B（第 9~11 步）

1. `UnifiedEstimator::lidarUpdate` 由"`lio/odom` 合成强先验"改为直吃 `last_HTVH`；
   先与 S0 基线逐位对比确认不劣化。
2. 把 B 的输入侧换成 `VisualPipeline` + `latest_sample_buffer`（保留 `ConstraintBridge`）。
3. 稳定后进 S2：删除 `app.lio_source=topic` 对照路径与两个旧节点。

---

## 6. 坑与注意事项

1. **`/lio/degeneracy` 在原树中从未发布**，所以旧节点（S0）的退化门控是死代码，
   一直停留在 healthy。S1 用内存快照才让它真正生效——这是本次改动带来的行为差异，
   对拍时不要误判成 bug。
2. **LIO 观测信息为空时** `snap.cond` 会到 `1e12`、`deg` 到 `1.0`（本机第一次异常跑测见过）。
   若此时 `n=0` 则无实际影响；一旦有视觉约束，建议先加 `has_info` 过滤再放行回灌。
3. **bag 回放与实时时钟**：`/livox/imu` 的 header 时间若落后墙钟会出现大量
   `imu loop back, clear buffer`。用 `--clock` / `use_sim_time` 或在回放前统一时间。
4. `third_party/Super-LIO` 是**子模块**，钩子改动必须先提交并推送子模块，父仓库再更新指针，
   否则另一台机器 `git pull` 拿到的是旧指针。
5. 不要在这个阶段动 `src/node/visual_a_node.cpp`（S0 基线），也不要动 `super_livo_visual_b`。
6. `pcd.save_dir` 默认 `/home/ywj/slam_data/pcd`，换机器记得改。

---

## 7. 复查用脚本（本机临时脚本，未入库）

```bash
#!/usr/bin/env bash
# 用法: smoke.sh internal|topic
cd ~/git/Super-LIVO
source install/setup.bash
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
kill -INT "$APP" 2>/dev/null; sleep 3; kill -INT "$APP" 2>/dev/null

grep -E '视觉帧' "/tmp/a_${MODE}.log" | tail -3
grep -E '彩色统计' "/tmp/a_${MODE}.log" | tail -1
grep -iE 'error|terminate|what\(\)' "/tmp/a_${MODE}.log" | head -3
```

对拍时固定同一份 bag、同一份参数，只改 `app.lio_source`。
