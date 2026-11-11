# Super-LIVO 实机测速报告（RK3588 · Retail_Street）

- 日期：2026-11-10
- 平台：RK3588S（Cortex-A76×4 [核4-7] + Cortex-A55×4 [核0-3]），Ubuntu 22.04 + ROS2 Humble，Release 编译
- 数据：`~/slam_data/Retail_Street`（ros2 sqlite3 bag，135.47 s；`/livox/lidar` Livox Avia CustomMsg 10 Hz、`/livox/imu` ~200 Hz、`/left_camera/image` 640×512 rgb8）
- 被测系统：**Super-LIO**（third_party/Super-LIO）、**官方 FAST-LIVO2**（third_party/FAST-LIVO2-ROS2）、**方案A**（super_livo_visual_a，轻量着色+视觉约束）、**方案B**（super_livo_visual_b，紧耦合）

## 1. 指标口径

参照 Super-LIO 现有 Timer/延迟统计体系（`include/common/timer.h` + `recordLatency`）：

| 指标 | 定义 |
| --- | --- |
| **odom 发出延迟** | 雷达消息到达节点回调（steady 墙钟打点）→ 该帧 `/lio/odom`（或 fast_livo `/odom`）publish 完成的墙钟差。包含预处理、排队、传播、配准、建图与输出线程调度。 |
| **整帧处理时间** | LIO 帧全流程 CPU 处理耗时：Super-LIO `[FrameProcess]`（stateProcess 全函数）；FAST-LIVO2 `[LIOTotal]`（handleLIO t4−t0，与自带 "Current Total Time" 同口径）与 `[VIOTotal]`（主循环一次 VIO 帧全流程）；方案A/B 视觉节点逐帧日志 `合计`。 |
| **时间波动** | 全体样本 std；A/B 视觉另给 P95/max；Super-LIO 60 s 窗口统计按 (n, avg, std) 精确合并为总体。 |

## 2. 插桩改动

- **Super-LIO**（子模块 `third_party/Super-LIO`，未提交）：
  - `super_lio.cpp` OutputThread：`pub_odom` 后新增 `recordLatency("[Lidar->Odom]")`；
  - `super_lio.cpp` process()：`state_fn_` 调用整体包入 `time_record_.Evaluate(..., "[FrameProcess]")`；
  - `timer.h` PrintAll：打印新增 `std`（波动）。
- **FAST-LIVO2-ROS2**（子模块 `third_party/FAST-LIVO2-ROS2`，未提交）：
  - `common_lib.h` MeasureGroup 新增 `lio_recv_wall`；`LIVMapper.cpp` livox/std 回调打到达时刻，sync_packages 出队时随帧携带；
  - handleLIO：publish_odometry 后记录 `[Recv->Odom]`，t4 处记录 `[LIOTotal]`；run 循环记录 `[VIOTotal]`；
  - 新增 `printPerfStats()`：每 100 帧与退出时打印 `[PERF] n= avg= std= p95= max=`。
- **方案A/B**：沿用节点自带逐帧日志 `耗时 ms: … 合计=…`（约 1 s 采样一条）。

## 3. CPU 资源分配（公平性约束）

与 `third_party/Super-LIO/src/super_lio/launch/Livox_mid360_drone.py` 实机 launch 一致：

| 进程 | 绑定 | 说明 |
| --- | --- | --- |
| `ros2 bag play`（顶替 livox 驱动） | `taskset -c 4` | 大核 A76 #4 |
| `super_lio_node` | `taskset -c 7` + `lio.single_core:=true` | 大核 A76 #7，单核串行 |
| `fastlivo_mapping` | `taskset -c 7` + `OMP_NUM_THREADS=1` | 同上，单线程 |
| `visual_a_node` / `visual_b_node` | `taskset -c 7` | 与 LIO **同挤一个大核**（按既定要求） |
| `parameter_blackboard`（fast_livo 相机参数托管） | `taskset -c 0` | 仅初始化时查询，不影响 |

组间均留 ≥15 s 间隔，降低热漂移影响。

## 4. 结果

### 4.1 odom 发出延迟（雷达到达 → odom publish，ms）

| 系统 | avg | std（波动） | max | n |
| --- | --- | --- | --- | --- |
| Super-LIO 单跑 | **14.6** | 13.3 | 58.7 | 1317 |
| FAST-LIVO2 | **5284** | 6402 | 20154 | 985 |
| 方案A（LIO+visual_a 同核） | **16.1** | 15.3 | 112.7 | 1313 |
| 方案B（LIO+visual_b 同核） | **16.4** | 16.3 | 113.0 | 1315 |

### 4.2 整帧处理时间（ms）

| 系统 / 环节 | avg | std | P95 | max | n |
| --- | --- | --- | --- | --- | --- |
| Super-LIO `[FrameProcess]`（单跑） | 7.6 | 1.4 | — | 16.5 | 1326 |
| Super-LIO `[Observe]`（单跑） | 5.0 | 1.2 | — | 13.7 | 1317 |
| Super-LIO `[FrameProcess]`（@方案A） | 9.4 | 3.9 | — | 32.5 | 1323 |
| Super-LIO `[FrameProcess]`（@方案B） | 8.8 | 3.0 | — | 30.2 | 1325 |
| FAST-LIVO2 `[LIOTotal]` | 71.1 | 11.0 | 90.3 | 107.4 | 1300 |
| FAST-LIVO2 `[VIOTotal]` | 27.9 | 8.0 | 39.8 | 45.8 | 1301 |
| 方案A 视觉 `合计` | 21.9 | 4.7 | 29.1 | 37.8 | 124 |
| 方案B 视觉 `合计` | 41.8 | 27.2 | 112.2 | 174.1 | 125 |
| 方案B 视觉 `约束`（主要波动源） | 22.7 | 26.6 | 96.3 | 146.6 | 125 |

### 4.3 实时性

- Super-LIO 单跑：处理总耗时 ≈10.1 s / bag 135.5 s，**实时率 0.07×**，余量巨大。
- 方案A/B：LIO 侧 0.09×；视觉合计 21.9/41.8 ms 对图像周期（~50-100 ms）均可实时（B 临界）。
- FAST-LIVO2：LIO 71 ms + VIO 28 ms ≈ **99 ms/帧，贴着 100 ms 雷达帧周期、实际略慢于实时**。135.5 s 数据到"放完 bag + 30 s"仅发出 985/约1354 帧 odom，积压从 0 单调增至 ~20 s——odom 延迟均值 5.3 s 即队列积压的直接体现，并非算法内部处理慢。

## 5. 结论

1. **单大核公平预算下，Super-LIO 是四组中唯一"实时且低延迟"的 LIO**：odom 延迟均值 ~15 ms、std 13 ms，整帧处理仅 7.6 ms。std 偏大源于输出线程/DDS 与处理同挤 7 号核的调度竞争，max 58.7 ms 集中在 bag 收尾。
2. **官方 FAST-LIVO2 在 RK3588 单大核上跑不满实时**：处理需求 ≈0.99 帧周期，长序列必然积压，odom 延迟退化到秒级（均值 5.3 s、峰值 20 s）。如需部署 FAST-LIVO2，需放开多核（关闭绑核/单线程约束）或降帧频。
3. **方案A 代价很小**：视觉与 LIO 同挤 1 核时 odom 延迟仅 +1.5 ms（16.1 ms），视觉整帧 21.9 ms、波动小（std 4.7 ms），着色+约束链路可实时。
4. **方案B 影响相当但自身波动大**：odom 延迟 16.4 ms 与 A 几乎持平；但视觉整帧 41.8 ms、**std 27.2 ms、P95 112 ms 已超一帧周期**，瓶颈集中在约束计算（22.7±26.6 ms，MSCKF/2D-3D 特征规模波动所致）。单核预算下 B 处于实时边缘，建议后续：约束注入限规模，或给 B 单独一个大核再测。
   > **该结论已过期，需重测**：上表 B 的数据是"选轨视差扫描 O(n²)"修复**之前**采的（当时单帧最坏 422 ms 集中在 `constraintTracks`）。修复后（观测史截断 + 视差缓存）选轨 max 已降到 2.1 ms，B 是否仍处于实时边缘必须重跑本报告第 4 节才能定论；重测前不应引用 41.8 / 22.7 ms 这组数。

## 6. 复现

跑测脚本与原始日志在 `/tmp/bench/`（换机需重建）：

```bash
bash /tmp/bench/run1_slio.sh              # Super-LIO 单跑
bash /tmp/bench/run2_fl2.sh               # parameter_blackboard + FAST-LIVO2
bash /tmp/bench/run_ab.sh a               # Super-LIO + 方案A
bash /tmp/bench/run_ab.sh b               # Super-LIO + 方案B
python3 /tmp/bench/parse_bench.py slio.log fl2.log vis_a.log vis_b.log 135.47
```

### 换机/新环境注意事项（本次实机踩坑）

- agent/受限终端务必 `unset LD_LIBRARY_PATH`，否则旧 libstdc++ 压垮系统 cmake 与 rclpy；
- `ros2 bag play` 不支持 `--duration`；后台放 bag 必须 `--disable-keyboard-controls < /dev/null`；
- demo 的 `parameter_blackboard` 节点**不能加 `--name`**（`UnknownROSArgsError` 直接崩），默认节点名即 `/parameter_blackboard`；
- 首次编译需 `PYTHONDONTWRITEBYTECODE=1` 或放行沙箱对 `__pycache__` 的写入；
- 新机器依赖：`ros-humble-sophus`、`libgoogle-glog-dev`、`libfmt-dev`、Livox-SDK2（源码装到 /usr/local）、vikit_common（同法安装 + 手写 `vikit_commonConfig.cmake`）、`demo_nodes_cpp`。

### 已知偏差说明

- 方案A/B 视觉逐帧耗时为约 1 s 一条的采样（非全帧），P95/max 反映采样分布；
- bag 消息 stamp 为设备时钟域，与墙钟差 ~717580 s，不影响各系统内部延迟统计口径（均为到达→发出墙钟差）；
- Super-LIO 延迟统计为 60 s 窗口合并（总体精确值），FAST-LIVO2 为累计样本。
