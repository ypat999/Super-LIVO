# 历史 sl odom 重放复现性验证（REPLAY_REPRODUCIBILITY）

- 日期：2026-11-10
- 结论：**在合理范围内可复现**。同一 bag、同一配置、同一启动方式重放，历史 sl odom 219 帧中 208 帧按 header.stamp 精确配对（配对时间差中位数 0 µs）；首帧平移差为机器精度（5×10⁻¹⁵ m），此后误差以 ~0.5 cm/s 平滑增长，窗口末端约 12–13 cm，路径长度差 1.8%，无阶跃、无轨迹形态差异。**不满足逐位浮点复现**（异步线程/网络到达时序所致，属预期）。

## 1. 固定条件（历史 run vs 重放 run）

| 条件 | 历史 sl（2026-11-09 00:55） | 本次重放（replay3） | 一致性 |
|---|---|---|---|
| 输入 bag | `/home/ywj/slam_data/Retail_Street` | 同 | ✓ |
| 参数文件 | `r3live.yaml` | 同，sha256 `632e5721bd41f74431de273009c4ddf92744e8ab27d436811d93b31d64e1d2a1` | ✓ |
| 启动命令 | `ros2 run super_lio super_lio_node --ros-args --params-file .../r3live.yaml`（无任何 override） | 同 | ✓ |
| 生效 QoS | `lidar_qos_reliable: false`（历史日志实证 `/tmp/sf_slio.log:5`，采用代码默认值） | 同（false） | ✓ |
| 环境变量 | `ROS_LOCALHOST_ONLY=1`、`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` | 同 | ✓ |
| 时间窗口 | 历史 odom 覆盖 946685507.40–531.10 | 重放覆盖 438.5–572.9（窗口全含） | ✓ |
| **二进制** | Nov 6 21:02 构建 | **Nov 9 11:39 构建**（HEAD `67c0acd4`） | ✗（见 §2） |
| 软件 commit | Super-LIVO `1f370532` 前状态（工作树 Nov 9 00:55） | Super-LIVO `1f370532`；Super-LIO `67c0acd4` | 近似 |

说明：用户清单中提及的 `use_local_timestamp:=false`、`lidar_qos_reliable:=true` 两项 override 经查**并不属于历史 sl 的实际运行方式**（该参数不存在于 super_lio_node；`lidar_qos_reliable` 历史生效值为默认 false）。重放严格按历史实际生效配置执行，而非按清单字面值，否则反而引入差异。

## 2. 二进制差异（唯一未控制变量）

历史运行使用 Nov 6 21:02 二进制；当前 install 为 Nov 9 11:39 重建。两者之间 Super-LIO 的提交：

- `6fd998e` revert（回滚 326ed21 实验代码）——恢复原状；
- `d630122`/`276831b` Ouster uint32 时间换算修复——**不触及 Livox CustomMsg 路径**（livoxHandler L1043 独立）；
- `67c0acd4` exp 目录迁入精度实验——生产代码未改。

结论：对 r3live.yaml 下 Livox 链路，两版二进制应等价，但**无法完全排除编译环境差异**；若需严格排除，可在 Nov 6 源码状态重建后复跑（本轮不做）。

## 3. 对比方法

脚本 `/tmp/final_replay3.py`（产物已存 `/home/ywj/slam_data/geom_replay_20261110/cmp_curve3.csv`、`cmp_dp3.csv`）：

1. 两路 `/lio/odom` 按 header.stamp（数据域）配对，容差 ±2 ms；
2. 首配对帧刚体对齐（`R_al = R_hist(0)ᵀ·R_rep(0)`，`t_al = p_hist(0) − R_al·p_rep(0)`）；
3. 逐时刻计算世界系平移差、机体系（历史帧系）平移差、旋转差（`angle(R_histᵀ·R_rep·R_al)`）、路径长度。

## 4. 结果

### 4.1 配对质量

- 历史 219 帧 → 配对 **208**，未配对 **11**（509.37、510.38、513.45、519.43、520.22、521.54、521.82、522.06、522.96、528.94、530.24 s）。
- 已配对帧的 stamp 时间差中位数 **0 µs**（逐位相等）——两路运行消费了相同的 LiDAR/IMU 帧集合；11 个未配对帧对应历史运行中"最后 IMU 采样时刻"与重放差一个采样（IMU 到达时序抖动），非轨迹跳变。

### 4.2 误差量值（窗口 25 s）

| 指标 | 数值 |
|---|---|
| 首帧平移差 | 5×10⁻¹⁵ m（对齐后精确重合） |
| 平移差 median / P90 / max | 5.24 / 11.79 / 15.73 cm |
| 前 5 帧 / 后 5 帧平移差 | ≈0–2.2 cm / 11.7–13.3 cm |
| 旋转差 median / P90 / max | 0.21 / 0.42 / 1.09° |
| 路径长度（历史 / 重放） | 10.025 / 9.849 m（Δ = 17.6 cm，1.76%） |
| 窗口末端 yaw（历史 / 重放） | −58.02 / −58.46°（Δ = 0.44°） |
| 机体系偏移中位数 | [0.84, 3.60, −1.74] cm（y 向为主） |

### 4.3 逐时刻误差曲线（采样，完整见 cmp_curve3.csv）

```
t(s)   dp(mm)   dang(°)
 1.2     0.0      0.00
 4.7     7.5      0.19
 8.5    10.4      0.14
 12.5   12.1      0.12
 16.1   50.4      0.16      ← 增长过渡段
 17.8   60.7      0.05
 20.4   74.6      0.04
 21.9   85.5      0.05
 23.7  ~110–132    ≤0.1
```

- 形态：**平滑单调发散**，无阶跃（阶跃仅对应 §4.1 的 stamp 抖动帧，幅度仍 <3 cm）；发散率约 0.5 cm/s（≈ 路径的 0.5%/s），与异步 LIO 对到达时序敏感的特性一致。
- 局部特征：17–23 s 段误差增速最大，对应场景中转弯/立面切换运动段——两路轨迹在该段形状一致（yaw 差始终 ≤0.44°），仅累计位置差拉大。
- 未出现"某方向系统性错位"或整段平移的结构性偏差。

## 5. 判定

- **停止条件 2（历史 sl odom 可合理复现）：满足**。同 pipeline、同输入、首帧精确重合，窗口内轨迹差 ≤0.16 m/25 s（1.8% 路径），旋转 ≤1.1°，可判定为"同一算法、同一场景"的合理复现；差异来源为运行时时序抖动（+§2 的二进制版本差异，影响未排除但预计可忽略）。
- 该复现水平**足以支撑阶段 A（帧内几何，与轨迹无关）**；**阶段 B（帧间位姿精度）不能直接依赖跨 run 对比**——应使用同一次 run 内自洽的 odom+dense+原始流（统一数据集，见 GEOMETRY_DATASET_MANIFEST.md），并报告该 ±0.5 cm/s 量级的 run-to-run 噪声作为误差下限。

## 6. 复跑命令

```bash
source /opt/ros/humble/setup.bash; source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
python3 /tmp/final_replay3.py        # 全部统计 + cmp_curve3.csv/cmp_dp3.csv
# 重放本体：bash /tmp/replay3_run.sh（含录制），bag 见 GEOMETRY_DATASET_MANIFEST.md
```
