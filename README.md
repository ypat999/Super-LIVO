# Super-LIVO

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

基于 ROS2 Humble 的 LiDAR–视觉–惯性里程计（LIVO）研究仓库：以 [Super-LIO](https://github.com/ypat999/Super-LIO) 为定位主干，
外挂两条视觉纠正支路（**A 轻量 / B 紧耦合**），官方 FAST-LIVO2 作为"同问题、不同解法"的对照系。

核心问题只有一个：**当 LiDAR 几何约束退化（长走廊、无特征空间）时，视觉约束能不能、以及以多大算力代价把位姿拉回来。**

- 主干不改滤波器结构与地图结构：视觉以观测形式注入 Super-LIO 的 ESIKF
  （A 走退化门控 + 状态回灌，B 走统一 ESIKF + MSCKF 多帧约束 + 跨代绝对锚点）。
- 退化度量不自造：两条支路都只消费 Super-LIO 的退化签名（`/lio/degeneracy` 或进程内快照）。
- 附带与定位解耦的彩色点云组件，用于外参精度与同步质量的目视核验。

**硬件**：Livox Avia / Mid-360 + 全局快门相机 + 集成 IMU；实机 RK3588S（A76 大核 4–7）。
**评测数据**：NTU VIRAL `eee_03`、RTK-SLAM Dataset（全站仪 <1 cm 真值）、HKU Retail_Street、自采 bag。

## 特性

- **两套视觉方案 + 官方对照，统一单进程部署**：`visual_a_app` / `visual_b_app` 各自一个进程、一个
  ROS 节点内同时跑 LIO 主干与视觉支路，LIO→视觉走进程内钩子，无 DDS 回环。
- **退化对抗有重复实验支撑**（NTU 走廊裁剪 ×8 次单跑）：方案 B 8/8 次不劣于同 bag LIO，
  终点误差跨跑散布从 LIO 的 55× 压到 15×（均值 −73% / 中位数 −50%）。
- **完整相机-雷达外参标定链路**：标定板（FAST-Calib-ROS2，自有 fork）与无目标
  （direct_visual_lidar_calibration）两条路线；本机外参四孔重投影 1.00/1.62 px。
- **可复现的评测与测速工具链**：官方协议 ATE、三层点云偏差、四臂统一跑批与 RK3588 绑核测速。
- 结论与踩坑全部记录在 [experiments/](experiments/README.md) 与 [docs/](docs/)。

## 仓库结构

```
Super-LIVO/
├── common/super_livo_common/      # A/B 共用纯算法头文件库（退化签名、先验、标定 loader、计时）
├── pointcloud_colorizer/          # 彩色点云独立包（A/B 共用，与定位解耦）
├── solution_a_lightweight/        # 方案 A：轻量后处理
│   └── super_livo_visual_a/       #   可执行 visual_a_app + ROS 无关 VisualPipeline
├── solution_b_tight_coupled/      # 方案 B：紧耦合写回 ESIKF
│   └── super_livo_visual_b/       #   可执行 visual_b_app + VisualPipeline + ConstraintBridge
├── third_party/                   # 子模块：Super-LIO 主干、FAST-LIVO2 对照、标定工具等
├── config/                        # 数据集无关的配置与相机内参
├── common/super_livo_common/config/  # LIO 主干参数（livox_360 / lio_move 等）
├── experiments/                   # 实验记录、四臂跑批、消融与历史脚本
├── tools/                         # 评测、探针、跑批守卫脚本
├── docs/                          # 设计与专项审计文档
├── freqcmd.txt                    # 常用命令清单（编译/启动/部署/跑批）
└── README.md
```

## 环境依赖

- Ubuntu 22.04 + ROS 2 Humble（x86 开发机与 RK3588S 实机同一套构建流程）
- Eigen3、PCL、OpenCV、Boost、glog、gflags、yaml-cpp
- 可选：`livox_ros_driver2` 消息包（跑官方 FAST-LIVO2 对照臂需要）

完整的一次性部署（apt 列表、子模块、DDS 环境、WSL 专用坑、标定文件落位）见
[freqcmd.txt §B](freqcmd.txt)。

## 构建

```bash
# 1. 系统依赖
sudo apt install -y ros-humble-desktop ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-pcl-ros ros-humble-tf2-ros libeigen3-dev libpcl-dev libopencv-dev \
  libboost-all-dev libgoogle-glog-dev libgflags-dev libyaml-cpp-dev

# 2. 递归克隆（依赖以 submodule 固定在 third_party/ 下）
git clone --recursive https://github.com/ypat999/Super-LIVO.git
cd Super-LIVO

# 3. 构建 LIO 核（super_lio 既是纯 LIO 基线，也是对下游暴露的库）
source /opt/ros/humble/setup.bash
(cd third_party/Super-LIO && \
  colcon build --packages-select super_lio --cmake-args -DCMAKE_BUILD_TYPE=Release)

# 4. 构建公共库、着色组件与 A/B 两套方案
source third_party/Super-LIO/install/setup.bash
colcon build --packages-select super_livo_common pointcloud_colorizer \
  super_livo_visual_a super_livo_visual_b --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- **必须 Release 构建**：Debug 下着色耗时从 4~5 ms 劣化到几十 ms。
- `direct_visual_lidar_calibration` 自带子模块，需 `git submodule update --init --recursive` 后才能编译。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
source third_party/Super-LIO/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=93

# 纯 LIO 基线
ros2 run super_lio super_lio_node --ros-args \
  --params-file common/super_livo_common/config/livox_360.yaml
ros2 bag play <bag> --rate 1.0

# 方案 A（LIO 主干 + 视觉后处理）
ros2 run super_livo_visual_a visual_a_app --ros-args \
  --params-file common/super_livo_common/config/lio_move.yaml \
  --params-file solution_a_lightweight/super_livo_visual_a/config/visual_a_move.yaml \
  -p super_livo_visual.timing:=true

# 方案 B（LIO 主干 + 视觉约束写回 ESIKF）
ros2 run super_livo_visual_b visual_b_app --ros-args \
  --params-file common/super_livo_common/config/lio_move.yaml \
  --params-file solution_b_tight_coupled/super_livo_visual_b/config/visual_b_move.yaml \
  -p super_livo_visual.inject:=true -p super_livo_visual.timing:=true
```

- 退出用 `SIGINT`（Ctrl-C）：两臂都走 `shutdownAndSave()` 同步落盘，`kill -9` 会丢最后的彩色地图。
- 推荐用统一跑批（含互斥、残留清理、产物归集，禁止并行）：

```bash
OUTROOT=/home/ywj/slam_data/paper_$(date +%Y%m%d) RATE=1.0 \
  bash experiments/scripts/paper_matrix/run_one.sh test4 a
# seq: ntu01 ntu03 rtk retail move test4 room    arm: fl2 a b sl
```

更多命令（bag 回放、绑核、探针、清理）见 [freqcmd.txt](freqcmd.txt)。

## 参数文件规范（ROS 2 params YAML）

仓库内所有 params YAML（A/B 支路配置、LIO 主干配置、跑批用的合并文件）统一为同一种格式：
**顶层只允许 `/**` 通配 + `ros__parameters`，禁止在顶层写死节点名**。

```yaml
/**:
  ros__parameters:
    lio.ros.lidar_topic: "/livox/lidar"     # ① 原版 Super-LIO 主干段：扁平 lio.* 名字，与上游一字不动
    super_livo_visual:                       # ② 视觉支路段（A / B 共用同一套名字，同类已合并）
      cam_width: 1280
      Rcl: [-0.0171, -0.9998, -0.0064, ...]
      gate.max_reprojection_rmse: 6.0
      pcd.save_dir: "/home/ywj/slam_data/output/move_a"
```

- **摊平机制**：rcl 把 `ros__parameters:` 下的嵌套 map 摊平成点号参数名，落到参数服务上的真实名字是
  `super_livo_visual.cam_width`、`super_livo_visual.pcd.save_dir`。代码侧读取必须带此前缀：
  A 侧 `vp("name")` helper、B 侧 `kVisualParamPrefix`。
  **文档里写分支内短名，命令行 `-p` 覆盖必须写全名**（`-p super_livo_visual.pcd.save_dir:=…`）。
- **同一份文件可直接喂官方 `super_lio_node` 跑纯 LIO**：主干不认识 `super_livo_visual` 分支，
  对未声明的 override 静默忽略，不必维护两份配置。
- **标定源覆盖（最高优先级）**：分支内 `calib.camera_intrinsic_yaml` / `calib.fast_calib_result`
  给出路径且读取成功时，用 `config/camera_intrinsic.yaml` 的内参+畸变与 FAST-Calib 的外参
  覆盖本文件的 `cam_*` / `Rcl` / `Pcl`；路径为空或读取失败则沿用数据集 yaml。
  loader 在 `common/super_livo_common/include/super_livo_common/calibration_source.hpp`。

### A/B 同类参数的合并改名

方案 B 旧名（`lv.*` / `app.*` / `est.enable_visual`）中与 A 语义相同的一律并入 A 的命名族：

| B 旧名 | 新名（分支内短名） | 说明 |
| --- | --- | --- |
| `lv.huber_delta` / `sigma_pix` / `sigma_anchor` / `min_constraints` / `max_pixel_error` / `max_pixel_error_anchored` | `reproj.*` 同名 | 与 A 的 `reproj.*` 合并 |
| `lv.pixel_radius` | `depth.pixel_radius` | 同一概念（都喂 `associationVariance`） |
| `lv.anchor_keep_sec` | `depth.anchor_keep_sec` | — |
| `lv.prior_*`（pos/rot/vel，含 healthy/degen） | `update.prior_*` | 与 A 的 `update.prior_*` 合并 |
| `lv.prior_directional_en` | `update.directional_prior_en` | — |
| `est.enable_visual` | `update.enable` | 与 A 同义 |
| `app.embed_visual` / `app.visual_inject` / `app.visual_inject_max_dt` | `embed` / `inject` / `inject_max_dt` | 挂在 `super_livo_visual` 分支根下 |

刻意**不合并**的三组（机制不同，强行合并会改行为）：A 的 `gate.*`、B 的
`est.visual_info_ratio(_degen)`、B 的 `guard.*`。

| 归属 | 参数族 |
| --- | --- |
| A 独有 | `gate.*`、`klt.*`、`evo.*`、`image_save.*`、`calib.*` |
| B 独有 | `est.*`、`msckf.*`、`photo.*`、`rot_refine.*`、`scheduler.*`、`lifecycle.*`、`kf.*`、`guard.*`、`patch.*`、`e2.*` |
| A/B 同名 | `cam_*`、`Rcl`、`Pcl`、`scale`、`image_topic`、`imu_topic`、`align.*`、`color.*`、`pcd.*`、`image.qos_reliable` |

> `experiments/config/livox_360_visa*.yaml` 里的 `lio.visual.*` 是上游 Super-LIO 自带视觉的参数，
> 属于 ① 主干段，不参与合并、不改名。

统一过程与验收见 [exp_records §40](experiments/exp_records.md)。

## 方案与目录

设计说明见 [docs/monolith_refactor.md](docs/monolith_refactor.md)（单体化设计与实施）与
[experiments/gpt双方案设计.md](experiments/gpt双方案设计.md)（最初的双方案设计）。

| 目录 | 方案 | 内容 |
| --- | --- | --- |
| [solution_a_lightweight](solution_a_lightweight/README.md) | A：极致轻量化 | Super-LIO + FAST/PyrLK + 2D-3D 重投影 + 退化门控 + 状态回灌（`update.reinject`） |
| [solution_b_tight_coupled](solution_b_tight_coupled/README.md) | B：紧耦合 | Patch 前端（金字塔 LK）+ 特征生命周期/关键帧地图 + MSCKF + 统一 ESIKF + 退化感知先验与守卫 |
| [pointcloud_colorizer](pointcloud_colorizer/README.md) | 共用组件 | 彩色点云独立包（逐点运动补偿 + 遮挡剔除 + 1 cm 体素时间中位抗椒盐） |
| [common/super_livo_common](common/super_livo_common/README.md) | 共用组件 | 退化签名映射、各向异性先验、测量方差、标定 loader、延时计时 |

### 公共库与独立可执行

`third_party/Super-LIO` 的 `super_lio` 只作为**公共代码库**被引用
（`find_package(super_lio REQUIRED)` + `super_lio::lio`）：

| 可执行 | 所属 | 内容 |
| --- | --- | --- |
| `super_lio_node` | `third_party/Super-LIO` | 纯 LIO 基线，不含任何视觉代码 |
| `visual_a_app` | `solution_a_lightweight` | 单进程：Super-LIO 主干 + 方案 A 视觉支路 |
| `visual_b_app` | `solution_b_tight_coupled` | 单进程：Super-LIO 主干 + 方案 B 视觉支路 + 约束写回 ESIKF |

`super_lio` 对下游只暴露两个与视觉无关的扩展点，默认路径逐位不变：

- `virtual void ExternalObserve()`：每帧 LiDAR 更新之后、建图与输出之前的空钩子；
- `ESKF::UpdateObserveFull(H, r, R)`：任意维观测更新（H 列序为 18 维误差状态）。

## 子模块（third_party）

```bash
git clone --recursive https://github.com/ypat999/Super-LIVO.git
# 已克隆过：
git submodule update --init --recursive
```

| 路径 | 上游仓库 | 分支 | 固定提交 |
| --- | --- | --- | --- |
| `third_party/FAST-LIVO2-ROS2` | https://github.com/ypat999/FAST-LIVO2-ROS2.git（fork 自 U-AMC/FAST-LIVO2-ROS2） | main | `cc47106` |
| `third_party/Super-LIO` | https://github.com/ypat999/Super-LIO.git | ros2 → no-de-check → exp/degeneracy-adaptive | `326ed21`（工作区检出 `67c0acd`） |
| `third_party/basalt` | https://github.com/VladyslavUsenko/basalt.git | master | `0f3b2b5` |
| `third_party/rpg_svo_pro_open` | https://github.com/uzh-rpg/rpg_svo_pro_open.git | master | `ca371f3` |
| `third_party/direct_visual_lidar_calibration` | https://github.com/koide3/direct_visual_lidar_calibration.git | main | `02a0dc0` |
| `third_party/FAST-Calib-ROS2` | https://github.com/ypat999/FAST-Calib-ROS2.git（fork 自 MaroSalah02/FAST-Calib-ROS2） | main | `8963f32` |

Super-LIO 与 FAST-LIVO2-ROS2 为自有 fork：

- Super-LIO 的实验改动在 `no-de-check` / `exp/degeneracy-adaptive` 分支（beam_err/dept_err
  距离自适应方差、K=8 拟合邻域、平面残差软降权等），另加了与视觉无关的
  `onLioOutput` / `onDenseCloud` 钩子与 `degeneracySnapshot()`。
- FAST-LIVO2-ROS2 在上游 `a09599a` 之上有三批改动：
  ① `ae28b0c` 修三处功能缺陷（`blind_sqr` 分支未赋值、MID360 话题订阅缺失、odom 时间戳改用
  lidar 帧时刻）；② `b9f931f` 加滚动淘汰视觉地图点（`pruneVisualMap`），解决长序列 OOM；
  ③ `cc47106` 加图像入 VIO 前去畸变（`camera.undistort_en` / `camera.dist_coeffs`）。

## 相机-雷达外参标定

两个标定子模块，分别对应"标定板"与"无目标"两条路线：

1. **标定板**：`FAST-Calib-ROS2` —— 四 ArUco + 四圆孔板，PnP 求外参，**不需要初值**，
   单场景即可出结果。参数在 `third_party/FAST-Calib-ROS2/config/qr_params.yaml`。
2. **无目标**：`direct_visual_lidar_calibration` —— 全自动直接视觉-雷达配准，无需标定板也无需初值。

本机相机（海康 MV-CB013，1280×1024 全局快门）的实测内参与畸变归档在
[config/camera_intrinsic.yaml](config/camera_intrinsic.yaml)；`qr_params.yaml` 与
`config/move_fastcalib.yaml` 的内参均出自该文件。

**标定板法向退化修正（holefix，2026-11-13）**：`qr_params.yaml` 的 `board_normal`
与雷达实测板面法向差 9.46°，导致孔心求解退化（四孔心偏离真实板面 24–90 mm）。修正为：
真实板面 RANSAC 拟合（σ=4.08 mm）+ 孔心取前表面（+8 mm）+ ±1° 扇区最小半径取点 +
重力消歧。复标定后四孔重投影 1.00/1.62 px（FAST-Calib 直出 1.26/2.17 px），
新外参已写入 A/B 的 `*_move.yaml` 与 `config/move_fastcalib.yaml`：

```
Rcl = [-0.016751, -0.999859, -0.001174,
        0.458555, -0.006640, -0.888641,
        0.888508, -0.015424,  0.458601]
Pcl = [0.016642, -0.113077, -0.037398]
```

远场（5–15 m）边缘距离场对两版外参不可分辨（差值 <1 px，本底 3–16 px），故以标定板残差最优为准。
细节见 [exp_records.md](experiments/exp_records.md)。

## 评测与工具

| 工具 | 用途 |
| --- | --- |
| [experiments/scripts/paper_matrix/](experiments/scripts/paper_matrix/) | 四臂（fl2/a/b/sl）× 七序列统一跑批，产物含 odom/pcd/latency/meta |
| `tools/odom_vs_gt.py` | 官方协议 ATE（T_B_PRISM + combteeth 0.1 s + SE(3) Umeyama） |
| `tools/rtk_pcdev.py` | 三层点云偏差（L1 散布 / L2 重影 / L3 刚体偏置） |
| `tools/traj_compare.py` / `tools/odom_vs_gt.py` | 轨迹逐点对比 |
| `tools/latency_probe_cpp` | 外部端到端延迟探针（C++ generic subscription，不反序列化） |
| `tools/ros_batch_guard.sh` | 跑批互斥与残留检查 |

实验结论、对照数据、RK3588 测速方法与待办实验见 [experiments/README.md](experiments/README.md)。

## 文档索引

| 文档 | 内容 |
| --- | --- |
| [docs/monolith_refactor.md](docs/monolith_refactor.md) | A/B 单体化：目标、架构、公共库改动、分阶段实施、验证数据、踩坑 |
| [docs/HANDOVER_2026-11-08.md](docs/HANDOVER_2026-11-08.md) | 工作主线交接：染色诊断链、官方图像消融、RTK-SLAM 评测、滚动淘汰 |
| [docs/](docs/) | 专项审计：地图更新机制、OctVox、去畸变管线、点云质量指标、数据清单 |
| [experiments/README.md](experiments/README.md) | 实验总览：与 FL2 的差异、关键结论、测速方法、待办 |
| [experiments/exp_records.md](experiments/exp_records.md) | 主线实验日志（逐次改动的数据与结论） |
| [freqcmd.txt](freqcmd.txt) | 编译、A/B 启动、环境部署、bag 回放与清理命令 |
| [experiments/speed_test.md](experiments/speed_test.md) | RK3588 实机测速原始报告 |

## 已知注意事项

- **Release 构建**，否则着色与前端耗时不可用。
- **退出用 SIGINT**，`kill -9` 会丢最终彩色地图。
- **跑批禁止并行**：`run_one.sh` 自带 `flock` 与残留清理，绕过它手工起多个实例会因
  FastDDS 共享内存端口冲突导致 0 odom。
- **图像时钟**：`move_ros2` / `test4_move` 的图像 header 是主机时钟，必须用
  `super_livo_visual.img_time_offset` 覆盖，否则图像时间落在 2026 年。
- **WSL**：需删除 `127.0.0.1` 的 loopback 单播路由，且统一用 FastDDS（CycloneDDS 回放会掉帧）。
- **数据侧问题**：自采 2026-11 move bag 的"幽灵层"源于相机话题断流 2.62 s，只能重录，
  不要从算法层修（详见实验结论）。

## 更新日志

- **2026-11-19**：全仓 params YAML 统一为 `/**` + `ros__parameters` + `super_livo_visual` 分支；
  A/B 完成单体化（S2），`visual_{a,b}_node` 删除，LIO→视觉全部进程内直推；
  B 修复退出落盘（`shutdownAndSave()`）。
- **2026-11**：B 前端由单层 SSD 改金字塔 LK；选轨 O(n²) 修复（单帧最坏 422 → 2.1 ms）；
  holefix 重标外参；官方 FAST-LIVO2 长序列 OOM 修复。
- 历史明细见 [experiments/exp_records.md](experiments/exp_records.md)。

## 致谢

本仓库建立并参考于以下开源工作（各子模块固定提交见上表）：

- [Super-LIO](https://github.com/ypat999/Super-LIO)：LIO 主干；
- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) 及 [FAST-LIVO2-ROS2](https://github.com/U-AMC/FAST-LIVO2-ROS2)：对照系统；
- [rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open)、[basalt](https://github.com/VladyslavUsenko/basalt)：视觉前端与特征生命周期管理参考；
- [FAST-Calib](https://github.com/xuankuzcr/FAST-Calib) 及 [FAST-Calib-ROS2](https://github.com/MaroSalah02/FAST-Calib-ROS2)：标定板外参；
- [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)：无目标外参；
- [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration)：标定板板型来源。

## License

本仓库代码以 [MIT License](LICENSE) 开源；`third_party/` 各子模块遵循其上游原始许可证。
