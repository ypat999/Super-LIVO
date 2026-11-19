# Super-LIVO

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

基于 ROS2 Humble 的 LiDAR–视觉–惯性里程计（LIVO）研究仓库：以 Super-LIO 为定位主干，外挂两条视觉纠正支路（**A 轻量 / B 紧耦合**），官方 FAST-LIVO2 作为"同问题、不同解法"的对照系。

## 项目简介

核心问题只有一个：**当 LiDAR 几何约束退化（长走廊、无特征空间）时，视觉约束能不能、以及以多大算力代价把位姿拉回来。**

- 主干不改滤波器结构、不改地图结构：视觉以观测的形式注入 Super-LIO 的 ESIKF（方案 A 走退化门控 + 状态回灌，方案 B 走统一 ESIKF + MSCKF 多帧约束 + 跨代绝对锚点）。
- 退化度量不自造：两条支路都只消费 Super-LIO 的 `/lio/degeneracy` 真度量。
- 附带与定位解耦的**彩色点云**组件，用于外参精度与同步质量的目视核验。

**硬件**：Livox Avia / Mid-360 + 全局快门相机 + 集成 IMU；实机平台 RK3588S（A76 大核 4–7）。
**评测数据**：NTU VIRAL `eee_03`、RTK-SLAM Dataset（全站仪 <1 cm 真值）、HKU Retail_Street、自采 bag。

## Highlights

- **两套方案均已实现并接入 ROS2 节点**：A 视觉合计 21.9 ms/帧（RK3588 单大核），B 具备独立短期运动估计能力（Patch 金字塔 LK 前端 + MSCKF）。
- **退化对抗有重复实验支撑**（NTU 走廊裁剪 ×8 次单跑）：方案 B 8/8 次不劣于同 bag LIO，终点误差跨跑散布从 LIO 的 55× 压到 15×（均值 −73% / 中位数 −50%）。
- **完整相机-雷达外参标定链路**：标定板（FAST-Calib-ROS2，自有 fork）与无目标（direct_visual_lidar_calibration）两条路线，含板面法向退化等真实问题的修正记录（孔心重投影 5.83 → 1.00 px）。
- **评测与测速工具链**：官方协议 ATE（`tools/odom_vs_gt.py`）、三层点云偏差（`tools/rtk_pcdev.py`）、RK3588 绑核公平测速（[speed_test.md](speed_test.md)）。
- **踩坑全部写在文档里**：FAST-LIVO2 长序列 OOM 的成因与修复、测速公平性口径、数据集标定误用等，见「关键实验结论」。

## 更新日志

- **2026-11-19**：**全仓 params YAML 统一为单一格式**（顶层 `/**` + `ros__parameters`，视觉支路全部收进
  `super_livo_visual:` 分支，A/B 同类参数合并改名，B 的 `lv.*` / `app.*` / `est.enable_visual` 并入 A 的
  `reproj.*` / `depth.*` / `update.*` 与分支根的 `embed` / `inject` / `inject_max_dt`）。合并后的同一份文件
  既能喂 `visual_{a,b}_app`，也能直接喂官方 `super_lio_node` 跑纯 LIO。规范与改名表见
  「[参数文件规范](#参数文件规范ros-2-params-yaml)」，过程与验收见 [exp_records §40](experiments/exp_records.md)。
- **2026-11-19**：方案 B 完成单体化（S2）并修掉退出落盘。`visual_b_app` 现在一个进程、一个节点
  （`ROSWrapper` + `VisualPipeline`），LIO→视觉全部进程内直推，`visual_b_node` 删除；
  配套把 `run_one.sh` 的 b 臂改起单进程、三份数据集配置顶层键改 `/**`（单体节点名是
  `super_lio`，写旧名字整块配置会静默失效）。新增 `shutdownAndSave()`：**退出时同步写**最终
  彩色地图（此前交给异步 save 线程，进程退出即被杀，test4 全量跑批 `pcd/` 只有目录没有文件）。
  启动时间对齐（`off = −(D + a)`）在单体下继承，离线回放因系统钟与图像 header 相差约 15.5 万秒
  按设计回退配置 `img_time_offset`。A 侧同批加入启动时间对齐与位姿按曝光时刻取用。
  见 [方案 B README §9.11](solution_b_tight_coupled/README.md)、
  [monolith_refactor_progress.md](monolith_refactor_progress.md)。
  ⚠️ 单体路径 `super_livo_visual.inject`（旧名 `app.visual_inject`）默认开，b 臂从此真正把视觉约束写回 ESKF，
  `paper_20261119` 的双进程 B 结果不可与之混比。
- **2026-11**：方案 B 前端与位姿链路加固：Patch 跟踪由单层 SSD 改**金字塔 LK**（Retail_Street 大位移下视觉相对旋转偏 1.5–2° 的问题消除）；`b_own` 速度观测 / 守卫 / 10 Hz 同频三层修复，彩色点云 634 万 → **1711 万点**；新增旋转后处理通道（默认关，eee_01 上 0.1 s 区间旋转 RMS 0.281° → 0.257°，偏航略退化）。见 [方案 B README](solution_b_tight_coupled/README.md#97-patch-前端单层-ssd--金字塔-lk)。
- **2026-11**：标定板法向退化修正（holefix）重标 A/B 外参，四孔重投影 5.83 → 1.00 px（2026-11-16 用 all bag 全量点云 + 同步后的标定参数复标定：FAST-Calib RMSE 7.9 mm、holefix 1.00/1.62 px）；FAST-Calib-ROS2 改动推入自有 fork（`bb1e748`、`8963f32`）。见「相机-雷达外参标定流程」。
- **2026-11**：官方 FAST-LIVO2 长序列 OOM 修复（视觉地图点滚动淘汰，fork `b9f931f`），本机 23 GB 内存跑满 99.3% 数据时长。
- **2026-11**：方案 B 退化对抗 8 次重复实验定版；选轨 O(n²) 耗时修复（单帧最坏 422 → 2.1 ms）。

## Quick Start（环境与构建）

环境：Ubuntu 22.04 + ROS2 Humble（x86 开发机与 RK3588S 实机同一套构建流程）。

```bash
# 1. 系统依赖（新机器一次性）
sudo apt install -y ros-humble-desktop ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-pcl-ros ros-humble-tf2-ros libeigen3-dev libpcl-dev libopencv-dev \
  libboost-all-dev libgoogle-glog-dev libgflags-dev libyaml-cpp-dev

# 2. 递归克隆（依赖以 submodule 固定在 third_party/ 下）
git clone --recursive https://github.com/ypat999/Super-LIVO.git
cd Super-LIVO

# 3. 构建 LIO 核（super_lio 既是纯 LIO 基线，也是对下游暴露的库）
source /opt/ros/humble/setup.bash
cd third_party/Super-LIO
colcon build --packages-select super_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ../..

# 4. 构建公共库、着色组件与 A/B 两套方案
source third_party/Super-LIO/install/setup.bash
colcon build --packages-select super_livo_common pointcloud_colorizer \
  super_livo_visual_a super_livo_visual_b --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- **必须 Release 构建**：Debug 下着色耗时从 4~5 ms 劣化到几十 ms。
- `direct_visual_lidar_calibration` 自带子模块（iridescence 等），需 `git submodule update --init --recursive` 递归拉取后才能编译。
- bag 回放、绑核测速、评测命令与已知坑见 [freqcmd.txt](freqcmd.txt)；两套方案的算法细节见 [solution_a_lightweight](solution_a_lightweight/README.md) / [solution_b_tight_coupled](solution_b_tight_coupled/README.md) 的 README。
- 所有 `--params-file` 与 `-p` 覆盖的写法遵循下一节「[参数文件规范](#参数文件规范ros-2-params-yaml)」：
  视觉参数挂在 `super_livo_visual` 分支下，`-p` 必须写带前缀的全名。

## 参数文件规范（ROS 2 params YAML）

仓库内所有 params YAML（A/B 支路配置、LIO 主干配置、跑批用的合并文件）统一为同一种格式：
**顶层只允许 `/**` 通配 + `ros__parameters`，禁止在顶层写死节点名**。

```yaml
/**:
  ros__parameters:
    lio.ros.lidar_topic: "/livox/lidar"     # ① 原版 Super-LIO 主干段：扁平 lio.* 名字，与上游一字不动
    super_livo_visual:                       # ② 视觉支路段（方案 A / 方案 B 共用同一套名字，同类已合并）
        cam_width: 1280
        Rcl: [-0.0171, -0.9998, -0.0064, ...]
        gate.max_reprojection_rmse: 6.0
        pcd:
            save_dir: "/home/ywj/slam_data/output/move_a"
```

- **摊平机制**：Humble 的 rcl 会把 `ros__parameters:` 下的嵌套 map 摊平成点号参数名，落到参数服务
  上的真实名字是 `super_livo_visual.cam_width`、`super_livo_visual.pcd.save_dir`。代码侧读取必须带
  这个前缀：A 侧在
  `solution_a_lightweight/super_livo_visual_a/src/app/visual_a_app.cpp`
  的 `vp("name")` helper（旧多 node 已删除，helper 从原 `visual_a_node.hpp` 搬入）；B 侧在
  `solution_b_tight_coupled/super_livo_visual_b/src/app/visual_b_app.cpp` 的 `getOrDeclare` 内用
  `kVisualParamPrefix` 统一加。**文档里写分支内短名（`gate.*`、`pcd.save_dir` 等）时一律省略前缀；
  命令行 `-p` 覆盖必须写全名**（`-p super_livo_visual.pcd.save_dir:=…`）。
- **同一份文件可直接喂官方 `super_lio_node` 跑纯 LIO**：主干不认识 `super_livo_visual` 分支，
  rclcpp 对未声明的 override 静默忽略，因此不必再为"纯 LIO / 带视觉"维护两份配置。
- **消灭的历史坑**：此前 B 的三份配置顶层写死 `super_livo_visual_b:`，而单体进程里节点名是
  `super_lio`，整块参数不匹配 ⇒ 内参、时延、各开关全部回落代码默认值且**不报错**。
- **标定源覆盖（各取所长，最高优先级）**：A/B 支路配置分支内的 `calib.camera_intrinsic_yaml` /
  `calib.fast_calib_result` 若给出路径且读取成功，则用 `config/camera_intrinsic.yaml` 的
  内参+畸变与 `third_party/FAST-Calib-ROS2/output/calib_result.txt` 的外参 Rcl/Pcl 覆盖本文件的
  `cam_*`/`Rcl`/`Pcl`（内参以 intrinsic.yaml 优先，FAST-Calib 内参仅作后备）；路径为空或读取失败
  沿用数据集 yaml——非标数据集只改 yaml 即可。共用 loader 在
  `common/super_livo_common/include/super_livo_common/calibration_source.hpp`，move 数据集 yaml
  已配置默认路径（其余数据集留空）。

### A/B 同类参数的合并改名

方案 B 旧名（`lv.*` / `app.*` / `est.enable_visual`）中与方案 A 语义相同的一律并入 A 的命名族；
其余名字不变，只是整体加上 `super_livo_visual.` 前缀：

| B 旧名 | 新名（分支内短名） | 说明 |
| --- | --- | --- |
| `lv.huber_delta` / `sigma_pix` / `sigma_anchor` / `min_constraints` / `max_pixel_error` / `max_pixel_error_anchored` | `reproj.*` 同名 | 与 A 的 `reproj.*` 合并 |
| `lv.pixel_radius` | `depth.pixel_radius` | 与 A 的 `depth.pixel_radius` 同一概念（两处都喂 `associationVariance`） |
| `lv.anchor_keep_sec` | `depth.anchor_keep_sec` | — |
| `lv.prior_pos_sigma` / `prior_pos_sigma_degen` / `prior_pos_sigma_healthy` / `prior_rot_sigma` / `prior_vel_sigma` | `update.prior_*` | 与 A 的 `update.prior_*` 合并 |
| `lv.prior_directional_en` | `update.directional_prior_en` | — |
| `est.enable_visual` | `update.enable` | 与 A 的 `update.enable` 同义 |
| `app.embed_visual` / `app.visual_inject` / `app.visual_inject_max_dt` | `embed` / `inject` / `inject_max_dt` | 挂在 `super_livo_visual` 分支根下 |

刻意**不合并**的三组（机制不同，强行合并会改行为）：A 的 `gate.*`（退化时整帧门控 / 降权阈值）、
B 的 `est.visual_info_ratio(_degen)`（信息矩阵比例）、B 的 `guard.*`（LIO 跳变守护）。

| 归属 | 参数族 |
| --- | --- |
| A 独有 | `gate.*`、`klt.*`、`evo.*`、`image_save.*`、`calib.*`（`lio_source` 随 S2 清理删除） |
| B 独有 | `est.*`、`msckf.*`、`photo.*`、`rot_refine.*`、`scheduler.*`、`lifecycle.*`、`kf.*`、`guard.*`、`patch.*`、`e2.*` |
| A/B 同名 | `cam_*`、`Rcl`、`Pcl`、`scale`、`image_topic`、`imu_topic`、`align.*`、`color.*`、`pcd.*`、`image.qos_reliable` |

> `experiments/config/livox_360_visa*.yaml` 里的 `lio.visual.*` 是**上游 Super-LIO 自带视觉**的参数，
> 属于 ① 主干段，不参与合并、不改名。

统一过程、改名对照的实测验收与受影响文件清单见
[experiments/exp_records.md §40](experiments/exp_records.md)。

## 方案目录

设计说明见 [gpt双方案设计.md](gpt双方案设计.md)。两套方案**均已实现算法并接入 ROS2 节点**，
在 NTU VIRAL `eee_03` 上完成评测：

| 目录 | 方案 | 内容 |
| --- | --- | --- |
| [solution_a_lightweight](solution_a_lightweight/README.md) | A：极致轻量化 | Super-LIO + FAST/PyrLK + 2D-3D 重投影 + 退化门控（消费 `/lio/degeneracy` 真度量）+ 状态回灌（`update.reinject`） |
| [solution_b_tight_coupled](solution_b_tight_coupled/README.md) | B：紧耦合 | Patch 前端（金字塔 LK）+ 特征生命周期/关键帧地图 + MSCKF 多帧约束 + 统一 ESIKF + 延迟地图提交；LiDAR 地图绝对锚定 + 历史锚点保持 + 退化感知先验与守卫 |
| [pointcloud_colorizer](pointcloud_colorizer/README.md) | 共用组件 | 彩色点云独立包（A/B 共用，与定位解耦；逐点运动补偿 + 遮挡剔除 + 1cm 体素时间中位抗椒盐） |
| [common/super_livo_common](common/super_livo_common/README.md) | 共用组件 | A/B 共用的纯算法头文件库：退化签名映射、各向异性先验、重投影测量方差（口径只改一处） |

### 公共库与独立可执行

`third_party/Super-LIO` 的 `super_lio` 只作为**公共代码库**被引用
（`find_package(super_lio REQUIRED)` + `super_lio::lio`），A/B 各自是完整、独立可执行的方案：

| 可执行 | 所属 | 内容 |
| --- | --- | --- |
| `super_lio_node` | `third_party/Super-LIO` | 纯 LIO 基线，不含任何视觉代码 |
| `visual_a_app` | `solution_a_lightweight` | 同进程：Super-LIO 主干 + 方案 A 视觉支路 |
| `visual_b_app` | `solution_b_tight_coupled` | 同进程：Super-LIO 主干 + 方案 B 视觉支路 + 视觉约束写回 ESIKF |

`super_lio` 对下游只暴露两个与视觉无关的通用扩展点，默认路径逐位不变：

- `virtual void ExternalObserve()`：每帧 LiDAR 更新（含信任域裁决）之后、建图与输出之前的空钩子；
- `ESKF::UpdateObserveFull(H, r, R)`：任意维观测更新（H 列序为 18 维误差状态）。

## 与官方 FAST-LIVO2 的差异

先说定位：A/B **不是从 FAST-LIVO2 改出来的**，而是挂在 Super-LIO 外部的视觉节点；官方
FAST-LIVO2 在这里是"同问题、不同解法"的对照系。三者的核心分歧不在估计器（都跑 ESIKF），
而在**视觉观测怎么构造、以谁为锚点**：

| 维度 | 官方 FAST-LIVO2 | 方案 A（轻量） | 方案 B（紧耦合） |
| --- | --- | --- | --- |
| 基座 | 自带 LIO+VIO 一体的 ESIKF | Super-LIO（复用其 ESIKF） | Super-LIO（复用其 ESIKF） |
| 视觉观测形式 | LiDAR 地图点投影到图像，取 **patch 光度误差**（直接法） | 角点（FAST + PyrLK）+ LiDAR 深度关联，**2D-3D 重投影误差**（几何法） | 同 A 的几何法，另加 **MSCKF 多帧零空间约束** |
| 视觉锚点来源 | 全局 LiDAR 地图点（地图漂了锚点就漂） | 全局 LiDAR 地图点（同 FL2，深度来自地图） | 地图点 **+ 健康期建立的持久锚点**（跨代绝对锚定，`anchors_` 只在成功观测时刷新 last_seen） |
| 是否依赖全局地图 | 是 | 是 | 部分否（MSCKF 用多帧位姿三角化，不依赖地图） |
| 退化处理 | 无显式退化检测，持续融合 | 消费 `/lio/degeneracy`，退化时才注入 + 状态回灌（`update.reinject`） | 消费 `/lio/degeneracy`，cond/sev 映射成先验注入强度，另加先验否决/速度限幅守卫 |
| 先验信息上限 | 无显式上限 | 有 | 有（`visual_info_ratio` 常态 3× / 退化 10×）+ σ 地板，防止一帧压塌协方差 |
| 视觉计算量（RK3588 单大核） | LIO 71 + VIO 28 ≈ 99 ms/帧 | 视觉合计 21.9 ms（std 4.7） | 视觉合计 41.8 ms（std 27.2，Patch 前端改 LK 前口径，未重测） |
| 真值精度（NTU eee_03 ATE） | **0.028 m**（最强） | 未单列（对位姿几乎无纠正力） | 见 §9.4，退化段方向稳定 |

一句话概括：

- **FL2 把视觉当"地图的稠密光度校正"**——精度天花板最高，代价是计算最重、且对 LiDAR 地图
  退化没有专门的对抗手段（地图一漂，光度残差就跟着错）。
- **A 把视觉当"退化时的稀疏几何补充"**——代价最小，但走廊段可用重投影约束太少
  （实测 n=0~33），对位姿几乎无纠正力。
- **B 把视觉当"不依赖地图的多帧几何约束 + 跨代绝对锚点"**——纠正力最强且方向稳定，
  代价是计算最重、实时性临界。

## 子模块（third_party）

依赖代码以 git submodule 方式固定在 `third_party/` 下，克隆时需递归拉取：

```bash
git clone --recursive https://github.com/ypat999/Super-LIVO.git
# 已克隆过的主仓库：
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

说明：后两个为相机-雷达外参标定工具（ROS2 版）。Super-LIO 与 FAST-LIVO2-ROS2 为自有 fork：
Super-LIO 的实验改动在 `no-de-check` / `exp/degeneracy-adaptive` 分支上推进（beam_err/dept_err
距离自适应方差、K=8 拟合邻域、平面残差软降权等）；FAST-LIVO2-ROS2 的 fork 在上游 `a09599a`
之上有三批改动：
① `ae28b0c` 修三处功能缺陷——`blind_sqr` 分支未赋值、MID360 话题订阅缺失、发布 odom 的时间戳
改用 lidar 帧时刻（原为系统时间，评测时会被 evo 判成时间轴错位）；
② `b9f931f` 加**滚动淘汰视觉地图点**（`pruneVisualMap`），解决视觉地图随时间线性增长导致长序列
OOM 的问题，详见「关键实验结论」；
③ `cc47106` 加**图像入 VIO 前去畸变**（参数 `camera.undistort_en` / `camera.dist_coeffs`）：
vikit PinholeCamera 只做针孔投影（`cam_d0..d3` 读取但不使用），而 FAST-Calib 求 Rcl/Pcl 是
"先 cv::undistort、再纯针孔 K 投影"，直接拿针孔模型投影原始畸变图在中部差约 9 px、边缘反向
约差 7 px，表现为墙面/门框边缘着色错位。其余子模块保持上游原始代码。

## 相机-雷达外参标定流程

两个标定子模块，分别对应"标定板"与"无目标"两条路线：

1. **标定板**：`FAST-Calib-ROS2` —— 目标板为四 ArUco + 四圆孔（板型基于
   `velo2cam_calibration`），图像侧检 ArUco 角点、雷达侧检圆孔中心，PnP 求外参，
   **不需要初值**，单场景即可出结果，多场景联合可进一步提升精度。
   参数在 `third_party/FAST-Calib-ROS2/config/qr_params.yaml`（内参 + 板尺寸 +
   距离滤波范围）。
2. **无目标**：`direct_visual_lidar_calibration` —— 全自动直接视觉-雷达配准，
   用环境结构与纹理，**无需标定板也无需初值**。

标定结果写回 SLAM 配置外参，用彩色点云与图像边缘对齐验证（根目录 preview 图流程）。

本机相机（海康 MV-CB013，1280x1024 全局快门）的实测内参与畸变系数归档在
[`config/camera_intrinsic.yaml`](config/camera_intrinsic.yaml)：`qr_params.yaml` 的内参、
`config/move_fastcalib.yaml` 的 `camera.dist_coeffs` 均出自该文件（同一标定结果）。

### 标定板厚度/安装框的影响（2026-11-13 修正）

标定板为官方 FAST-Calib 板 **1.3×0.9×0.015 m**（marker 0.2 m、孔距 0.5/0.4 m、孔半径 0.12 m），
四孔为通孔（有圆柱内壁），板装在铝框上，雷达点位会打在
孔内壁与框上。由此暴露的真实问题不是板厚（量级 4–8 mm），而是 `qr_params.yaml` 里的
`board_normal = (-0.973, -0.004, -0.229)` 与雷达实测板面法向 `(0.923, 0.003, 0.385)`
**差 9.46°**：四孔共面，法向无法由孔心约束，法向先验一错孔心求解即退化，四个孔心离
真实板面分别偏差 90.4 / 91.4 / 23.6 / 24.5 mm（远超板厚 15 mm）。

修正做法（`holefix`）：

- 板面法向改用 338,796 点 RANSAC 拟合的真实板面（σ=4.08 mm，d=−2.04357）；
- 孔心取**前表面**（沿法向 +8 mm，即板厚一半），而非名义板面；
- 每个孔只在 **±1° 扇区的最小半径轮廓**上取点，自动避开孔内壁和安装框的杂点；
- 半径/边长自检：0.1244/0.1265/0.1289/0.1280 m，长边 493.6/490.9（名义 500）、
  短边 398.1/394.9（名义 400）mm，面内残差 1.9/2.0/4.2/4.2 mm；
- 180° 歧义用重力方向消解（雷达 +z=上 → 相机 −y）。

新外参（2026-11-16 复标定定稿，已写入 A/B 的 `*_move.yaml` 与 `config/move_fastcalib.yaml`）：

```
Rcl = [-0.016751, -0.999859, -0.001174,
        0.458555, -0.006640, -0.888641,
        0.888508, -0.015424,  0.458601]
Pcl = [0.016642, -0.113077, -0.037398]
```

2026-11-16 用同步后的标定参数（FAST-Calib-ROS2 `8963f32`：MV-CB013 内参 / 板型 / 距离门限全精度）
在 `all_ros2` 全段累积点云（8,260,032 点）+ `img_0120.png` 上复标定：FAST-Calib 直出 RMSE **7.9 mm**，
其外参与原 remote7.9 逐位一致；再用上述 holefix 链（真实板面 RANSAC + 前表面 + ±1° 扇区最小半径 + 圆拟合
+ 2D Kabsch + 重力消歧）重解，得上面这版。四孔重投影（帧 120，去畸变像素）：
FAST-Calib 直出 1.26/2.17 px、旧 holefix 1.25/1.65 px、**新 holefix 1.00/1.62 px** ⇒ 取新 holefix。

新旧两版差 0.63°/33 mm，在 2 m 观测距离处仍不足 1 px，**1.3 m 板在 2 m 下无法分辨**；用 move bag
分别在 5–15 m 场景做边缘距离场评分（Canny + DT，边缘点来自单帧雷达非平面点）：#592 旧 15.78 / 新 16.41 px、
#296 旧 2.98 / 新 3.07 px（均值 5.89 vs 5.72），**远场边缘指标对两版不可分辨**（差值 <1 px，本底 3–16 px）。
结论：取新 `holefix`（标定板残差最优，物理推导自洽），远场若需更严格判别须把板放到 ≥8 m。

注意：`direct_visual_lidar_calibration` 自带子模块（iridescence 等），需
`git submodule update --init --recursive` 递归拉取后才能编译。

## RK3588 实机对比测试（怎么测才算公平）

平台：RK3588S（A76×4 = 核 4-7，A55×4 = 核 0-3），Ubuntu 22.04 + ROS2 Humble，Release
（`-O3 -march=native`）。完整报告见 [speed_test.md](speed_test.md)。

**1. 绑核（必须与实机 launch 一致，否则数字没有可比性）**

| 进程 | 绑定 |
| --- | --- |
| `ros2 bag play`（顶替 Livox 驱动） | `taskset -c 4` |
| `super_lio_node` | `taskset -c 7` + `lio.single_core:=true` |
| `fastlivo_mapping` | `taskset -c 7` + `OMP_NUM_THREADS=1` |
| `visual_a_app` / `visual_b_app` | `taskset -c 7`（**与 LIO 同挤一个大核**；A/B 单体化后视觉支路已在 LIO 同进程内，历史双节点数据见 [speed_test.md](speed_test.md)） |
| `parameter_blackboard`（FL2 相机参数托管） | `taskset -c 0` |

组间留 ≥15 s 间隔降温；每组**至少重复 3 次**（本仓库已实测：裁剪输入下 Super-LIO 本身
不可复现，单跑数字会骗人）。

**2. 四档对照**

```bash
bash /tmp/bench/run1_slio.sh    # ① Super-LIO 单跑（基线）
bash /tmp/bench/run2_fl2.sh     # ② 官方 FAST-LIVO2（+ parameter_blackboard）
bash /tmp/bench/run_ab.sh a     # ③ Super-LIO + 方案A（同核）
bash /tmp/bench/run_ab.sh b     # ④ Super-LIO + 方案B（同核）
python3 /tmp/bench/parse_bench.py slio.log fl2.log vis_a.log vis_b.log <bag时长>
```

**3. 指标口径**（三者必须同口径，否则不可比）

| 指标 | 定义 |
| --- | --- |
| odom 发出延迟 | 雷达消息到达回调（steady 墙钟）→ 该帧 odom publish 完成的墙钟差；含排队与线程调度 |
| 整帧处理时间 | LIO 帧全流程 CPU 耗时：Super-LIO `[FrameProcess]`、FL2 `[LIOTotal]` / `[VIOTotal]`、A/B 视觉 `合计` |
| 波动 | 全体样本 std + P95/max（A/B 视觉是 ~1 s 一条采样，不是全帧） |

**4. 三个必须注意的坑**

- **B 的实时性数字已过期**：[speed_test.md](speed_test.md) 里 B 视觉 41.8 ms、约束
  22.7±26.6 ms 是**修复选轨 O(n²) 之前**测的（当时单帧最坏 422 ms）。修复后选轨 max 已降到
  2.1 ms，B 是否仍"单大核实时临界"必须重测才能下结论。
- **FL2 在单大核上跑不满实时**：处理需求 ≈0.99 帧周期，长序列必然积压，odom 延迟退化到
  秒级（均值 5.3 s、峰值 20 s）。所以"在 RK3588 上测 FL2"要么放开绑核/降帧频，要么明确
  标注"该组为积压态，延迟指标不代表算法耗时"，否则对比不公平。另外其视觉地图原本常驻累积、
  长序列会 OOM，现已在 fork `b9f931f` 用滚动淘汰修掉（详见"关键实验结论"）。
- **不要只看延迟**：A/B 的价值在退化段，健康段应与 LIO 逐点贴合（`|B−LIO|` p50 ≈ 2~5 cm）。
  建议的判据组合是「延迟分布 + 同 bag 逐点差 + 退化段终点误差多跑分布」三者一起看，
  任何单一指标都能给出误导性结论。

## 仓库结构与主要文档

| 路径 | 内容 |
| --- | --- |
| [docs/HANDOVER_2026-11-08.md](docs/HANDOVER_2026-11-08.md) | **工作主线交接文档**：§16 总纲、§17 按帧染色诊断链、§18/§20 官方图像消融、§19 RTK-SLAM 真值评测、§22 滚动淘汰视觉地图点（99.3% 覆盖 / ATE 0.561 m）与三层点云偏差结果 |
| [docs/](docs/) | 专项审计：地图更新机制（MAP_UPDATE_*）、OctVox 审计、去畸变管线、点云质量指标、数据清单（DATA_INVENTORY）等 |
| [tools/](tools/) | 评测与取数脚本：`odom_vs_gt.py`（官方协议 ATE）、`traj_compare.py`、着色一致性/耗时/平面厚度等；RTK-SLAM 参考跑批链路 `run_fl2_rtk.sh` / `run_rtk.sh`（经 `ros_batch_guard.sh` 做互斥与"只杀同批"收尾）+ `fl2_pcd_merge.py`（合并 pcd 分片）+ `rtk_traj_prep.py` / `rtk_pcdev.py`（轨迹导出与点云偏差） |
| [experiments/](experiments/) | 成对消融实验：`fl2_img_ablation`（官方 FAST-LIVO2 有/无图像 ×3 轮）、`octvox_ablation`（地图表示消融；`10~13_*.sh` 为 LIO 核 + 方案A 上色对比跑批） |
| [freqcmd.txt](freqcmd.txt) | 换环境/换机常用命令清单：依赖、构建、主链路跑批、测速方法与已知坑 |
| [speed_test.md](speed_test.md) | RK3588 实机测速报告（Super-LIO / FAST-LIVO2 / 方案 A/B，绑核公平对比） |
| 根目录 `*.png` / `*.html` | 各数据集点云预览图与诊断报告（如 `fastlivo2_bag_yaw_drift_report.html`） |

## 关键实验结论（摘要）

- **NTU eee_03 ATE**：官方 LIVO 0.028 m / 官方纯 LIO 0.043 m；Super-LIO 0.138→0.130 m
  （观测模型对齐后）。剩余差距主因是地图表示（0.25 m 子体素均值压缩）的内存/速度取舍，不建议改。
- **RTK-SLAM Dataset**（全站仪 <1cm 真值，不做 SE(3) 对齐，`construction_seq2` 整轮 16 个 GT 点全覆盖）：
  `super-lio` 0.347 m / `fast-lio-sam` 0.374 m / `super-lio-beam3` 0.508 m / **官方 FAST-LIVO2 0.561 m** /
  `super-lio-beam` 0.656 m。fl2 的 SE(3) 误差 0.091 m 是最低的一档——"形状准、绝对位置不准"；
  其余系统无累积漂移，误差主要是子图刚体偏置而非逐帧抖动。
- **三层点云偏差（`tools/rtk_pcdev.py`）**：以 fl2 的稠密建图为参考面网（39457 面 / 覆盖 26.7%），
  量各系统点的 L1 距离分层散布、L2 跨帧重影、L3 绝对刚体偏差。L1 出斜率（如 fl2 σ0=0.96 cm、
  σθ=0.134°，beam3 σθ=0.282°）；**L3 目前分辨率只到 ~10 cm**（参考面厚度上限 8 cm 主导），
  各系统法向 MAD 7.7–10.6 cm 全落在自比对刻度 9.53 cm 的 0.81–1.11 倍内，**不能据此排序**；
  L2 样本稀疏且非单调，暂不结论。要让 L3 可排序需换更薄的面或改点到最近点的法向距离。
- **官方 FAST-LIVO2 的内存坑已解决（`b9f931f`）**：开图像时原实现会让每个视觉地图点钉住一帧
  完整原图（`FEATURE::img_`），1.92 MB/帧 × 16.5 Hz ≈ 32 MB/s，加上 VisualPoint 自身增长 ≈
  16 MB/s，合计 ≈48 MB/s 无界涨；`img_en=1` 51.8 MB/数据秒 vs `img_en=0` 4.3 MB/数据秒，
  12 倍差全在视觉侧。`pcd_save.interval` 分片只保住已 flush 的地图，**对这部分内存无效**
  （旧轮 46 分片仍在 480 s / 80% 被 OOM）。现在 `pruneVisualMap` 在每次分片落盘后以当前帧时刻
  为 cutoff 删掉已落盘点（连带释放图像）并清空 `warp_map`，当前帧点留到下一轮匹配：本机 23 GB
  可跑满 **595.5/599.5 s = 99.3%**，58 分片对应 58 次 prune，合并 24.88M → 8.92M 点 @5 cm。
  代价是视觉地图只保留最近一片（约 200 帧 / 12 s），**长时视觉重观测能力被放弃**；
  若要做长序列回环需改成按空间距离淘汰。
- **图像消融**（§20）：官方 `img_en=1/0` ×3 轮判决——图像确实参与位姿（厘米级），稳态精度增益≈0，
  有效作用是"锚定/防跑飞"。
- **LiDAR 退化对抗**（NTU eee_03 走廊裁剪，`Y_HALF=1.2` 温和退化，B 侧 8 次重复单跑
  tag=y12fix2~y12fix9）：B 方案（绝对锚定 + 历史锚点保持 + sev 映射 + 守卫）**8/8 次不劣于
  同 bag LIO**，且把终点误差的跨跑散布从 LIO 的 14.1~794.4 m（55×）压到 B 的 8.4~127.8 m
  （15×）——视觉介入不仅降低误差，还压缩了不可复现性本身。幅度只按分布口径给：均值
  71.4 vs 267.7 m（−73%）、中位数 84.9 vs 170.3 m（−50%）、逐次比值 0%~87%（中位 41.5%）；
  **单次数字不可复现，禁止再引用"437→123 m / 72%"这类单跑结论**。选轨耗时已修复
  （视差扫描 O(n²)：`kMaxObservations=16` 截断 + 视差缓存），8 次跑 `>50 ms 帧数 = 0/176~178`。
  A 方案状态回灌链路已通但对位姿无纠正力（同 bag |A−LIO|≤0.7 m），瓶颈在约束质量（走廊段
  大量帧判"健康不更新"/rmse 拒绝，可用重投影约束 n=0~33 太少）。纪律：不自造退化检测，只
  消费 Super-LIO 的 `/lio/degeneracy`；Super-LIO 在裁剪输入下不可复现（同输入终值随机），
  一切对照只看同 bag 内逐点 |VIS−LIO| 或多跑分布。细节见两份方案 README 的退化章节。
- **自采 2026-10-30 bag**：三种跑法（Super-LIO / FAST-LIVO2 纯 LIO / LIVO）均存在约 17° yaw
  漂移重影，源于 IMU 陀螺零偏（gz≈−0.42°/s）+ 回环未纠正，属数据侧问题；诊断见
  [fastlivo2_bag_yaw_drift_report.html](fastlivo2_bag_yaw_drift_report.html)。
- **自采 2026-11 move bag（点云上方漂浮"幽灵层"）**：根因是 **bag 内相机话题断流 2.62 s**
  撞上 `sync_packages` 无降级等待——主线程停摆期间视觉更新用过期关联，生成整片漂浮层。
  曾尝试"VIO 更新位移护栏"（`vio.pos_jump_thresh`），实验证伪：整 bag 0 次触发、幽灵层不变，
  临时代码已全部移除。**结论：数据侧问题，只能重录；别再从算法层修这个 bag。**
- **Retail_Street（HKU 公开数据集）**：必须用官方标定 `camera_pinhole.yaml`（fx≈1293.6）+
  官方 `avia.yaml`（`img_time_offset: 0.1`）；bag 自带 `avia_lvi.yaml`（fx≈906、
  `img_time_offset: 1200.25`）是 DeepRobotics 车载配置，误用会导致 RGB 严重错位。
  标定文件与来源注释已归档在 bag 目录 `/home/ywj/slam_data/Retail_Street/`。

## 待办（后续测试）

- **B2 退化分段验证**：录新长走廊退化 bag，按 §7 方法统计 `[VisRot]` 分段接受率/Δyaw，
  验证 χ² 门控在退化段是否自动开门拉回漂移（配置 B 定版前的最后一步）。
  过程与结论见 [B2B3-光流旋转约束实验记录](.trae/documents/B2B3-光流旋转约束实验记录.md)。
- **彩色点云配准（颜色残差 ICP）**：Colored Point Cloud Registration Revisited,
  Park、Zhou、Koltun，ICCV 2017。待测：在点云配准/观测环节引入颜色残差项。
  前置条件：Mid-360 只有反射强度、无 RGB，需先接上着色链路（见 `pointcloud_colorizer`）；
  颜色只对沿墙面等平移退化方向有补充力，**治不了 yaw 漂移**。

## 致谢

本仓库建立并参考于以下开源工作（各子模块固定提交见上表）：

- [Super-LIO](https://github.com/ypat999/Super-LIO)：LIO 主干（自有 fork，实验改动在 `no-de-check` / `exp/degeneracy-adaptive` 分支）；
- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) 及其 ROS2 移植 [FAST-LIVO2-ROS2](https://github.com/U-AMC/FAST-LIVO2-ROS2)：对照系统；
- [rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open)、[basalt](https://github.com/VladyslavUsenko/basalt)：视觉前端与特征生命周期管理的参考来源；
- [FAST-Calib](https://github.com/xuankuzcr/FAST-Calib) 及其 ROS2 移植 [FAST-Calib-ROS2](https://github.com/MaroSalah02/FAST-Calib-ROS2)：标定板外参（自有 fork 含 CustomMsg CDR 解析与板面法向先验改动）；
- [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)：无目标外参；
- [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration)：标定板板型来源。

## License

本仓库代码以 [MIT License](LICENSE) 开源；`third_party/` 各子模块遵循其上游原始许可证。
