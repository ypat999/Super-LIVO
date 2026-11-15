# A 测试：图像约束写回 Super-LIO 的 ESKF（单体、单核目标）

## Context

现有方案 A（`solution_a_lightweight/super_livo_visual_a`）是**旁路节点**：订阅 `lio/odom` 与
`lio/cloud_world`，用 KLT + LiDAR 深度关联 + 2D-3D 重投影算出 6×6 法方程，再以
「持久偏移 / 状态回灌」的方式去"修"一条已经偏掉的轨迹。这不能满足要求——视觉必须**真正抑制**
LIO 的退化漂移，而不是在偏移数据上持续打补丁；且 topic 交互在实时算法里太低效。

本轮只做 **A 测试**：把 A 的 6 维视觉信息**直接注入 Super-LIO 的 ESKF 观测更新**，合并为单体进程，
不做 B 的 30 维滤波。验收：视觉量能真实改变 ESKF 后验；同一条 bag 上退化段绝对漂移下降。

## 已核实的关键结论（实现须遵守）

1. **符号一致，直接累加**：A 的 `ReprojectionConstraint::accumulate` 产出 `H += w·JᵀJ`、
   `b += w·Jᵀr`（`r = z - π`）；ESKF 的 `Observe` 同为 `HTVH += J w Jᵀ`、`HTVr -= J w error`，
   `UpdateObserve` 里 `dx = Qk·b`。故注入为 `HTVH.block<6,6>(0,0) += H`、`HTVr.head<6>() += b`（**不取负**）。
2. **坐标变换**：A 的 H/b 是相机位姿 `T_wc` 的**世界系**左扰动 `[φ_w; Δp_w]`；ESKF 的旋转误差是
   **机体系右扰动**（`ESKF.cpp` Update：`R_ = R_ * SO3::Exp(dx.head<3>())`），位置误差是世界系。
   转换 `A = blkdiag(R_wb, I₃)`：`H_eskf = Aᵀ H A`，`b_eskf = Aᵀ b`（即只转旋转块与旋转-平移交叉块）。
3. **无需杠杆臂**：`T_wc = T_wb·T_bc`，`T_bc` 固定，对 `T_wc` 的世界左扰动等价于对 `T_wb` 的同一扰动。
   `T_bc = inverse(Rcl/Pcl)`（Rcl/Pcl 是 `T_cam_lidar`，与 A 的 `makeTlcFromRclPcl` 同口径）。
4. **注入点**：`stateProcess()` 中 `Observe()` 之后、`checkPoseTrust()` 之前，单独一次
   `kf_->UpdateObserve(视觉 lambda)`。顺序更新（线性高斯下等价联合更新），好处：
   视觉不被 `truncateDegenerate` 删掉（视觉要接管的正是退化方向），且视觉 dx 可单独观测，不改 `Observe`。
5. **迭代重线性化**：`UpdateObserve` 会多次调用 lambda。深度关联/锚点每帧只做一次（贵），
   但残差与雅可比必须在 lambda 内**按当前迭代位姿**重算（便宜），否则迭代更新退化成一阶。
6. **信息量 cap 必须有**：视觉 H 不能压塌 ESKF 协方差。每帧用先验 `P⁻¹` 与地板值求单标量
   `s = min(1, sqrt(cap_rot/λmax(H_rot)), sqrt(cap_pos/λmax(H_pos)))`
   （`cap_rot = min(κ·λmax(P⁻¹|rot), 1/σ_rot_floor²)`，pos 同理），`H ← s²H, b ← s²b`。
   默认 `κ_healthy=1.0`、`κ_degen=5.0`、`σ_rot_floor=0.005 rad`、`σ_pos_floor=0.02 m`。
7. **A 的 `VisualUpdate::apply` 不用**（内部含 LiDAR 先验正则，会与 ESKF 的 `P⁻¹` 双重计数）。
   只搬 `KltFrontend` / `LidarDepthAssociation` / `ReprojectionConstraint` / `DegeneracyGate`。
8. **时间对齐**：A 实测图像真实采集时刻 `t_capture = header.stamp + img_time_offset`（默认 0.1 s）。
   单体进 ESKF 后，线性化点就是雷达帧末状态（≈ t_capture），无需 deque 插值；
   但**必须按 `t_capture` 挑帧**：只接受 `t_capture ∈ [lidar.start_time, lidar.end_time]` 的最新一帧图像，
   否则会拿差 0.1 s 的位姿去算 2-3 px 的系统偏差。

## 改动清单

### 1. 新增视觉模块（搬运 + 改 namespace）

目录 `third_party/Super-LIO/src/super_lio/{include,src}/visual/`：

- `klt_frontend.hpp/.cpp`、`depth_association.hpp/.cpp`、`reprojection_constraint.hpp/.cpp`、
  `degeneracy_gate.hpp/.cpp`：从 `solution_a_lightweight/super_livo_visual_a` 原样搬运，
  namespace 改 `LI2Sup::vis`，include 路径改 `visual/xxx.hpp`。
- `measurement_noise.hpp`：从 `common/super_livo_common/include/super_livo_common/` 搬入
  （只需 `reprojectionVariance`），或在 `reprojection_constraint.cpp` 内联等价函数。

### 2. 新增 `visual/visual_a_backend.hpp/.cpp`（门面）

```cpp
struct VisualAConfig { /* 内参 fx..cy、width/height、D、Rcl/Pcl、img_time_offset、
                          klt/depth/reproj/gate 全部阈值、cap 的 κ 与地板、enable */ };

class VisualABackend {
 public:
  void configure(const VisualAConfig&);
  // 图像回调线程：逐帧必须调用（KLT 内部维护 prev_gray_）
  bool onImage(const cv::Mat& bgr, double stamp);      // rectify → gray → KLT → 缓存{t_capture, tracked}
  // process 线程：取 [t0,t1] 内最新且未消费的图像帧做深度关联，产出对应关系
  bool prepare(double t0, double t1, const std::vector<Eigen::Vector3d>& sweep_world,
               double degen_sev, const Eigen::Vector3d& degen_dir);
  // lambda 内：按当前 T_wb 重算残差/雅可比 → 6×6 H、6×1 b（世界系左扰动）
  void evaluate(const Eigen::Matrix4d& T_wb, Eigen::Matrix<double,6,6>& H,
                Eigen::Matrix<double,6,1>& b) const;
  // 本帧诊断
  const VisualAStats& stats() const;
};
```

- `prepare` 内部：`local_map_` 累积（`sweep_world` 按 stride 追加 + 上限裁剪）+ 本帧 sweep
  → `LidarDepthAssociation::associate(tracked, T_wc)` → `ReprojectionConstraint::build(...)`
  → 用 `H` 算 `rmse / bias / spread / cond` → `DegeneracyGate::evaluate`。
  只保留 `(uv, xyz_world, variance, huber_weight)` 对应关系供 `evaluate` 迭代重算。
  门控 `visual_weight` 作为整体缩放系数乘进 `H/b`。
- 复用 A 的 `GateInputs` 组装逻辑（`visual_a_node.cpp` L528-600：spread、rmse/bias、
  按先验尺度归一化后的条件数），`lidar_degeneracy` 由 Super-LIO 的退化签名给出。

### 3. 参数（沿用现有 g_ 全局风格）

`include/lio/params.h` + `src/lio/params.cpp` 增加 `g_visual_a_*`（enable、cam_fx/fy/cx/cy/width/height、
cam_d、Rcl、Pcl、img_time_offset、klt.*、depth.*、reproj.*、gate.*、cap.*）；
`ROSWrapper::LoadParamFromRos` 里 `declare_parameter` + `get_parameter`（与现有 `lio.observe.*` 同写法），
前缀统一 `lio.visual.*`。

### 4. SuperLIO 集成

`include/lio/super_lio.h`：
- `#include "visual/visual_a_backend.hpp"`，成员 `std::unique_ptr<vis::VisualABackend> visual_;`
- 公开 `void onVisualImage(const cv::Mat& bgr, double stamp);`
- 私有 `void VisualObserve();` + 诊断成员（`last_vis_n_`、`last_vis_rmse_`、`last_vis_w_`、`last_vis_dx_`）

`src/lio/super_lio.cpp`：
- `init()`：`g_visual_a_enable` 时按配置构造 `visual_`。
- `onVisualImage()`：直接转发 `visual_->onImage(...)`（不持 LIO 锁）。
- `stateProcess()`：在 `Observe()` 之后、`checkPoseTrust()` 之前调用一次 `VisualObserve()`
  （`g_time_eva` 下用 `time_record_.Evaluate` 包一层，新增 `"[VisualObs]"` 计时项）。
- `VisualObserve()`：
  1. `visual_->prepare(measures_.lidar.start_time, measures_.lidar.end_time, points_world_v3_, 退化签名)`；
  2. 取 `cov = kf_->GetCov()`，算 `P⁻¹` 的 rot/pos 块 → 求 cap 与 `s`（退化时用 `κ_degen`）；
  3. `kf_->UpdateObserve([&](const KFState& st, M6& HTVH, V6& HTVr){ ... })`：
     `visual_->evaluate(st.pose.matrix(), H, b)` → `A=blkdiag(R_wb,I)` → `H=AᵀHA*s²·w`、`b=Aᵀb*s²·w`
     → `HTVH.block<6,6>(0,0) += H; HTVr.head<6>() += b;`
  4. 记录视觉单独 dx（对比 `kf_->GetSE3()` 前后差）用于验收日志。
- 1 Hz 日志：`[VisA] n=%d rmse=%.2f spread=%.3f cond=%.0f mode=%d w=%.2f cap=%s(%.0f) dx=%.1fmm/%.3fmrad`。

### 5. ROSWrapper 图像输入

`include/ros/ROSWrapper.h`：加 `rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;`
与 `void imageHandler(const sensor_msgs::msg::Image::SharedPtr msg);`

`src/ros/ROSWrapper.cpp::setupIO()`：`g_visual_a_enable` 时建 `sensor_msgs/Image` 订阅
（`rclcpp::SensorDataQoS()`，独立 `cb_image_` 回调组）；
`imageHandler`：`cv_bridge::toCvShare`（**只在本回调内使用**，转完立刻调用）→
`super_lio_->onVisualImage(bgr, stamp)`。

### 6. 构建

`src/super_lio/CMakeLists.txt`：
- glob 扩到 `src/visual/*.cpp`；
- `find_package(OpenCV REQUIRED)`、`find_package(cv_bridge REQUIRED)`；
- 目标 `ament_target_dependencies(lio ... cv_bridge)` + `target_link_libraries(lio ${OpenCV_LIBS})`；
  include 目录加 `${OpenCV_INCLUDE_DIRS}`。

`src/super_lio/package.xml`：加 `<depend>cv_bridge</depend>`、`<depend>opencv</depend>`。

## 验证

1. `colcon build --packages-select super_lio`（x86 先打通链路）。
2. 同一条 bag 跑两遍：`lio.visual.enable=false`（基线）与 `=true`；固定 `lio.single_core=true`。
3. 逐帧对比（`lio.debug.level>0` 的 CSV + `[VisA]` 日志）：
   - `last_iters_ / last_dx_inf_` 是否变化（证明视觉真的进了后验）；
   - 视觉门控 `w>0` 的帧占比、`n`（有效约束数）、`rmse`；
   - 退化段（`deg_cond_t` 高）的绝对漂移（与 RTK/真值脚本 `tools/rtk_*.py` 对齐后比较）；
   - `VisualObserve` 单独产生的 `dx` 量级（预期：位置 mm~cm、旋转 mrad 级，不应出现米级）。
4. 若视觉完全无影响：先查 `n`（深度关联命中率）与 `w`（门控），再查 cap 是否把 `s` 压到 ~0。
5. 若出现发散/协方差塌陷：把 `cap.kappa_healthy` 降到 0.3、`σ_pos_floor` 提到 0.05 m，重跑。

## 不改动

- `Observe()` 内部逻辑、`truncateDegenerate`、退化签名计算；
- 方案 B（`super_livo_visual_b`）与 A 的独立节点（保留作对照，不删除）。
