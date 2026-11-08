#pragma once

#include <Eigen/Core>

#include "super_livo_visual_b/state/state_layout.hpp"

namespace superlivo_b {

// ============================================================================
// 方案 B · 统一状态估计器（对应 gpt双方案设计.md §3.4）
// ----------------------------------------------------------------------------
// 仍以 Super-LIO 的 ESIKF 为中心，不引入第二套独立 EKF/滑窗优化器。
// 三类更新共用同一套状态与协方差：
//   IMU 传播 -> LiDAR 更新 -> 视觉更新
// 视觉/LiDAR 残差以"信息形式"(H = J^T J, b = J^T r) 注入，采用信息滤波更新：
//   P_total = (P^-1 + w·H)^-1,  dx = -P_total · w·b
// ============================================================================

/// 视觉测量更新输入。
struct VisualUpdateInput {
  Eigen::MatrixXd H;  ///< 6x6 相机位姿信息矩阵（内部映射到 30 维状态）
  Eigen::VectorXd b;  ///< 6x1
  double cost{0.0};
  double quality{0.0};  ///< 视觉质量评估（0~1）
  int used_features{0};
};

/// 统一状态估计器。
class UnifiedEstimator {
 public:
  struct Options {
    bool enable_visual{false};
    double visual_weight{1.0};
    bool estimate_extrinsics{false};

    // ---- 视觉注入强度上限 -------------------------------------------------
    // 即使残差方差模型建对了，单帧视觉也不该比 LiDAR 先验更自信：
    // 地图点/关键帧位姿的误差是系统性的（LIO 漂移），不会随特征数按 √N 平均掉。
    // 取两个上限的**较小者**：
    //   ① 相对上限 κ：视觉信息量不超过 κ·λmax(P⁻¹|该块)；
    //   ② 绝对下限 floor：视觉隐含的位姿精度不得优于 σ_floor。
    // 只保留 ① 会被"自指"击穿——P 一旦被压小，λmax(P⁻¹) 随之变大、cap 也跟着变大，
    // 协方差沿正反馈一路压到 1 cm 以下（实测 60 s 漂到 149 m）。
    double visual_info_ratio{1.0};
    double visual_rot_sigma_floor{0.005};  ///< 视觉隐含姿态精度下限（rad，≈0.3°）
    double visual_pos_sigma_floor{0.02};   ///< 视觉隐含位置精度下限（m）
    // 视觉更新是否只允许修改 rot/pos（屏蔽速度/零偏的协方差耦合项）。
    // 必须为 true：视觉观测本质是"相对地图的位姿"，对速度没有独立信息，
    // dx 的速度分量全部来自 P 的耦合项；即便每帧限幅到 0.05 m/s，600 帧也能
    // 累积 30 m/s，再经 Φ(pos←vel) 放大成百米级漂移（实测 149 m 即此模式）。
    // 速度/零偏只应由 IMU 传播与 LiDAR 先验修正。
    bool visual_pose_only{true};

    // 过程噪声（简化模型，单位：rad/√s、m/√s² 等）
    double gyro_noise{1.0e-3};
    double acc_noise{1.0e-2};
    double bias_gyro_noise{1.0e-5};
    double bias_acc_noise{1.0e-4};
    double initial_covariance{1.0};
  };

  /// 最近一次视觉更新的诊断量（供节点日志判断修正量是否合理）。
  struct VisualDiagnostics {
    double info_lambda{0.0};   ///< 缩放前 λmax(H_vis)
    double cap_lambda{0.0};    ///< 允许的上限 κ·λmax(P⁻¹|pose)
    double scale{1.0};         ///< 实际施加的缩放系数
    double dx_rot{0.0};        ///< 本帧注入的旋转修正（rad）
    double dx_pos{0.0};        ///< 本帧注入的平移修正（m）
    double sigma_pos{0.0};     ///< 更新后位置协方差开方（m）
    double sigma_rot{0.0};     ///< 更新后旋转协方差开方（rad）
    double vel_norm{0.0};      ///< 当前速度估计（m/s），用于诊断发散
  };

  explicit UnifiedEstimator(const Options& options);

  /// IMU 传播（复用 Super-LIO 的传播思路，状态维度扩展到 kSize）。
  void propagate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt);

  /// LiDAR 更新：残差由 Super-LIO 原有模块以信息形式提供（30 维）。
  void lidarUpdate(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, double cost);

  /// 视觉更新：6 维相机位姿法方程，内部映射到统一状态。
  void visualUpdate(const VisualUpdateInput& input);

  /// 误差状态注入：把增量写回状态（协方差已在更新中处理）。
  void injectError(const Eigen::VectorXd& dx);

  /// 用外部先验（如首帧 lio/odom 的 T_wl）初始化状态与协方差。
  /// 节点层在收到第一帧里程计后调用，避免滤波器从单位阵起步而长期"追不上"。
  void resetState(const State& state);

  /// 当前状态与协方差。
  const FilterState& filterState() const;

  const Options& options() const;

  /// 最近一次视觉更新的诊断量。
  const VisualDiagnostics& visualDiagnostics() const { return vis_diag_; }

 private:
  /// 信息形式更新：P^-1 <- P^-1 + w·H，dx = -P_new·w·b。
  /// max_* > 0 时按各块上限对 dx 做整体等比限幅（发散防线）。
  void applyInformationUpdate(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, double weight,
                              double max_rot = 0.0, double max_pos = 0.0,
                              double max_vel = 0.0, double max_bg = 0.0, double max_ba = 0.0,
                              double max_grav = 0.0, bool pose_only = false,
                              VisualDiagnostics* diag = nullptr);

  /// 视觉更新单帧步长上限：0.5°、2 cm、5 cm/s、1e-4 rad/s、1e-3 m/s²、5e-3 m/s²(重力)。
  static constexpr double kMaxVisualRotStep = 0.0087;
  static constexpr double kMaxVisualPosStep = 0.02;
  static constexpr double kMaxVisualVelStep = 0.05;
  static constexpr double kMaxVisualBgStep = 1.0e-4;
  static constexpr double kMaxVisualBaStep = 1.0e-3;
  static constexpr double kMaxVisualGravStep = 5.0e-3;

  Options options_;
  FilterState filter_;
  VisualDiagnostics vis_diag_;
};

}  // namespace superlivo_b
