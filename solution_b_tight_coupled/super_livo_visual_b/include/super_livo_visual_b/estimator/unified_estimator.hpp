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

    // 过程噪声（简化模型，单位：rad/√s、m/√s² 等）
    double gyro_noise{1.0e-3};
    double acc_noise{1.0e-2};
    double bias_gyro_noise{1.0e-5};
    double bias_acc_noise{1.0e-4};
    double initial_covariance{1.0};
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

 private:
  /// 信息形式更新：P^-1 <- P^-1 + w·H，dx = -P_new·w·b。
  void applyInformationUpdate(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, double weight);

  Options options_;
  FilterState filter_;
};

}  // namespace superlivo_b
