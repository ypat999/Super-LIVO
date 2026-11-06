#pragma once

#include <Eigen/Core>

#include "super_livo_visual_b/state/state_layout.hpp"

namespace superlivo_b {

/// 视觉测量更新输入。
struct VisualUpdateInput {
  Eigen::MatrixXd H;
  Eigen::VectorXd b;
  double cost{0.0};
  double quality{0.0};
  int used_features{0};
};

/// 统一状态估计器：IMU 传播 + LiDAR 更新 + 视觉更新，共用一套状态与协方差。
/// 不引入第二套独立 EKF，Super-LIO 的 ESIKF 仍是唯一滤波器。
class UnifiedEstimator {
 public:
  struct Options {
    bool enable_visual{false};
    double visual_weight{1.0};
    bool estimate_extrinsics{false};
  };

  explicit UnifiedEstimator(const Options& options);

  /// IMU 传播。
  void propagate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt);

  /// LiDAR 更新（点到面/几何匹配），由 Super-LIO 原有模块提供残差。
  void lidarUpdate(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, double cost);

  /// 视觉更新（2D-3D 与多帧约束统一注入）。
  void visualUpdate(const VisualUpdateInput& input);

  /// 误差状态注入：把增量写回状态并更新协方差。
  void injectError(const Eigen::VectorXd& dx);

  /// 当前状态与协方差。
  const FilterState& filterState() const;

  const Options& options() const;

 private:
  Options options_;
  FilterState filter_;
};

}  // namespace superlivo_b
