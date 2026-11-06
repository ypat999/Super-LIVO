#pragma once

#include <vector>

#include <Eigen/Core>

#include "super_livo_visual_a/constraint/depth_association.hpp"
#include "super_livo_visual_a/frontend/klt_frontend.hpp"

namespace superlivo_a {

/// 一个特征块的重投影残差及其对状态（SE3 左扰动）的雅可比。
struct VisualResidualBlock {
  int id{0};
  Eigen::Vector2d residual{Eigen::Vector2d::Zero()};
  Eigen::Matrix<double, 2, 6> jacobian{ Eigen::Matrix<double, 2, 6>::Zero()};
  double weight{1.0};
  bool valid{false};
};

/// 2D-3D 重投影约束构造器。
/// 观测方程：r_i = z_i - pi(K * T_CW * P_W,i)
class ReprojectionConstraint {
 public:
  struct Options {
    double huber_delta{1.0};
    int min_constraints{8};
    double max_pixel_error{8.0};
  };

  explicit ReprojectionConstraint(const Options& options);

  /// 根据特征观测与 LiDAR 深度生成残差块。
  std::vector<VisualResidualBlock> build(const std::vector<FeatureObs>& features,
                                         const std::vector<FeatureDepth>& depths,
                                         const Eigen::Matrix4d& T_wc) const;

  /// 汇总为 ESIKF 可用的法方程 H * dx = b 与鲁棒代价。
  void accumulate(const std::vector<VisualResidualBlock>& blocks,
                  Eigen::MatrixXd& H,
                  Eigen::VectorXd& b,
                  double& cost) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace superlivo_a
