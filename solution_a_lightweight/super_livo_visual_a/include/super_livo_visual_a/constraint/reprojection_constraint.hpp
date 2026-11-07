#pragma once

#include <vector>

#include <Eigen/Core>

#include "super_livo_visual_a/constraint/depth_association.hpp"
#include "super_livo_visual_a/frontend/klt_frontend.hpp"

namespace superlivo_a {

// ============================================================================
// 方案 A · 2D-3D 重投影约束（对应 gpt双方案设计.md §2.2）
// ----------------------------------------------------------------------------
// 观测方程：r_i = z_i - pi(K * T_CW * P_W,i)
// 雅可比基于相机位姿的左扰动（se3，6 维），因此 H/b 是 6x6 / 6x1，
// 由宿主 ESIKF 注入到状态的旋转/位置块。
// ============================================================================

/// 一个特征块的重投影残差及其对相机位姿（左扰动）的雅可比。
struct VisualResidualBlock {
  int id{0};
  Eigen::Vector2d residual{Eigen::Vector2d::Zero()};                          ///< r = z - pi(...)
  Eigen::Matrix<double, 2, 6> jacobian{Eigen::Matrix<double, 2, 6>::Zero()};  ///< dr/d(se3)
  double weight{1.0};                                                          ///< Huber 权重
  bool valid{false};
};

/// 2D-3D 重投影约束构造器。
class ReprojectionConstraint {
 public:
  struct Options {
    double fx{0.0};  ///< 相机内参（必须设置）
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double huber_delta{1.0};      ///< Huber 核阈值（像素）
    int min_constraints{8};       ///< 最小有效约束数
    double max_pixel_error{8.0};  ///< 外点阈值（像素）
  };

  explicit ReprojectionConstraint(const Options& options);

  /// 根据特征观测与 LiDAR 深度生成残差块。
  std::vector<VisualResidualBlock> build(const std::vector<FeatureObs>& features,
                                         const std::vector<FeatureDepth>& depths,
                                         const Eigen::Matrix4d& T_wc) const;

  /// 汇总为 6 维法方程 H * dx = b 与鲁棒代价。
  void accumulate(const std::vector<VisualResidualBlock>& blocks,
                  Eigen::MatrixXd& H,
                  Eigen::VectorXd& b,
                  double& cost) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace superlivo_a
