#include "super_livo_visual_b/constraint/lidar_visual_constraint.hpp"

namespace superlivo_b {

LidarVisualConstraint::LidarVisualConstraint(const Options& options) : options_(options) {}

std::vector<VisualResidual> LidarVisualConstraint::build(const std::vector<FeatureTrack>& /*tracks*/,
                                                         const Eigen::Matrix4d& /*T_wc*/) const {
  // TODO(阶段 B3): 从 OctVox 取三维锚点，投影到图像计算 2D-3D 残差与雅可比
  return {};
}

void LidarVisualConstraint::accumulate(const std::vector<VisualResidual>& /*residuals*/,
                                       Eigen::MatrixXd& /*H*/,
                                       Eigen::VectorXd& /*b*/,
                                       double& /*cost*/) const {
  // TODO(阶段 B3): Huber 加权汇总为统一状态空间下的法方程
}

const LidarVisualConstraint::Options& LidarVisualConstraint::options() const { return options_; }

}  // namespace superlivo_b
