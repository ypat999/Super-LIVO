#include "super_livo_visual_a/constraint/reprojection_constraint.hpp"

namespace superlivo_a {

ReprojectionConstraint::ReprojectionConstraint(const Options& options) : options_(options) {}

std::vector<VisualResidualBlock> ReprojectionConstraint::build(
    const std::vector<FeatureObs>& /*features*/,
    const std::vector<FeatureDepth>& /*depths*/,
    const Eigen::Matrix4d& /*T_wc*/) const {
  // TODO(阶段 3): 计算 r = z - pi(K * T_CW * P_W)，并给出对 SE3 左扰动的雅可比
  return {};
}

void ReprojectionConstraint::accumulate(const std::vector<VisualResidualBlock>& /*blocks*/,
                                        Eigen::MatrixXd& /*H*/,
                                        Eigen::VectorXd& /*b*/,
                                        double& /*cost*/) const {
  // TODO(阶段 3): Huber 加权汇总 H、b 与代价，交由 VisualUpdate 决策
}

const ReprojectionConstraint::Options& ReprojectionConstraint::options() const { return options_; }

}  // namespace superlivo_a
