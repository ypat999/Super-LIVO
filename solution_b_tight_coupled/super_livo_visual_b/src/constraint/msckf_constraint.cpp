#include "super_livo_visual_b/constraint/msckf_constraint.hpp"

namespace superlivo_b {

MsckfConstraint::MsckfConstraint(const Options& options) : options_(options) {}

bool MsckfConstraint::triangulate(const FeatureTrack& /*track*/, Eigen::Vector3d& /*xyz*/) const {
  // TODO(阶段 B3): 最小二乘三角化，检查视差与重投影误差
  return false;
}

MultiFrameNormalEquations MsckfConstraint::build(const std::vector<FeatureTrack>& /*tracks*/) const {
  // TODO(阶段 B3): 构造多帧重投影 -> 边缘化特征位置 -> 左零空间投影，得到状态空间法方程
  return {};
}

const MsckfConstraint::Options& MsckfConstraint::options() const { return options_; }

}  // namespace superlivo_b
