#include "super_livo_visual_b/estimator/unified_estimator.hpp"

namespace superlivo_b {

UnifiedEstimator::UnifiedEstimator(const Options& options) : options_(options) {}

void UnifiedEstimator::propagate(const Eigen::Vector3d& /*acc*/,
                                 const Eigen::Vector3d& /*gyro*/,
                                 double /*dt*/) {
  // TODO(阶段 B1): 复用 Super-LIO ESIKF 的 IMU 传播，扩展状态维度的协方差传播
}

void UnifiedEstimator::lidarUpdate(const Eigen::MatrixXd& /*H*/,
                                   const Eigen::VectorXd& /*b*/,
                                   double /*cost*/) {
  // TODO(阶段 B1): 复用 Super-LIO 的 LiDAR 更新，保持与视觉更新同一套状态与协方差
}

void UnifiedEstimator::visualUpdate(const VisualUpdateInput& /*input*/) {
  // TODO(阶段 B3): 视觉法方程 -> 统一 ESIKF 更新，输出带协方差与质量评估的修正
}

void UnifiedEstimator::injectError(const Eigen::VectorXd& /*dx*/) {
  // TODO(阶段 B1): 误差状态注入，更新 filter_.state 与 filter_.covariance
}

const FilterState& UnifiedEstimator::filterState() const { return filter_; }

const UnifiedEstimator::Options& UnifiedEstimator::options() const { return options_; }

}  // namespace superlivo_b
