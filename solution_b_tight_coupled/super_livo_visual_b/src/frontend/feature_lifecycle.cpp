#include "super_livo_visual_b/frontend/feature_lifecycle.hpp"

namespace superlivo_b {

FeatureLifecycle::FeatureLifecycle(const Options& options) : options_(options) {}

FeatureLifecycleInfo FeatureLifecycle::onObserved(int /*feature_id*/, double /*stamp*/) {
  // TODO(阶段 B2): 更新观测次数/时间戳，达到阈值后置为 kMature
  return {};
}

FeatureLifecycleInfo FeatureLifecycle::onMissed(int /*feature_id*/, double /*stamp*/) {
  // TODO(阶段 B2): 累计失败次数，超限置为 kLost
  return {};
}

std::vector<int> FeatureLifecycle::matureFeatures() const {
  // TODO(阶段 B2): 返回处于 kMature 且未边缘化的特征
  return {};
}

void FeatureLifecycle::prune() {
  // TODO(阶段 B2): 清理 kLost / kMarginalized 记录
}

const FeatureLifecycle::Options& FeatureLifecycle::options() const { return options_; }

}  // namespace superlivo_b
