#include "super_livo_visual_b/frontend/feature_lifecycle.hpp"

#include <algorithm>

namespace superlivo_b {

namespace {
FeatureLifecycleInfo makeDefaultInfo(int feature_id) {
  FeatureLifecycleInfo info;
  info.feature_id = feature_id;
  return info;
}
}  // namespace

FeatureLifecycle::FeatureLifecycle(const Options& options) : options_(options) {}

FeatureLifecycleInfo FeatureLifecycle::onObserved(int feature_id, double stamp) {
  auto& info = features_[feature_id];
  info.feature_id = feature_id;
  if (info.observations == 0) {
    info.first_stamp = stamp;  // 首次观测
  }
  info.observations += 1;
  info.consecutive_failures = 0;
  info.last_stamp = stamp;

  // 状态推进：新建 -> 跟踪中 -> 成熟。
  if (info.status == FeatureStatus::kNew) {
    info.status = FeatureStatus::kTracked;
  }
  if (info.observations >= options_.mature_observations &&
      info.status != FeatureStatus::kMarginalized) {
    info.status = FeatureStatus::kMature;
  }
  // 轨迹过长则边缘化，避免旧观测污染多帧约束。
  if (info.observations > options_.max_track_length) {
    info.status = FeatureStatus::kMarginalized;
  }
  return info;
}

FeatureLifecycleInfo FeatureLifecycle::onMissed(int feature_id, double stamp) {
  auto it = features_.find(feature_id);
  if (it == features_.end()) {
    return makeDefaultInfo(feature_id);
  }
  auto& info = it->second;
  info.consecutive_failures += 1;
  info.last_stamp = stamp;
  if (info.consecutive_failures > options_.max_failures) {
    info.status = FeatureStatus::kLost;
  }
  return info;
}

std::vector<int> FeatureLifecycle::matureFeatures() const {
  std::vector<int> ids;
  ids.reserve(features_.size());
  for (const auto& kv : features_) {
    if (kv.second.status == FeatureStatus::kMature) {
      ids.push_back(kv.first);
    }
  }
  return ids;
}

void FeatureLifecycle::prune() {
  for (auto it = features_.begin(); it != features_.end();) {
    if (it->second.status == FeatureStatus::kLost ||
        it->second.status == FeatureStatus::kMarginalized) {
      it = features_.erase(it);
    } else {
      ++it;
    }
  }
}

const FeatureLifecycle::Options& FeatureLifecycle::options() const { return options_; }

}  // namespace superlivo_b
