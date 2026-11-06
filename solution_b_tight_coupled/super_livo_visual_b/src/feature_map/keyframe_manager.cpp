#include "super_livo_visual_b/feature_map/keyframe_manager.hpp"

namespace superlivo_b {

KeyframeManager::KeyframeManager(const Options& options) : options_(options) {}

bool KeyframeManager::shouldCreateKeyframe(const Eigen::Matrix4d& /*T_wc*/,
                                           int /*tracked_features*/,
                                           double /*stamp*/) const {
  // TODO(阶段 B2): 依据与上一关键帧的平移/旋转、跟踪特征数与时间间隔判断
  return false;
}

void KeyframeManager::addKeyframe(const VisualKeyframe& /*keyframe*/) {
  // TODO(阶段 B3): 插入并裁剪超出滑窗窗口的关键帧
}

const std::vector<VisualKeyframe>& KeyframeManager::keyframes() const { return keyframes_; }

void KeyframeManager::updateTracks(const std::vector<FeatureTrack>& /*tracks*/) {
  // TODO(阶段 B3): 合并新观测、剔除失效轨迹、维护特征索引
}

const KeyframeManager::Options& KeyframeManager::options() const { return options_; }

}  // namespace superlivo_b
