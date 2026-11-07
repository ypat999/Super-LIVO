#include "super_livo_visual_b/feature_map/keyframe_manager.hpp"

#include <algorithm>
#include <cmath>

// Matrix4d::inverse() 需要 LU 模块的 Inverse 求值特化，否则运行期无限递归。
#include <Eigen/Geometry>
#include <Eigen/LU>

namespace superlivo_b {

namespace {
constexpr double kPi = 3.14159265358979323846;

/// 从位姿矩阵提取相对平移与旋转角（度）。
void relativeMotion(const Eigen::Matrix4d& from, const Eigen::Matrix4d& to, double& translation,
                    double& rotation_deg) {
  const Eigen::Matrix4d delta = from.inverse() * to;
  translation = delta.block<3, 1>(0, 3).norm();
  const Eigen::Matrix3d R = delta.block<3, 3>(0, 0);
  const double cos_angle = std::max(-1.0, std::min(1.0, (R.trace() - 1.0) * 0.5));
  rotation_deg = std::acos(cos_angle) * 180.0 / kPi;
}
}  // namespace

KeyframeManager::KeyframeManager(const Options& options) : options_(options) {}

bool KeyframeManager::shouldCreateKeyframe(const Eigen::Matrix4d& T_wc, int tracked_features,
                                           double /*stamp*/) const {
  if (tracked_features < options_.min_tracked_features) {
    return false;
  }
  if (keyframes_.empty()) {
    return true;  // 第一帧直接作为关键帧
  }

  double translation = 0.0;
  double rotation_deg = 0.0;
  relativeMotion(keyframes_.back().T_wc, T_wc, translation, rotation_deg);

  return translation >= options_.min_translation || rotation_deg >= options_.min_rotation_deg;
}

void KeyframeManager::addKeyframe(const VisualKeyframe& keyframe) {
  keyframes_.push_back(keyframe);

  // 滑窗裁剪：移除最旧关键帧，同时剔除只被它观测到的特征轨迹。
  while (static_cast<int>(keyframes_.size()) > options_.max_keyframes) {
    const VisualKeyframe removed = keyframes_.front();
    keyframes_.erase(keyframes_.begin());
    for (int feature_id : removed.feature_ids) {
      tracks_.erase(feature_id);
    }
  }
}

const std::vector<VisualKeyframe>& KeyframeManager::keyframes() const { return keyframes_; }

void KeyframeManager::updateTracks(const std::vector<FeatureTrack>& tracks) {
  double latest_stamp = 0.0;
  for (const auto& incoming : tracks) {
    latest_stamp = std::max(latest_stamp, incoming.observations.empty()
                                               ? 0.0
                                               : incoming.observations.back().stamp);
    auto& existing = tracks_[incoming.id];
    existing.id = incoming.id;

    // 合并观测：按 stamp 去重后追加。
    for (const auto& obs : incoming.observations) {
      const bool duplicated =
          std::any_of(existing.observations.begin(), existing.observations.end(),
                      [&obs](const FeatureObservation& a) { return a.stamp == obs.stamp; });
      if (!duplicated) {
        existing.addObservation(obs);
      }
    }

    // 更新三角化结果与质量。
    if (incoming.triangulated) {
      existing.triangulated = true;
      existing.xyz_world = incoming.xyz_world;
    }
    existing.quality = incoming.quality;
    existing.valid = existing.observations.size() >= 2;
  }

  // 剔除"长时间未再被观测"的轨迹。
  // 注意：不能用 observations.size()<2 作为剔除条件——单帧新特征会在同一帧被删掉，
  // 导致轨迹永远无法累积到多帧（MSCKF 就永远拿不到约束）。这里改为按时间老化。
  constexpr double kTrackMaxAgeSeconds = 1.0;  // 10Hz 下约 10 帧
  if (latest_stamp <= 0.0) {
    return;  // 本帧没有有效观测，保持现状
  }
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    const FeatureTrack& track = it->second;
    if (track.observations.empty() ||
        latest_stamp - track.observations.back().stamp > kTrackMaxAgeSeconds) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<FeatureTrack> KeyframeManager::constraintTracks(int min_observations,
                                                            double min_parallax_deg) const {
  std::vector<FeatureTrack> result;
  result.reserve(tracks_.size());
  for (const auto& kv : tracks_) {
    if (kv.second.readyForConstraint(min_observations, min_parallax_deg)) {
      result.push_back(kv.second);
    }
  }
  return result;
}

void KeyframeManager::clear() {
  keyframes_.clear();
  tracks_.clear();
}

const KeyframeManager::Options& KeyframeManager::options() const { return options_; }

}  // namespace superlivo_b
