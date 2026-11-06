#pragma once

#include <vector>

#include "super_livo_visual_b/feature_map/feature_track.hpp"

namespace superlivo_b {

/// 视觉关键帧。
struct VisualKeyframe {
  int id{0};
  double stamp{0.0};
  Eigen::Matrix4d T_wc{Eigen::Matrix4d::Identity()};
  std::vector<int> feature_ids;
};

/// 视觉特征地图 / 关键帧管理。
/// 独立于 OctVox 几何地图，用于保留不依赖 LiDAR 地图的视觉观测能力。
class KeyframeManager {
 public:
  struct Options {
    int max_keyframes{20};
    double min_translation{0.15};
    double min_rotation_deg{5.0};
    int min_tracked_features{40};
  };

  explicit KeyframeManager(const Options& options);

  /// 依据运动与特征数量判断是否新建关键帧。
  bool shouldCreateKeyframe(const Eigen::Matrix4d& T_wc,
                            int tracked_features,
                            double stamp) const;

  /// 加入关键帧并维护滑窗大小。
  void addKeyframe(const VisualKeyframe& keyframe);

  /// 当前滑窗内的关键帧。
  const std::vector<VisualKeyframe>& keyframes() const;

  /// 维护特征轨迹：合并观测、剔除失效轨迹。
  void updateTracks(const std::vector<FeatureTrack>& tracks);

  const Options& options() const;

 private:
  Options options_;
  std::vector<VisualKeyframe> keyframes_;
  // TODO: feature_id -> FeatureTrack 索引
};

}  // namespace superlivo_b
