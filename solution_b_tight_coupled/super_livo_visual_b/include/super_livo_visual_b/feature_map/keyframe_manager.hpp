#pragma once

#include <unordered_map>
#include <vector>

#include "super_livo_visual_b/feature_map/feature_track.hpp"

namespace superlivo_b {

// ============================================================================
// 方案 B · 视觉关键帧与特征地图管理（对应 gpt双方案设计.md §3.3 / §四）
// ----------------------------------------------------------------------------
// 独立于 OctVox 的视觉特征地图：维护关键帧滑窗与特征轨迹索引，
// 为 MSCKF 多帧约束提供数据，同时保留"不依赖 LiDAR 全局地图"的观测能力。
// ============================================================================

/// 视觉关键帧。
struct VisualKeyframe {
  int id{0};
  double stamp{0.0};
  Eigen::Matrix4d T_wc{Eigen::Matrix4d::Identity()};
  std::vector<int> feature_ids;
};

/// 视觉特征地图 / 关键帧管理。
class KeyframeManager {
 public:
  struct Options {
    int max_keyframes{20};
    double min_translation{0.15};
    double min_rotation_deg{5.0};
    int min_tracked_features{40};
  };

  explicit KeyframeManager(const Options& options);

  /// 依据与上一关键帧的平移/旋转、跟踪特征数判断是否新建关键帧。
  bool shouldCreateKeyframe(const Eigen::Matrix4d& T_wc,
                            int tracked_features,
                            double stamp) const;

  /// 加入关键帧并裁剪超出滑窗的最旧帧。
  void addKeyframe(const VisualKeyframe& keyframe);

  /// 当前滑窗内的关键帧。
  const std::vector<VisualKeyframe>& keyframes() const;

  /// 维护特征轨迹：合并新观测、剔除失效轨迹、更新索引。
  void updateTracks(const std::vector<FeatureTrack>& tracks);

  /// 取当前可参与多帧约束的轨迹（观测数足够、视差足够）。
  std::vector<FeatureTrack> constraintTracks(int min_observations, double min_parallax_deg) const;

  /// 轨迹池规模（耗时排查用）：条数、观测总数、单条最长观测数、观测对总数上限。
  struct TrackPoolStats {
    std::size_t num_tracks{0};
    std::size_t total_obs{0};
    std::size_t max_obs{0};
    std::size_t pair_work{0};  ///< Σ n(n-1)/2，视差扫描的配对数上限
  };
  TrackPoolStats trackPoolStats() const;

  /// 清空地图。
  void clear();

  const Options& options() const;

 private:
  Options options_;
  std::vector<VisualKeyframe> keyframes_;
  std::unordered_map<int, FeatureTrack> tracks_;  ///< feature_id -> 轨迹
};

}  // namespace superlivo_b
