#pragma once

#include <vector>

#include <Eigen/Core>

#include "super_livo_visual_b/feature_map/feature_track.hpp"

namespace superlivo_b {

/// 多帧约束汇聚结果（法方程形式）。
struct MultiFrameNormalEquations {
  Eigen::MatrixXd H;
  Eigen::VectorXd b;
  double cost{0.0};
  int used_features{0};
  bool valid{false};
};

/// 第二种视觉约束：MSCKF 风格多帧视觉轨迹约束。
/// 将特征位置作为临时变量消去，只更新滤波器状态；
/// 即使 OctVox 地图已扭曲，也能从连续图像的视差中得到独立运动信息。
class MsckfConstraint {
 public:
  struct Options {
    int min_observations{4};
    int max_track_length{20};
    double min_parallax_deg{1.0};
    double huber_delta{1.0};
    bool use_nullspace_projection{true};
  };

  explicit MsckfConstraint(const Options& options);

  /// 三角化特征位置（相机系/世界系按实现约定）。
  bool triangulate(const FeatureTrack& track, Eigen::Vector3d& xyz) const;

  /// 构造多帧重投影约束，并投影到状态空间的左零空间。
  MultiFrameNormalEquations build(const std::vector<FeatureTrack>& tracks) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace superlivo_b
