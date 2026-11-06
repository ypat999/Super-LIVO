#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "super_livo_visual_b/feature_map/feature_track.hpp"

namespace superlivo_b {

/// 光度残差块（可选能力）。
struct PhotometricResidual {
  int feature_id{0};
  double residual{0.0};
  Eigen::Matrix<double, 1, 6> jacobian{Eigen::Matrix<double, 1, 6>::Zero()};
  double weight{1.0};
  bool valid{false};
};

/// 第三种视觉约束（可选）：FAST-LIVO2 风格 Patch 光度残差。
/// 当 KLT 几何约束不足时启用，利用纹理信息补充约束。
class PhotometricConstraint {
 public:
  struct Options {
    int patch_size{5};
    int pyramid_levels{3};
    bool enable{false};
    double huber_delta{1.0};
  };

  explicit PhotometricConstraint(const Options& options);

  std::vector<PhotometricResidual> build(const cv::Mat& image,
                                         const std::vector<FeatureTrack>& tracks) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace superlivo_b
