#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "super_livo_visual_b/feature_map/feature_track.hpp"

namespace superlivo_b {

// ============================================================================
// 方案 B · 第三种视觉约束（可选）：Patch 光度残差（对应 gpt §3.2 / §五）
// ----------------------------------------------------------------------------
// 当 KLT 几何约束不足时，用图像纹理补充约束（FAST-LIVO2 风格）。
// 残差定义：以轨迹最后一次观测的位姿为线性化点，把三维锚点投影得到预测像素，
// 取预测像素与实际观测像素处的灰度差作为光度残差。
// ============================================================================

/// 光度残差块。
struct PhotometricResidual {
  int feature_id{0};
  double residual{0.0};
  Eigen::Matrix<double, 1, 6> jacobian{Eigen::Matrix<double, 1, 6>::Zero()};
  double weight{1.0};
  bool valid{false};
};

/// FAST-LIVO2 风格 Patch 光度残差约束。
class PhotometricConstraint {
 public:
  struct Options {
    double fx{0.0};  ///< 相机内参（必须设置）
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    int patch_size{5};
    int pyramid_levels{3};
    bool enable{false};
    double huber_delta{1.0};
    double max_intensity_error{40.0};  ///< 灰度差外点阈值（0~255）
  };

  explicit PhotometricConstraint(const Options& options);

  std::vector<PhotometricResidual> build(const cv::Mat& image,
                                         const std::vector<FeatureTrack>& tracks) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace superlivo_b
