#pragma once

#include <cstdint>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace colorizer {

/// 带颜色的点。
struct ColoredPoint {
  Eigen::Vector3f xyz{Eigen::Vector3f::Zero()};
  Eigen::Vector3f rgb{Eigen::Vector3f::Zero()};  ///< 0~255
  float quality{0.0F};                            ///< 颜色观测质量
};

/// RGB 采样与质量维护。
/// 写入累计地图时应保留颜色观测质量，避免劣质帧覆盖此前较好的颜色。
class RgbAssigner {
 public:
  struct Options {
    bool bilinear{true};
    bool keep_best_quality{true};
    float min_quality{0.2F};
  };

  explicit RgbAssigner(const Options& options);

  /// 从图像采样并写入点颜色。
  bool assign(const cv::Mat& bgr,
              const Eigen::Vector2d& pixel,
              float quality,
              ColoredPoint& point) const;

  /// 新观测是否应覆盖已有颜色。
  bool shouldOverride(float new_quality, float old_quality) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace colorizer
