#pragma once

#include <cstdint>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace colorizer {

// ============================================================================
// RGB 采样与颜色质量维护（对应 gpt双方案设计.md §2.4）
// ----------------------------------------------------------------------------
// gpt 文档强调：把颜色写入累计地图时，应保留点的 RGB 观测质量，避免某一张
// 曝光不佳的图片覆盖此前较好的颜色。因此每个点带 quality，并按质量决定是否覆盖。
// ============================================================================

/// 带颜色的点。
struct ColoredPoint {
  Eigen::Vector3f xyz{Eigen::Vector3f::Zero()};
  Eigen::Vector3f rgb{Eigen::Vector3f::Zero()};  ///< 0~255 的 RGB
  float quality{0.0F};                           ///< 颜色观测质量（0~1）
};

/// RGB 采样与质量维护。
class RgbAssigner {
 public:
  struct Options {
    bool bilinear{true};          ///< true=双线性插值；false=最近邻
    bool keep_best_quality{true}; ///< true=仅当新质量更高才覆盖（保留最佳颜色）
    float min_quality{0.2F};      ///< 低于该质量的颜色观测直接丢弃
  };

  explicit RgbAssigner(const Options& options);

  /// 从图像采样并写入点颜色。
  /// @param bgr     BGR 图像
  /// @param pixel   像素坐标
  /// @param quality 本次观测质量
  /// @param point   输出：写入 rgb 与 quality
  bool assign(const cv::Mat& bgr,
              const Eigen::Vector2d& pixel,
              float quality,
              ColoredPoint& point) const;

  /// 新观测是否应覆盖已有颜色（依据 keep_best_quality 与质量对比）。
  bool shouldOverride(float new_quality, float old_quality) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace colorizer