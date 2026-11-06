#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace colorizer {

/// 带时间戳的图像帧。
struct TimedImage {
  cv::Mat image;
  double exposure_mid_stamp{0.0};  ///< 曝光中点时间，优先使用它做对齐
  double arrival_stamp{0.0};       ///< 驱动/回调到达时间，仅作兜底
  bool valid{false};
};

/// 图像与 LiDAR 扫描窗口的时间对齐。
class ImageSync {
 public:
  struct Options {
    double max_offset{0.02};   ///< 允许的图像-扫描时间差
    bool use_exposure_mid{true};
  };

  explicit ImageSync(const Options& options);

  /// 在图像缓存中挑选与 [sweep_start, sweep_end] 最匹配的一帧。
  TimedImage match(const std::vector<TimedImage>& images,
                   double sweep_start,
                   double sweep_end) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace colorizer
