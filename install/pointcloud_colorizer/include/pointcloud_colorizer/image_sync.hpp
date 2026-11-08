#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace colorizer {

// ============================================================================
// 图像与扫描窗口时间对齐（对应 gpt双方案设计.md §2.4）
// ----------------------------------------------------------------------------
// gpt 文档强调：相机全局快门应使用准确的曝光时间参考，最好是"曝光中点"，
// 而不是图像到达驱动程序的时刻。因此 TimedImage 同时保留 exposure_mid_stamp
// 与 arrival_stamp，前者优先用于对齐，后者仅作兜底。
// ============================================================================

/// 带时间戳的图像帧。
struct TimedImage {
  cv::Mat image;
  double exposure_mid_stamp{0.0};  ///< 曝光中点时间，优先用于对齐
  double arrival_stamp{0.0};       ///< 驱动/回调到达时间，仅作兜底
  bool valid{false};
};

/// 图像与 LiDAR 扫描窗口的时间对齐。
class ImageSync {
 public:
  struct Options {
    double max_offset{0.02};        ///< 允许的图像-扫描时间差（秒）
    bool use_exposure_mid{true};    ///< true=优先曝光中点；false=退回到达时间
  };

  explicit ImageSync(const Options& options);

  /// 在图像缓存中挑选与 [sweep_start, sweep_end] 最匹配的一帧。
  /// @return 未找到满足 max_offset 的帧时返回 valid=false
  TimedImage match(const std::vector<TimedImage>& images,
                   double sweep_start,
                   double sweep_end) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace colorizer