#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace superlivo_a {

/// 单个视觉特征在当前帧的观测。
struct FeatureObs {
  int id{0};
  float u{0.0F};
  float v{0.0F};
  int level{0};
  double stamp{0.0};
};

/// 前端输出：本帧跟踪到的特征与质量统计。
struct TrackResult {
  std::vector<FeatureObs> tracked;
  int detected{0};
  int rejected_fb{0};
  cv::Mat gray;  ///< 去畸变灰度图，供后续约束模块复用
};

/// 轻量视觉前端：灰度金字塔 + FAST/Shi-Tomasi + PyrLK + 前后向一致性。
/// 只负责产生特征轨迹与跟踪质量，不参与状态更新、不持有位姿。
class KltFrontend {
 public:
  struct Options {
    int max_features{150};
    int pyramid_levels{3};
    double min_distance{10.0};
    double fb_error_threshold{1.0};
    int min_track_length{3};
    bool use_fast{true};
  };

  explicit KltFrontend(const Options& options);

  /// 处理一帧图像：检测新特征并对上一帧特征做 PyrLK 跟踪。
  TrackResult process(const cv::Mat& gray, double stamp);

  /// 清空内部特征缓存（重置/重定位时调用）。
  void reset();

  const Options& options() const;

 private:
  Options options_;
  // TODO: 特征缓存、上一帧图像、金字塔、ID 分配器
};

}  // namespace superlivo_a
