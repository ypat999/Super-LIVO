#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace superlivo_b {

/// 图像块跟踪的一个观测。
struct PatchObservation {
  int feature_id{0};
  float u{0.0F};
  float v{0.0F};
  double stamp{0.0};
  int level{0};
  float score{0.0F};
};

/// Basalt 风格视觉前端：KLT/LSSD 图像块跟踪。
/// 相比方案 A，增加了图像块描述与多帧特征生命周期管理，输出可持续的轨迹。
class PatchTracker {
 public:
  struct Options {
    int patch_size{5};
    int pyramid_levels{4};
    int max_features{200};
    double lssd_threshold{0.5};
    double min_distance{8.0};
    bool use_lssd{true};
  };

  explicit PatchTracker(const Options& options);

  /// 处理一帧灰度图，返回本帧跟踪成功的观测。
  std::vector<PatchObservation> process(const cv::Mat& gray, double stamp);

  /// 重置跟踪器（重定位或长时间丢失后调用）。
  void reset();

  const Options& options() const;

 private:
  Options options_;
  // TODO: 图像金字塔、图像块缓存、上一帧观测
};

}  // namespace superlivo_b
