#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace superlivo_b {

// ============================================================================
// 方案 B · 视觉前端：图像块跟踪（对应 gpt双方案设计.md §3.2）
// ----------------------------------------------------------------------------
// Basalt 风格前端：金字塔 LK 跟踪上一帧的 patch + 角点补充 + 特征生命周期输入。
// 早期实现的"单层 5x5 SSD 模板匹配"已废弃（实测在本数据集的大帧间位移下会锁到
// 错误极值，见 patch_tracker.cpp 里的说明）。
// ============================================================================

/// 图像块跟踪的一个观测。
struct PatchObservation {
  int feature_id{0};
  float u{0.0F};
  float v{0.0F};
  double stamp{0.0};
  int level{0};
  float score{0.0F};  ///< 归一化相关得分（1 = 完全匹配）
};

/// Basalt 风格视觉前端：金字塔 LK 图像块跟踪。
class PatchTracker {
 public:
  struct Options {
    int patch_size{5};             ///< 匹配得分取样块边长（奇数）
    int pyramid_levels{4};
    int max_features{200};
    double lssd_threshold{0.5};
    double min_distance{8.0};
    double quality_level{0.01};
    bool use_lssd{true};
    int search_radius{4};          ///< LK 窗口半边长（像素）：窗 = 2r+1；默认 4 => 9x9
  };

  explicit PatchTracker(const Options& options);

  /// 处理一帧灰度图，返回本帧跟踪成功的观测。
  std::vector<PatchObservation> process(const cv::Mat& image, double stamp);

  /// 重置跟踪器。
  void reset();

  const Options& options() const;

 private:
  /// 在给定层的图像上检测新角点。
  std::vector<cv::Point2f> detect(const cv::Mat& gray, std::size_t budget) const;

  Options options_;
  cv::Mat prev_gray_;                      ///< 上一帧灰度图（LK 金字塔内部自建）
  std::vector<PatchObservation> prev_obs_; ///< 上一帧观测
  int next_id_{0};                         ///< 特征 ID 分配器
};

}  // namespace superlivo_b
