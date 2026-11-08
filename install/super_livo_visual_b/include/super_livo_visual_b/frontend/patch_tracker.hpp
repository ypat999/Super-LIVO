#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace superlivo_b {

// ============================================================================
// 方案 B · 视觉前端：图像块跟踪（对应 gpt双方案设计.md §3.2）
// ----------------------------------------------------------------------------
// Basalt 风格前端：图像金字塔 + 图像块（patch）SSD/LSSD 匹配 + 特征生命周期输入。
// 相比方案 A 的 KltFrontend，这里显式维护 patch 与金字塔，输出可持续跟踪的观测。
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

/// Basalt 风格视觉前端：KLT/LSSD 图像块跟踪。
class PatchTracker {
 public:
  struct Options {
    int patch_size{5};
    int pyramid_levels{4};
    int max_features{200};
    double lssd_threshold{0.5};
    double min_distance{8.0};
    double quality_level{0.01};
    bool use_lssd{true};
    int search_radius{4};  ///< 最粗层每层搜索半径（像素）
  };

  explicit PatchTracker(const Options& options);

  /// 处理一帧灰度图，返回本帧跟踪成功的观测。
  std::vector<PatchObservation> process(const cv::Mat& image, double stamp);

  /// 重置跟踪器。
  void reset();

  const Options& options() const;

 private:
  /// 建立图像金字塔。
  static void buildPyramid(const cv::Mat& gray, int levels, std::vector<cv::Mat>& pyramid);

  /// 在给定层的图像上检测新角点。
  std::vector<cv::Point2f> detect(const cv::Mat& gray, std::size_t budget) const;

  Options options_;
  std::vector<cv::Mat> prev_pyramid_;      ///< 上一帧金字塔
  std::vector<PatchObservation> prev_obs_; ///< 上一帧观测
  int next_id_{0};                         ///< 特征 ID 分配器
};

}  // namespace superlivo_b
