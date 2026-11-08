#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

namespace superlivo_a {

// ============================================================================
// 方案 A · 轻量视觉前端（对应 gpt双方案设计.md §2.2）
// ----------------------------------------------------------------------------
// 流程：灰度化/金字塔 -> FAST 或 Shi-Tomasi 检测 -> PyrLK 跟踪 ->
//       前后向一致性检查 -> 短期轨迹缓存（最近 trace_window 帧）。
//
// 设计要点：
//   - 视觉不恢复尺度，深度由 LiDAR 提供（见 depth_association.hpp）。
//   - 短期轨迹用于 LiDAR 退化时的多帧交叉验证，不依赖全局地图。
// ============================================================================

/// 单个视觉特征在当前帧的观测。
struct FeatureObs {
  int id{0};          ///< 跨帧稳定 ID
  float u{0.0F};      ///< 像素坐标 u
  float v{0.0F};      ///< 像素坐标 v
  int level{0};       ///< 金字塔层
  double stamp{0.0};  ///< 观测时刻
};

/// 方案 A 的短期特征轨迹（gpt §2.2 "保留最近 3~5 帧的特征轨迹"）。
/// 与方案 B 的 FeatureTrack 不同：不三角化、不恢复尺度、不做关键帧管理。
struct FeatureTrace {
  int id{0};
  std::vector<FeatureObs> observations;  ///< 最近若干帧观测（时间升序）
  int frames_seen{0};                    ///< 累计成功跟踪帧数
  bool active{true};                     ///< 是否仍在跟踪
};

/// 前端输出。
struct TrackResult {
  std::vector<FeatureObs> tracked;   ///< 本帧成功跟踪到的特征
  std::vector<FeatureTrace> traces;  ///< 短期轨迹（供交叉验证）
  int detected{0};                   ///< 本帧新检测到的角点数
  int rejected_fb{0};                ///< 前后向一致性剔除数量
  cv::Mat gray;                      ///< 去畸变灰度图
};

/// 轻量视觉前端：灰度金字塔 + FAST/Shi-Tomasi + PyrLK + 前后向一致性。
class KltFrontend {
 public:
  struct Options {
    int max_features{150};
    int pyramid_levels{3};
    double min_distance{10.0};
    double fb_error_threshold{1.0};
    int min_track_length{3};
    int trace_window{5};
    bool use_fast{true};
    int fast_threshold{20};
    double quality_level{0.01};
    int lk_window{21};
  };

  explicit KltFrontend(const Options& options);

  /// 处理一帧图像（BGR 或灰度均可）。
  TrackResult process(const cv::Mat& gray, double stamp);

  /// 清空内部状态（重定位时调用）。
  void reset();

  const Options& options() const;

 private:
  /// 角点检测（fast 或 shi-tomasi），已按 min_distance 抑制。
  std::vector<cv::Point2f> detect(const cv::Mat& gray, std::size_t budget) const;

  Options options_;
  cv::Mat prev_gray_;                  ///< 上一帧灰度图
  std::vector<FeatureObs> prev_obs_;   ///< 上一帧特征观测
  std::unordered_map<int, FeatureTrace> traces_;  ///< 短期轨迹缓存
  int next_id_{0};                     ///< 特征 ID 分配器
};

}  // namespace superlivo_a
