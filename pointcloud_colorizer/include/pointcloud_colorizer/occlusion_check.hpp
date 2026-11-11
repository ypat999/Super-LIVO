#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "pointcloud_colorizer/projection.hpp"

namespace colorizer {

// ============================================================================
// 遮挡与深度边界检查（对应 gpt双方案设计.md §2.4）
// ----------------------------------------------------------------------------
// gpt 文档强调：运动补偿 + 投影后还需做"视野、深度和遮挡检查"，避免把被前景
// 遮挡的 LiDAR 点错误地染上背景颜色。深度边界处（前景/背景交界）也易错色。
//
// 实现方式：以图像像素为索引维护一张 z-buffer（每像素记录最近的相机深度）。
// 使用流程（同一帧内两遍扫描）：
//   1. beginFrame(w, h)          清空缓冲；
//   2. insertDepth(pixel, depth) 把所有可投影点的深度光栅化进缓冲；
//   3. isVisible / isDepthEdge   在第二遍扫描中逐点判定。
// ============================================================================

/// 遮挡与深度边界检查。
class OcclusionChecker {
 public:
  struct Options {
    double depth_tolerance{0.1};   ///< 相对深度容差：point_depth > buffer*(1+tol) 判为被遮挡
    bool check_depth_edges{true};  ///< 是否做深度不连续边缘检查
    double edge_threshold{0.5};    ///< 3x3 邻域深度极差超过该值（米）判为边缘
  };

  explicit OcclusionChecker(const Options& options);

  /// 开始新一帧：按图像尺寸重置深度缓冲（全部置为 +inf 表示空）。
  void beginFrame(int width, int height);

  /// 把一个投影点的深度写入缓冲（取该像素的最小深度，即 z-buffer）。
  void insertDepth(const Eigen::Vector2d& pixel, double depth);

  /// 深度缓冲检查：point_depth 显著大于缓冲可见深度时判为被遮挡。
  /// 缓冲为空（未调用 beginFrame）时返回 true，表示不做遮挡剔除。
  bool isVisible(const ProjectionResult& projection, double point_depth) const;

  /// 是否落在深度不连续边缘（可能产生错色）。
  bool isDepthEdge(const ProjectionResult& projection) const;

  const Options& options() const;

 private:
  Options options_;
  cv::Mat depth_buffer_;  ///< CV_32F，无效像素为 +inf
  int width_{0};
  int height_{0};
};

}  // namespace colorizer
