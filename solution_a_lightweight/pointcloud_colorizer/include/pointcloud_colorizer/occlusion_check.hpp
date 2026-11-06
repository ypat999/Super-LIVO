#pragma once

#include <Eigen/Core>

#include "pointcloud_colorizer/projection.hpp"

namespace colorizer {

/// 遮挡与深度边界检查。
/// 用于避免把被前景遮挡的 LiDAR 点错误地染上背景颜色。
class OcclusionChecker {
 public:
  struct Options {
    double depth_tolerance{0.1};   ///< 相对深度容差
    bool check_depth_edges{true};
  };

  explicit OcclusionChecker(const Options& options);

  /// 深度缓冲检查：point_depth 与相机深度图上对应像素的深度是否一致。
  bool isVisible(const ProjectionResult& projection, double point_depth) const;

  /// 是否落在深度不连续边缘（可能产生错色）。
  bool isDepthEdge(const ProjectionResult& projection) const;

  const Options& options() const;

 private:
  Options options_;
  // TODO: 深度图/局部点云栅格缓冲
};

}  // namespace colorizer
