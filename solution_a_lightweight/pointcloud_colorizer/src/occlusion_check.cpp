#include "pointcloud_colorizer/occlusion_check.hpp"

namespace colorizer {

OcclusionChecker::OcclusionChecker(const Options& options) : options_(options) {}

bool OcclusionChecker::isVisible(const ProjectionResult& /*projection*/,
                                 double /*point_depth*/) const {
  // TODO(阶段 1): 用深度缓冲/局部点云栅格判断该像素的可见深度，过滤被遮挡的点
  return false;
}

bool OcclusionChecker::isDepthEdge(const ProjectionResult& /*projection*/) const {
  // TODO(阶段 1): 检查像素邻域深度不连续，标记可能错色的边缘点
  return false;
}

const OcclusionChecker::Options& OcclusionChecker::options() const { return options_; }

}  // namespace colorizer
