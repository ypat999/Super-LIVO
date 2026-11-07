#include "pointcloud_colorizer/occlusion_check.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace colorizer {

OcclusionChecker::OcclusionChecker(const Options& options) : options_(options) {}

void OcclusionChecker::beginFrame(int width, int height) {
  width_ = width;
  height_ = height;
  if (width <= 0 || height <= 0) {
    depth_buffer_.release();
    return;
  }
  // 用 +inf 表示"该像素没有可见点"，便于后续直接取 min 做 z-buffer。
  depth_buffer_ = cv::Mat(height, width, CV_32F, cv::Scalar(std::numeric_limits<float>::infinity()));
}

void OcclusionChecker::insertDepth(const Eigen::Vector2d& pixel, double depth) {
  if (depth_buffer_.empty() || depth <= 0.0) {
    return;
  }
  const int u = static_cast<int>(std::lround(pixel.x()));
  const int v = static_cast<int>(std::lround(pixel.y()));
  if (u < 0 || v < 0 || u >= depth_buffer_.cols || v >= depth_buffer_.rows) {
    return;
  }
  float& slot = depth_buffer_.at<float>(v, u);
  if (static_cast<float>(depth) < slot) {
    slot = static_cast<float>(depth);  // 只保留最近的可见深度
  }
}

bool OcclusionChecker::isVisible(const ProjectionResult& projection, double point_depth) const {
  if (depth_buffer_.empty() || point_depth <= 0.0) {
    return true;  // 没有深度缓冲信息时不做遮挡剔除
  }
  const int u = static_cast<int>(std::lround(projection.pixel.x()));
  const int v = static_cast<int>(std::lround(projection.pixel.y()));
  if (u < 0 || v < 0 || u >= depth_buffer_.cols || v >= depth_buffer_.rows) {
    return false;
  }
  const float visible = depth_buffer_.at<float>(v, u);
  if (!std::isfinite(visible)) {
    return true;  // 该像素没有任何点，无法判断遮挡
  }
  // 相对容差：只有明显更远（被前景挡住）才剔除。
  return point_depth <= static_cast<double>(visible) * (1.0 + options_.depth_tolerance);
}

bool OcclusionChecker::isDepthEdge(const ProjectionResult& projection) const {
  if (!options_.check_depth_edges || depth_buffer_.empty()) {
    return false;
  }
  const int u = static_cast<int>(std::lround(projection.pixel.x()));
  const int v = static_cast<int>(std::lround(projection.pixel.y()));

  float min_depth = std::numeric_limits<float>::infinity();
  float max_depth = 0.0F;
  bool has_data = false;
  for (int dv = -1; dv <= 1; ++dv) {
    for (int du = -1; du <= 1; ++du) {
      const int x = u + du;
      const int y = v + dv;
      if (x < 0 || y < 0 || x >= depth_buffer_.cols || y >= depth_buffer_.rows) {
        continue;
      }
      const float d = depth_buffer_.at<float>(y, x);
      if (!std::isfinite(d)) {
        continue;
      }
      has_data = true;
      min_depth = std::min(min_depth, d);
      max_depth = std::max(max_depth, d);
    }
  }
  if (!has_data || max_depth <= 0.0F || min_depth <= 0.0F) {
    return false;
  }
  return (max_depth - min_depth) > static_cast<float>(options_.edge_threshold);
}

const OcclusionChecker::Options& OcclusionChecker::options() const { return options_; }

}  // namespace colorizer
