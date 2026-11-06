#include "pointcloud_colorizer/rgb_assignment.hpp"

namespace colorizer {

RgbAssigner::RgbAssigner(const Options& options) : options_(options) {}

bool RgbAssigner::assign(const cv::Mat& /*bgr*/,
                         const Eigen::Vector2d& /*pixel*/,
                         float /*quality*/,
                         ColoredPoint& /*point*/) const {
  // TODO(阶段 1): 按双线性/最近邻采样 BGR，写入点的 rgb 与 quality
  return false;
}

bool RgbAssigner::shouldOverride(float /*new_quality*/, float /*old_quality*/) const {
  // TODO(阶段 1): 依据质量阈值决定是否覆盖已有颜色
  return false;
}

const RgbAssigner::Options& RgbAssigner::options() const { return options_; }

}  // namespace colorizer
