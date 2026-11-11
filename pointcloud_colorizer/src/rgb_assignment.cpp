#include "pointcloud_colorizer/rgb_assignment.hpp"

#include <algorithm>
#include <cmath>

namespace colorizer {

RgbAssigner::RgbAssigner(const Options& options) : options_(options) {}

bool RgbAssigner::assign(const cv::Mat& bgr,
                         const Eigen::Vector2d& pixel,
                         float quality,
                         ColoredPoint& point) const {
  if (bgr.empty() || quality < options_.min_quality) {
    return false;
  }

  const int width = bgr.cols;
  const int height = bgr.rows;
  const double u = pixel.x();
  const double v = pixel.y();
  if (u < 0.0 || v < 0.0 || u >= static_cast<double>(width - 1) ||
      v >= static_cast<double>(height - 1)) {
    return false;
  }

  const bool is_bgr = (bgr.channels() >= 3);
  auto sample = [&](int x, int y, int c) -> float {
    const cv::Vec3b& px = bgr.at<cv::Vec3b>(y, x);
    return static_cast<float>(px[is_bgr ? c : 0]);
  };

  float b = 0.0F;
  float g = 0.0F;
  float r = 0.0F;

  if (options_.bilinear) {
    const int x0 = static_cast<int>(std::floor(u));
    const int y0 = static_cast<int>(std::floor(v));
    const double ax = u - x0;
    const double ay = v - y0;
    for (int c = 0; c < 3; ++c) {
      const float v00 = sample(x0, y0, c);
      const float v10 = sample(x0 + 1, y0, c);
      const float v01 = sample(x0, y0 + 1, c);
      const float v11 = sample(x0 + 1, y0 + 1, c);
      const float value = static_cast<float>((1.0 - ay) * ((1.0 - ax) * v00 + ax * v10) +
                                             ay * ((1.0 - ax) * v01 + ax * v11));
      if (c == 0) {
        b = value;
      } else if (c == 1) {
        g = value;
      } else {
        r = value;
      }
    }
  } else {
    const int x = static_cast<int>(std::lround(u));
    const int y = static_cast<int>(std::lround(v));
    b = sample(x, y, 0);
    g = sample(x, y, 1);
    r = sample(x, y, 2);
  }

  // OpenCV 的图像通道顺序是 BGR，这里转成 RGB 写入点云。
  point.rgb = Eigen::Vector3f(std::min(r, 255.0F), std::min(g, 255.0F), std::min(b, 255.0F));
  point.quality = quality;
  return true;
}

bool RgbAssigner::shouldOverride(float new_quality, float old_quality) const {
  if (new_quality < options_.min_quality) {
    return false;
  }
  if (!options_.keep_best_quality) {
    return true;
  }
  return new_quality > old_quality;
}

const RgbAssigner::Options& RgbAssigner::options() const { return options_; }

}  // namespace colorizer
