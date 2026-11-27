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
  const int over_thr = options_.overexposed_min;
  // 亮背景（过曝天空）判定：取"最小通道"而不是三通道同时超限。
  // 海康 ACE 经 WB trim（R×0.971）+ Gamma 后 R 通道封顶在 245~249，
  // "三通道同时 >250" 实测整帧只有 0.01% 像素成立，等于过滤器失效；
  // 改看最小通道后，天空（min 235~254）可稳定识别，白墙/玻璃/地面几乎零误伤。
  auto overexposed = [&](float pb, float pg, float pr) {
    return std::min({pb, pg, pr}) > static_cast<float>(over_thr);
  };

  float b = 0.0F;
  float g = 0.0F;
  float r = 0.0F;

  if (options_.bilinear) {
    const int x0 = static_cast<int>(std::floor(u));
    const int y0 = static_cast<int>(std::floor(v));
    const double ax = u - x0;
    const double ay = v - y0;
    const int xs[4] = {x0, x0 + 1, x0, x0 + 1};
    const int ys[4] = {y0, y0, y0 + 1, y0 + 1};
    double w[4] = {(1.0 - ax) * (1.0 - ay), ax * (1.0 - ay),
                   (1.0 - ax) * ay,         ax * ay};
    bool dead[4] = {false, false, false, false};
    if (over_thr > 0) {
      // 过曝是"整像素"属性：饱和邻点直接剔除，剩余权重归一化；四点全饱和弃色。
      double wsum = 0.0;
      for (int i = 0; i < 4; ++i) {
        if (overexposed(sample(xs[i], ys[i], 0), sample(xs[i], ys[i], 1),
                        sample(xs[i], ys[i], 2))) {
          dead[i] = true;
          w[i] = 0.0;
        } else {
          wsum += w[i];
        }
      }
      if (wsum <= 1e-9) {
        return false;
      }
      for (int i = 0; i < 4; ++i) {
        w[i] /= wsum;
      }
    }
    for (int c = 0; c < 3; ++c) {
      float value = 0.0F;
      for (int i = 0; i < 4; ++i) {
        if (!dead[i]) {
          value += static_cast<float>(w[i] * sample(xs[i], ys[i], c));
        }
      }
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
    if (over_thr > 0 && overexposed(b, g, r)) {
      return false;
    }
  }

  // 邻域均值（可选）：以中心色为基准，剔除跨颜色边缘的像素后再平均，
  // 降低"落点偏移几像素 → 取到相邻表面"的坏点比例。
  if (options_.patch_radius > 0) {
    const int rad = options_.patch_radius;
    const int cx = static_cast<int>(std::lround(u));
    const int cy = static_cast<int>(std::lround(v));
    float sb = 0.0F;
    float sg = 0.0F;
    float sr = 0.0F;
    int n = 0;
    for (int dy = -rad; dy <= rad; ++dy) {
      const int y = cy + dy;
      if (y < 0 || y >= height) continue;
      for (int dx = -rad; dx <= rad; ++dx) {
        const int x = cx + dx;
        if (x < 0 || x >= width) continue;
        const float pb = sample(x, y, 0);
        const float pg = sample(x, y, 1);
        const float pr = sample(x, y, 2);
        if (over_thr > 0 && overexposed(pb, pg, pr)) continue;  // 过曝像素不参与均值
        const float dev = std::max({std::fabs(pb - b), std::fabs(pg - g), std::fabs(pr - r)});
        if (dev > options_.patch_color_tol) continue;
        sb += pb;
        sg += pg;
        sr += pr;
        ++n;
      }
    }
    if (n > 0) {
      b = sb / static_cast<float>(n);
      g = sg / static_cast<float>(n);
      r = sr / static_cast<float>(n);
    }
  }

  // OpenCV 的图像通道顺序是 BGR，这里转成 RGB 写入点云。
  if (over_thr > 0 && overexposed(b, g, r)) {
    return false;  // 兜底：插值/均值后仍整体饱和（如大面积过曝区中心）
  }
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
