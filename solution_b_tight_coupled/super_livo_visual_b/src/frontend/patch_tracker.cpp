#include "super_livo_visual_b/frontend/patch_tracker.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace superlivo_b {

namespace {
bool rectInside(const cv::Rect& rect, const cv::Mat& image) {
  return rect.x >= 0 && rect.y >= 0 && rect.x + rect.width <= image.cols &&
         rect.y + rect.height <= image.rows;
}
}  // namespace

PatchTracker::PatchTracker(const Options& options) : options_(options) {}

void PatchTracker::buildPyramid(const cv::Mat& gray, int levels, std::vector<cv::Mat>& pyramid) {
  pyramid.clear();
  pyramid.push_back(gray);
  for (int i = 1; i < levels; ++i) {
    cv::Mat down;
    cv::pyrDown(pyramid.back(), down);  // 每层尺寸减半
    pyramid.push_back(down);
  }
}

std::vector<cv::Point2f> PatchTracker::detect(const cv::Mat& gray, std::size_t budget) const {
  std::vector<cv::Point2f> pts;
  if (budget == 0) {
    return pts;
  }
  cv::goodFeaturesToTrack(gray, pts, static_cast<int>(budget), options_.quality_level,
                          options_.min_distance);
  return pts;
}

std::vector<PatchObservation> PatchTracker::process(const cv::Mat& image, double stamp) {
  std::vector<PatchObservation> observations;

  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }
  if (gray.type() != CV_8UC1) {
    cv::Mat converted;
    gray.convertTo(converted, CV_8UC1);
    gray = converted;
  }

  const int levels = std::max(1, options_.pyramid_levels);
  std::vector<cv::Mat> pyramid;
  buildPyramid(gray, levels, pyramid);

  const int patch = std::max(3, options_.patch_size | 1);  // 奇数边长
  const int radius = std::max(1, options_.search_radius);

  // ---- 1. 对上一帧每个 patch 做 SSD 匹配 ----------------------------------
  if (!prev_pyramid_.empty() && !prev_obs_.empty()) {
    const int prev_levels = static_cast<int>(prev_pyramid_.size());

    for (const auto& obs : prev_obs_) {
      const int level = std::max(0, std::min({obs.level, prev_levels - 1, levels - 1}));
      const double scale = std::pow(2.0, level);
      const cv::Point2f prev_pt(static_cast<float>(obs.u / scale),
                                static_cast<float>(obs.v / scale));

      const cv::Mat& prev_img = prev_pyramid_[level];
      const cv::Mat& curr_img = pyramid[level];

      const cv::Rect patch_rect(static_cast<int>(std::lround(prev_pt.x)) - patch / 2,
                                static_cast<int>(std::lround(prev_pt.y)) - patch / 2, patch,
                                patch);
      const cv::Rect search_rect(patch_rect.x - radius, patch_rect.y - radius,
                                 patch + 2 * radius, patch + 2 * radius);
      if (!rectInside(patch_rect, prev_img) || !rectInside(search_rect, curr_img)) {
        continue;  // 靠近边界，无法取块
      }

      const cv::Mat templ = prev_img(patch_rect);
      const cv::Mat search = curr_img(search_rect);
      cv::Mat score_map;
      cv::matchTemplate(search, templ, score_map, cv::TM_SQDIFF_NORMED);
      if (score_map.empty()) {
        continue;
      }

      double min_val = 1.0;
      cv::Point min_loc(0, 0);
      cv::minMaxLoc(score_map, &min_val, nullptr, &min_loc, nullptr);

      const float score = static_cast<float>(1.0 - min_val);  // 1 = 完全匹配
      if (score < options_.lssd_threshold) {
        continue;  // 匹配质量不足，判为丢失
      }

      PatchObservation out;
      out.feature_id = obs.feature_id;
      out.u = static_cast<float>((search_rect.x + min_loc.x + patch / 2) * scale);
      out.v = static_cast<float>((search_rect.y + min_loc.y + patch / 2) * scale);
      out.stamp = stamp;
      out.level = level;
      out.score = score;
      observations.push_back(out);
    }
  }

  // ---- 2. 补充新特征，维持特征数量 ---------------------------------------
  if (static_cast<int>(observations.size()) < options_.max_features) {
    const std::size_t budget =
        static_cast<std::size_t>(options_.max_features - static_cast<int>(observations.size()));
    for (const auto& point : detect(pyramid[0], budget)) {
      bool too_close = false;
      for (const auto& obs : observations) {
        const double dx = obs.u - point.x;
        const double dy = obs.v - point.y;
        if (std::sqrt(dx * dx + dy * dy) < options_.min_distance) {
          too_close = true;
          break;
        }
      }
      if (too_close) {
        continue;
      }
      PatchObservation out;
      out.feature_id = next_id_++;
      out.u = point.x;
      out.v = point.y;
      out.stamp = stamp;
      out.level = 0;
      out.score = 1.0F;
      observations.push_back(out);
    }
  }

  // ---- 3. 保存本帧状态 ---------------------------------------------------
  prev_pyramid_ = pyramid;
  prev_obs_ = observations;
  return observations;
}

void PatchTracker::reset() {
  prev_pyramid_.clear();
  prev_obs_.clear();
  next_id_ = 0;
}

const PatchTracker::Options& PatchTracker::options() const { return options_; }

}  // namespace superlivo_b
