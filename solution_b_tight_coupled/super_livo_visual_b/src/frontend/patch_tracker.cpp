#include "super_livo_visual_b/frontend/patch_tracker.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace superlivo_b {

namespace {
bool rectInside(const cv::Rect& rect, const cv::Mat& image) {
  return rect.x >= 0 && rect.y >= 0 && rect.x + rect.width <= image.cols &&
         rect.y + rect.height <= image.rows;
}
}  // namespace

PatchTracker::PatchTracker(const Options& options) : options_(options) {}

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
  const int patch = std::max(3, options_.patch_size | 1);  // 奇数边长
  const int radius = std::max(1, options_.search_radius);

  // ---- 1. 金字塔 LK 跟踪上一帧的每个 patch --------------------------------
  // 为什么不是单层 5x5 SSD 模板匹配（原实现）：实测该数据集帧间光流中位
  // 6~25 px（Retail_Street 手持快速转动），单层 ±search_radius 的 5x5 SSD 在
  // 大位移下会锁到错误极值——同一批特征用 LK 复算，位移差中位 6~16 px，
  // 94~100% 的特征差 >3 px，视觉相对旋转因此偏差 1.5~2 deg（是真实值的数倍），
  // 修正器只会把位姿往错方向拉。改用 LK（自带金字塔 + 亚像素）后位移一致。
  if (!prev_gray_.empty() && !prev_obs_.empty()) {
    std::vector<cv::Point2f> pts_prev;
    pts_prev.reserve(prev_obs_.size());
    for (const auto& obs : prev_obs_) {
      pts_prev.emplace_back(obs.u, obs.v);
    }
    std::vector<cv::Point2f> pts_curr;
    std::vector<unsigned char> status;
    std::vector<float> err;
    const cv::Size win(2 * radius + 1, 2 * radius + 1);
    cv::calcOpticalFlowPyrLK(
        prev_gray_, gray, pts_prev, pts_curr, status, err, win, levels - 1,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01), 0);

    for (std::size_t k = 0; k < prev_obs_.size(); ++k) {
      if (status[k] == 0) {
        continue;  // LK 未收敛
      }
      const cv::Rect prev_rect(static_cast<int>(std::lround(pts_prev[k].x)) - patch / 2,
                               static_cast<int>(std::lround(pts_prev[k].y)) - patch / 2,
                               patch, patch);
      const cv::Rect curr_rect(static_cast<int>(std::lround(pts_curr[k].x)) - patch / 2,
                               static_cast<int>(std::lround(pts_curr[k].y)) - patch / 2,
                               patch, patch);
      if (!rectInside(prev_rect, prev_gray_) || !rectInside(curr_rect, gray)) {
        continue;  // 靠近边界，无法取块
      }
      // 得分口径与原实现一致：1 - TM_SQDIFF_NORMED（1 = 完全匹配）
      cv::Mat score_map;
      cv::matchTemplate(gray(curr_rect), prev_gray_(prev_rect), score_map,
                        cv::TM_SQDIFF_NORMED);
      if (score_map.empty()) {
        continue;
      }
      const float score = static_cast<float>(1.0 - score_map.at<float>(0, 0));
      if (score < options_.lssd_threshold) {
        continue;  // 匹配质量不足，判为丢失
      }

      PatchObservation out;
      out.feature_id = prev_obs_[k].feature_id;
      out.u = pts_curr[k].x;
      out.v = pts_curr[k].y;
      out.stamp = stamp;
      out.level = 0;
      out.score = score;
      observations.push_back(out);
    }
  }

  // ---- 2. 补充新特征，维持特征数量 ---------------------------------------
  if (static_cast<int>(observations.size()) < options_.max_features) {
    const std::size_t budget =
        static_cast<std::size_t>(options_.max_features - static_cast<int>(observations.size()));
    for (const auto& point : detect(gray, budget)) {
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
  prev_gray_ = gray;
  prev_obs_ = observations;
  return observations;
}

void PatchTracker::reset() {
  prev_gray_.release();
  prev_obs_.clear();
  next_id_ = 0;
}

const PatchTracker::Options& PatchTracker::options() const { return options_; }

}  // namespace superlivo_b
