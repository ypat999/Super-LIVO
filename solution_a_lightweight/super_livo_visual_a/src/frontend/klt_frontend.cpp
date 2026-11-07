#include "super_livo_visual_a/frontend/klt_frontend.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace superlivo_a {

KltFrontend::KltFrontend(const Options& options) : options_(options) {}

std::vector<cv::Point2f> KltFrontend::detect(const cv::Mat& gray, std::size_t budget) const {
  std::vector<cv::Point2f> pts;
  if (budget == 0) {
    return pts;
  }

  if (options_.use_fast) {
    // FAST + 手写距离抑制：按响应降序贪心保留。
    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(gray, keypoints, options_.fast_threshold, true);
    std::sort(keypoints.begin(), keypoints.end(),
              [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });

    for (const auto& kp : keypoints) {
      if (pts.size() >= budget) {
        break;
      }
      bool ok = true;
      for (const auto& p : pts) {
        if (cv::norm(p - kp.pt) < options_.min_distance) {
          ok = false;
          break;
        }
      }
      if (ok) {
        pts.push_back(kp.pt);
      }
    }
  } else {
    // Shi-Tomasi（goodFeaturesToTrack 自带 minDistance 抑制）。
    cv::goodFeaturesToTrack(gray, pts, static_cast<int>(budget), options_.quality_level,
                            options_.min_distance);
  }
  return pts;
}

TrackResult KltFrontend::process(const cv::Mat& image, double stamp) {
  TrackResult result;

  // ---- 0. 统一成单通道灰度图 ----------------------------------------------
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
  result.gray = gray;

  const int max_level = std::max(0, options_.pyramid_levels - 1);
  const cv::Size win(options_.lk_window, options_.lk_window);

  std::vector<FeatureObs> tracked;
  tracked.reserve(static_cast<std::size_t>(options_.max_features));

  // ---- 1. PyrLK 跟踪 + 前后向一致性检查 -----------------------------------
  if (!prev_gray_.empty() && !prev_obs_.empty()) {
    std::vector<cv::Point2f> prev_pts;
    prev_pts.reserve(prev_obs_.size());
    for (const auto& obs : prev_obs_) {
      prev_pts.emplace_back(obs.u, obs.v);
    }

    std::vector<cv::Point2f> next_pts;
    std::vector<uchar> status_forward;
    std::vector<float> err_forward;
    cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_pts, next_pts, status_forward, err_forward, win,
                             max_level);

    // 反向跟踪：拿正向结果回到上一帧，用于前后向一致性检查。
    std::vector<cv::Point2f> forward_pts = next_pts;
    for (std::size_t i = 0; i < forward_pts.size(); ++i) {
      if (!status_forward[i]) {
        forward_pts[i] = prev_pts[i];  // 占位，保证索引对齐
      }
    }
    std::vector<cv::Point2f> back_pts;
    std::vector<uchar> status_backward;
    std::vector<float> err_backward;
    cv::calcOpticalFlowPyrLK(gray, prev_gray_, forward_pts, back_pts, status_backward, err_backward,
                             win, max_level);

    for (std::size_t i = 0; i < prev_obs_.size(); ++i) {
      if (!status_forward[i] || !status_backward[i]) {
        continue;  // 跟踪失败，丢弃
      }
      const double fb_error = cv::norm(back_pts[i] - prev_pts[i]);
      if (fb_error > options_.fb_error_threshold) {
        ++result.rejected_fb;  // 前后向不一致，判为错误匹配
        continue;
      }
      const cv::Point2f& p = next_pts[i];
      if (p.x < 0.0F || p.y < 0.0F || p.x >= static_cast<float>(gray.cols) ||
          p.y >= static_cast<float>(gray.rows)) {
        continue;  // 越界
      }

      FeatureObs obs;
      obs.id = prev_obs_[i].id;
      obs.u = p.x;
      obs.v = p.y;
      obs.level = 0;
      obs.stamp = stamp;
      tracked.push_back(obs);
    }
  }

  // ---- 2. 补充新检测特征，保持特征数量 -----------------------------------
  const std::size_t budget =
      tracked.size() < static_cast<std::size_t>(options_.max_features)
          ? static_cast<std::size_t>(options_.max_features) - tracked.size()
          : 0;
  if (budget > 0) {
    const auto fresh = detect(gray, budget);
    for (const auto& c : fresh) {
      bool too_close = false;
      for (const auto& obs : tracked) {
        if (cv::norm(cv::Point2f(obs.u, obs.v) - c) < options_.min_distance) {
          too_close = true;
          break;
        }
      }
      if (too_close) {
        continue;
      }
      FeatureObs obs;
      obs.id = next_id_++;
      obs.u = c.x;
      obs.v = c.y;
      obs.level = 0;
      obs.stamp = stamp;
      tracked.push_back(obs);
      ++result.detected;
    }
  }

  // ---- 3. 更新短期轨迹缓存（最近 trace_window 帧） ------------------------
  const std::size_t window =
      options_.trace_window > 0 ? static_cast<std::size_t>(options_.trace_window) : 1U;

  for (const auto& obs : tracked) {
    auto& trace = traces_[obs.id];
    trace.id = obs.id;
    trace.active = true;
    trace.observations.push_back(obs);
    if (trace.observations.size() > window) {
      trace.observations.erase(trace.observations.begin(),
                               trace.observations.end() - static_cast<long>(window));
    }
    trace.frames_seen += 1;
  }

  // 未在本帧出现的轨迹：标记失效并清理，避免缓存无限增长。
  for (auto it = traces_.begin(); it != traces_.end();) {
    bool present = false;
    for (const auto& obs : tracked) {
      if (obs.id == it->first) {
        present = true;
        break;
      }
    }
    if (!present) {
      it = traces_.erase(it);
    } else {
      ++it;
    }
  }

  result.traces.reserve(traces_.size());
  for (const auto& kv : traces_) {
    result.traces.push_back(kv.second);
  }
  result.tracked = std::move(tracked);

  // ---- 4. 保存本帧状态，供下一帧跟踪 -------------------------------------
  prev_gray_ = gray;
  prev_obs_ = result.tracked;

  return result;
}

void KltFrontend::reset() {
  prev_gray_.release();
  prev_obs_.clear();
  traces_.clear();
  next_id_ = 0;
}

const KltFrontend::Options& KltFrontend::options() const { return options_; }

}  // namespace superlivo_a
