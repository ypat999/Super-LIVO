#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

namespace superlivo_common {

// ============================================================================
// 单体内 LIO → 视觉的推式共享缓冲
// ----------------------------------------------------------------------------
// 背景：`visual_a_app` / `visual_b_app` 虽然与 LIO 同进程，但此前仍靠 DDS 话题
// 回环取 LIO 输出。单体化后由 LIO 输出线程主动 `push`，视觉流水线按图像时刻
// `latest` / `nearest` / `bracket` 拉取，进程内不再经过 DDS。
//
// 约定：`Sample` 必须是值语义类型，且提供 `double stamp` 成员（秒）。
// 只保留最近 `Capacity` 帧，旧帧自动淘汰；点云等大对象请用 shared_ptr 放入
// `Sample`，拷贝只复制指针，不复制点。
//
// 本类不依赖 lio / pcl，具体载荷由调用方定义（见 A/B 的 pipeline 头文件）。
// ============================================================================
template <typename Sample, std::size_t Capacity = 64>
class LatestSampleBuffer {
public:
  void push(Sample s) {
    std::lock_guard<std::mutex> lk(mutex_);
    buf_.push_back(std::move(s));
    while (buf_.size() > Capacity) {
      buf_.pop_front();
    }
  }

  void clear() {
    std::lock_guard<std::mutex> lk(mutex_);
    buf_.clear();
  }

  bool empty() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return buf_.empty();
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return buf_.size();
  }

  /// 最新一帧。
  bool latest(Sample& out) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (buf_.empty()) return false;
    out = buf_.back();
    return true;
  }

  /// 时间戳最接近 `stamp` 的一帧。
  bool nearest(double stamp, Sample& out) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (buf_.empty()) return false;
    std::size_t best = 0;
    double best_dt = std::abs(buf_[0].stamp - stamp);
    for (std::size_t i = 1; i < buf_.size(); ++i) {
      const double dt = std::abs(buf_[i].stamp - stamp);
      if (dt < best_dt) {
        best_dt = dt;
        best = i;
      }
    }
    out = buf_[best];
    return true;
  }

  /// 取跨越 `stamp` 的前后两帧（供位姿插值）。只有一个可用帧时，
  /// before 与 after 都返回该帧并返回 false（表示无法插值）。
  bool bracket(double stamp, Sample& before, Sample& after) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (buf_.empty()) return false;
    if (buf_.size() == 1 || stamp <= buf_.front().stamp) {
      before = after = buf_.front();
      return false;
    }
    if (stamp >= buf_.back().stamp) {
      before = after = buf_.back();
      return false;
    }
    for (std::size_t i = 0; i + 1 < buf_.size(); ++i) {
      if (buf_[i].stamp <= stamp && stamp <= buf_[i + 1].stamp) {
        before = buf_[i];
        after = buf_[i + 1];
        return true;
      }
    }
    before = after = buf_.back();
    return false;
  }

private:
  mutable std::mutex mutex_;
  std::deque<Sample> buf_;
};

}  // namespace superlivo_common
