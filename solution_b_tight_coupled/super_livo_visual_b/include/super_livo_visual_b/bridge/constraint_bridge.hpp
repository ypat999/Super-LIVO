#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <utility>

#include <Eigen/Core>

namespace superlivo_b {

// ============================================================================
// 方案 B · 视觉约束 -> Super-LIO ESKF 的进程内桥接
// ----------------------------------------------------------------------------
// 视觉前端在图像回调线程里算出 6 维相机位姿法方程（H 6x6、b 6x1），
// LIO 主干在 process 线程里通过 SuperLIO::ExternalObserve() 取用。
// 两者线程不同，且视觉帧率低于雷达帧率，因此这里只保留"最新一条"：
//   - 不做排队积压（旧的视觉约束对当前状态已无意义）；
//   - take() 是消费式的，同一条约束只会被注入一次。
// ============================================================================

/// 一条已白化、已映射到 ESKF 18 维误差状态的观测。
///   H : m×18，列序 R(0-2) p(3-5) v(6-8) bg(9-11) ba(12-14) g(15-17)
///   r : m×1，残差（观测 ⊖ 预测，与 ESKF::UpdateObserve 的符号约定一致）
///   R : m×m，观测噪声协方差（已白化时为单位阵）
struct VisualObs {
  double stamp{0.0};
  Eigen::MatrixXd H;
  Eigen::VectorXd r;
  Eigen::MatrixXd R;
};

/// 单槽信箱：视觉线程 push，LIO 线程 take。
class ConstraintBridge {
 public:
  void push(VisualObs obs) {
    std::lock_guard<std::mutex> lock(mtx_);
    slot_ = std::move(obs);
    has_ = true;
    ++pushed_;
  }

  /// 取走当前约束；没有则返回 false。取走后该约束不会被再次注入。
  bool take(VisualObs& out) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_) {
      return false;
    }
    has_ = false;
    out = std::move(slot_);
    slot_ = VisualObs{};
    ++taken_;
    return true;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    has_ = false;
    slot_ = VisualObs{};
  }

  /// 统计：push 次数 / take 次数（供 1Hz 日志判断通路是否打通）。
  std::pair<long, long> stats() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return {pushed_, taken_};
  }

 private:
  mutable std::mutex mtx_;
  VisualObs slot_;
  bool has_{false};
  long pushed_{0};
  long taken_{0};
};

}  // namespace superlivo_b
