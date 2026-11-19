#ifndef SUPER_LIVO_COMMON_LATENCY_TIMER_HPP_
#define SUPER_LIVO_COMMON_LATENCY_TIMER_HPP_

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

namespace superlivo_common {

/// 线程安全的延时统计计时器。
/// 参照 Super-LIO `common/timer.h` 的口径：按名字累计 avg/max/calls，
/// 60 s 窗口 PrintAndClear 周期报告、退出前 PrintAll 总报告，
/// 与主干 `[Lidar->Odom]` 等指标同格式，便于日志并排对照。
/// 写入方：流水线工作线程（Record）；读取方：任意线程（打印/退出）。
class LatencyTimer {
 public:
  void Record(const std::string& name, double time_usage_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    records_[name].time_usage_in_ms_.emplace_back(time_usage_ms);
  }

  void PrintAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (records_.empty()) {
      return;
    }
    std::cout << ">>> ===== Printing run time =====" << std::endl;
    for (const auto& r : records_) {
      const auto& vec = r.second.time_usage_in_ms_;
      if (vec.empty()) {
        continue;
      }
      double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
      double avg = sum / double(vec.size());
      double mx = *std::max_element(vec.begin(), vec.end());
      std::cout << "> [ " << r.first << " ] avg: " << avg
                << " ms , max: " << mx << " ms , calls: " << vec.size() << std::endl;
    }
  }

  /// 打印当前统计并清空，使下一次打印为独立窗口（周期报告用）
  void PrintAndClear() {
    PrintAll();
    Clear();
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    records_.clear();
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& r : records_) {
      if (!r.second.time_usage_in_ms_.empty()) {
        return false;
      }
    }
    return true;
  }

 private:
  struct Entry {
    std::vector<double> time_usage_in_ms_;
  };
  mutable std::mutex mutex_;
  std::map<std::string, Entry> records_;
};

}  // namespace superlivo_common

#endif  // SUPER_LIVO_COMMON_LATENCY_TIMER_HPP_
