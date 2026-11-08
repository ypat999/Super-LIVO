#pragma once

#include <unordered_map>
#include <vector>

namespace superlivo_b {

// ============================================================================
// 方案 B · 特征生命周期管理（对应 gpt双方案设计.md §3.2 / §四）
// ----------------------------------------------------------------------------
// 特征经历 kNew -> kTracked -> kMature -> kLost / kMarginalized 的完整生命周期；
// 只有足够成熟（观测次数 >= mature_observations）的特征才进入多帧约束。
// 这是方案 B 与方案 A（仅短期缓存）的核心差异之一。
// ============================================================================

/// 特征生命周期状态。
enum class FeatureStatus {
  kNew,          ///< 新检测，还未稳定跟踪
  kTracked,      ///< 正在跟踪
  kMature,       ///< 观测足够，可参与多帧约束
  kLost,         ///< 连续失败超限，丢弃
  kMarginalized  ///< 轨迹过长，已边缘化
};

/// 单个特征的生命周期统计。
struct FeatureLifecycleInfo {
  int feature_id{0};
  FeatureStatus status{FeatureStatus::kNew};
  int observations{0};
  int consecutive_failures{0};
  double first_stamp{0.0};
  double last_stamp{0.0};
};

/// 特征生命周期管理。
class FeatureLifecycle {
 public:
  struct Options {
    int mature_observations{4};
    int max_failures{2};
    int max_track_length{30};
  };

  explicit FeatureLifecycle(const Options& options);

  /// 记录一次成功观测，返回更新后的状态。
  FeatureLifecycleInfo onObserved(int feature_id, double stamp);

  /// 记录一次跟踪失败，失败次数超限则标记丢失。
  FeatureLifecycleInfo onMissed(int feature_id, double stamp);

  /// 取出可以进入多帧约束的特征（kMature）。
  std::vector<int> matureFeatures() const;

  /// 清理丢失/边缘化记录。
  void prune();

  const Options& options() const;

 private:
  Options options_;
  std::unordered_map<int, FeatureLifecycleInfo> features_;  ///< feature_id -> 生命周期
};

}  // namespace superlivo_b
