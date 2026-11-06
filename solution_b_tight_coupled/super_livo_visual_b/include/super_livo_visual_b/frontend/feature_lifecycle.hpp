#pragma once

#include <cstdint>
#include <vector>

namespace superlivo_b {

/// 特征生命周期状态。
enum class FeatureStatus {
  kNew,
  kTracked,
  kMature,   ///< 观测足够，可参与多帧约束
  kLost,
  kMarginalized
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

/// 特征生命周期管理：新建、增长、成熟、丢失与边缘化。
/// 是方案 B 与方案 A（仅短期缓存）的核心差异之一。
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

  /// 取出可以进入多帧约束的特征。
  std::vector<int> matureFeatures() const;

  /// 清理丢失/边缘化的记录。
  void prune();

  const Options& options() const;

 private:
  Options options_;
  // TODO: feature_id -> FeatureLifecycleInfo 表
};

}  // namespace superlivo_b
