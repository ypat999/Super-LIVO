#pragma once

#include <vector>

#include <Eigen/Core>

namespace superlivo_b {

// ============================================================================
// 方案 B · 延迟地图提交调度器（对应 gpt双方案设计.md §3.4）
// ----------------------------------------------------------------------------
// gpt 文档特别提醒状态更新与地图插入的顺序：视觉更新会改变当前位姿，
// 不能用更新前位姿把点插入地图。需要重新计算相关变换，或采用延迟提交机制。
// 本类把地图提交推迟到本轮更新完成后，并用更新后的位姿重新赋值。
// ============================================================================

/// 待提交到地图的点云。
struct PendingCloud {
  std::vector<Eigen::Vector3f> points;
  Eigen::Matrix4d T_wl{Eigen::Matrix4d::Identity()};  ///< 采集该云时使用的位姿
  double stamp{0.0};
};

/// 延迟提交调度器。
class MapCommitScheduler {
 public:
  struct Options {
    bool enable{true};   ///< 是否启用延迟提交
    int max_pending{3};  ///< 待提交队列最大长度（超出丢弃最旧）
  };

  explicit MapCommitScheduler(const Options& options);

  /// 暂存点云，等待本轮更新完成。
  void stage(const PendingCloud& cloud);

  /// 用更新后的位姿刷新待提交点云并返回，供 OctVox 插入。
  std::vector<PendingCloud> flush(const Eigen::Matrix4d& T_wl);

  void clear();

  const Options& options() const;

 private:
  Options options_;
  std::vector<PendingCloud> pending_;
};

}  // namespace superlivo_b