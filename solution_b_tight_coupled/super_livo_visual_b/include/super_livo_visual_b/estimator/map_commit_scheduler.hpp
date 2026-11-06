#pragma once

#include <vector>

#include <Eigen/Core>

namespace superlivo_b {

/// 待提交到地图的点云。
struct PendingCloud {
  std::vector<Eigen::Vector3f> points;
  Eigen::Matrix4d T_wl{Eigen::Matrix4d::Identity()};  ///< 采集该云时使用的位姿
  double stamp{0.0};
};

/// 延迟提交调度器。
/// 视觉更新会改变当前位姿，不能用更新前位姿直接插点；
/// 这里把地图提交推迟到本轮更新完成后，并重新计算相关变换。
class MapCommitScheduler {
 public:
  struct Options {
    bool enable{true};
    int max_pending{3};
  };

  explicit MapCommitScheduler(const Options& options);

  /// 暂存点云，等待本轮更新完成。
  void stage(const PendingCloud& cloud);

  /// 用更新后的位姿重新赋值并返回可提交的点云。
  std::vector<PendingCloud> flush(const Eigen::Matrix4d& T_wl);

  void clear();

  const Options& options() const;

 private:
  Options options_;
  std::vector<PendingCloud> pending_;
};

}  // namespace superlivo_b
