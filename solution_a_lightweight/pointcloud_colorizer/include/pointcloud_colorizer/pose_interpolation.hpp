#pragma once

#include <vector>

#include <Eigen/Core>

namespace colorizer {

// ============================================================================
// 逐点位姿插值（运动补偿，对应 gpt双方案设计.md §2.4）
// ----------------------------------------------------------------------------
// gpt 文档强调：点云扫描不是瞬时完成的，必须考虑每个点的采集时间做运动补偿，
// 不能用帧头位姿代替。本类按时间戳维护位姿采样缓存，插值得到单点时刻的 T_WL。
// ============================================================================

/// 位姿采样点（T_WL：LiDAR 在世界系下的位姿，即 LiDAR 系 -> 世界系）。
struct PoseSample {
  double stamp{0.0};
  Eigen::Matrix4d T_wl{Eigen::Matrix4d::Identity()};
};

/// 为每个 LiDAR 点提供采集时刻的位姿，用于运动补偿。
class LidarPoseInterpolator {
 public:
  struct Options {
    double max_gap{0.05};  ///< 相邻采样最大时间间隔（秒），超限视为无有效插值
  };

  explicit LidarPoseInterpolator(const Options& options);

  /// 加入一个位姿采样（内部按时间升序维护，自动裁剪过旧项）。
  void addSample(const PoseSample& sample);

  /// 按时间戳插值：位置线性插值，旋转球面插值（slerp）。
  /// @return 采样不足、无邻居或相邻间隔超过 max_gap 时返回 false
  bool poseAt(double stamp, Eigen::Matrix4d& T_wl) const;

  void clear();

  const Options& options() const;

 private:
  Options options_;
  std::vector<PoseSample> samples_;  ///< 按 stamp 升序的采样队列
};

}  // namespace colorizer
