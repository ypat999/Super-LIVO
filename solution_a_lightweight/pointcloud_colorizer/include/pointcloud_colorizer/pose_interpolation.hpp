#pragma once

#include <vector>

#include <Eigen/Core>

namespace colorizer {

/// 位姿采样点（T_WL：LiDAR 在世界系下的位姿）。
struct PoseSample {
  double stamp{0.0};
  Eigen::Matrix4d T_wl{Eigen::Matrix4d::Identity()};
};

/// 为每个 LiDAR 点提供采集时刻的位姿，用于运动补偿。
/// 点云不是瞬时完成的，必须按每点时间戳插值，不能用帧头位姿。
class LidarPoseInterpolator {
 public:
  struct Options {
    double max_gap{0.05};
  };

  explicit LidarPoseInterpolator(const Options& options);

  void addSample(const PoseSample& sample);

  /// 按时间戳插值（线性位置 + 旋转插值），无有效邻居时返回 false。
  bool poseAt(double stamp, Eigen::Matrix4d& T_wl) const;

  void clear();

  const Options& options() const;

 private:
  Options options_;
  // TODO: 位姿采样队列（按时间排序，保留插值窗口）
};

}  // namespace colorizer
