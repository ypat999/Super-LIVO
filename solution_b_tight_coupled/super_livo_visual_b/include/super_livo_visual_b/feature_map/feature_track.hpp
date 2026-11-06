#pragma once

#include <vector>

#include <Eigen/Core>

namespace superlivo_b {

/// 单个特征的一次观测：像素坐标 + 当时相机系下的位姿。
struct FeatureObservation {
  double stamp{0.0};
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
  Eigen::Matrix4d T_wc{Eigen::Matrix4d::Identity()};
};

/// 特征轨迹：同一空间特征的多帧观测。
/// 用于在不依赖 OctVox 全局地图的情况下构造独立的视觉运动约束。
struct FeatureTrack {
  int id{0};
  std::vector<FeatureObservation> observations;
  Eigen::Vector3d xyz_world{Eigen::Vector3d::Zero()};  ///< 三角化结果（可选）
  double quality{0.0};
  bool triangulated{false};
  bool valid{false};
};

}  // namespace superlivo_b
