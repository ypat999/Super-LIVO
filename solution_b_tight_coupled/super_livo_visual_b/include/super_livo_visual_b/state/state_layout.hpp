#pragma once

#include <Eigen/Core>

namespace superlivo_b {

/// 统一滤波器状态：X = [R, p, v, bg, ba, g, T_IC, T_IL]
/// 外参已可靠标定时可固定，不在线估计。
struct State {
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
  Eigen::Vector3d g{0.0, 0.0, -9.81};
  Eigen::Matrix4d T_ic{Eigen::Matrix4d::Identity()};  ///< IMU -> Camera
  Eigen::Matrix4d T_il{Eigen::Matrix4d::Identity()};  ///< IMU -> LiDAR
};

/// 状态块的维度与索引约定，供 H/b 拼接时对齐。
struct StateLayout {
  static constexpr int kRot = 0;
  static constexpr int kPos = 3;
  static constexpr int kVel = 6;
  static constexpr int kBg = 9;
  static constexpr int kBa = 12;
  static constexpr int kGrav = 15;
  static constexpr int kExtrinsic = 18;
  static constexpr int kSize = 30;  ///< 6(IC) + 6(IL)
};

/// 滤波器协方差与状态容器（由宿主 ESIKF 持有）。
struct FilterState {
  State state;
  Eigen::MatrixXd covariance;
};

}  // namespace superlivo_b
