#pragma once

#include <Eigen/Core>

namespace superlivo_b {

// ============================================================================
// 方案 B · 统一滤波器状态布局（对应 gpt双方案设计.md §3.4）
// ----------------------------------------------------------------------------
// gpt 文档定义状态：X = [R, p, v, b_g, b_a, g, T_IC, T_IL]
//   - R, p, v  ：IMU 姿态/位置/速度
//   - b_g, b_a ：陀螺/加计零偏
//   - g        ：重力
//   - T_IC, T_IL ：IMU-相机 / IMU-LiDAR 外参（可靠标定后可固定，第一版不必在线估计）
//
// 仍以 Super-LIO 的 ESIKF 为中心，不引入第二套独立 EKF/滑窗优化器。
// StateLayout 仅约定各状态块在向量中的索引，便于 H/b 拼接对齐。
// ============================================================================

/// 统一滤波器状态：X = [R, p, v, bg, ba, g, T_IC, T_IL]。
struct State {
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};  ///< 姿态（世界系 <- IMU 系）
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};      ///< 位置
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};      ///< 速度
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};     ///< 陀螺零偏
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};     ///< 加计零偏
  Eigen::Vector3d g{0.0, 0.0, -9.81};              ///< 重力
  Eigen::Matrix4d T_ic{Eigen::Matrix4d::Identity()};///< IMU -> Camera 外参
  Eigen::Matrix4d T_il{Eigen::Matrix4d::Identity()};///< IMU -> LiDAR 外参
};

/// 状态块的维度与索引约定，供 H/b 拼接时对齐。
/// 块顺序（每个姿态用 so(3) 三参数，外参每项 6 参数 = 旋转3 + 平移3）：
///   旋转(0..2) 位置(3..5) 速度(6..8) b_g(9..11) b_a(12..14)
///   重力(15..17) 外参(18..29)，其中 T_IC 占 18..23、T_IL 占 24..29。
struct StateLayout {
  static constexpr int kRot = 0;        ///< 旋转块起始
  static constexpr int kPos = 3;        ///< 位置块起始
  static constexpr int kVel = 6;        ///< 速度块起始
  static constexpr int kBg = 9;         ///< 陀螺零偏块起始
  static constexpr int kBa = 12;        ///< 加计零偏块起始
  static constexpr int kGrav = 15;      ///< 重力块起始
  static constexpr int kExtrinsic = 18; ///< 外参块起始（T_IC + T_IL）
  static constexpr int kSize = 30;      ///< 18（IMU 部分）+ 12（T_IC 6 + T_IL 6）
};

/// 滤波器协方差与状态容器（由宿主 ESIKF 持有）。
struct FilterState {
  State state;
  Eigen::MatrixXd covariance;  ///< 统一状态协方差，维度 = StateLayout::kSize
};

}  // namespace superlivo_b