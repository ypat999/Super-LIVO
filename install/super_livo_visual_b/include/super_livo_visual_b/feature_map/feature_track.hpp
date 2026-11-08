#pragma once

#include <vector>

#include <Eigen/Core>

namespace superlivo_b {

// ============================================================================
// 方案 B · 特征轨迹（对应 gpt双方案设计.md §3.3 "多帧视觉轨迹约束"）
// ----------------------------------------------------------------------------
// 即使 OctVox 几何地图出现扭曲，连续图像中同一特征仍能形成独立的多帧运动约束。
// ============================================================================

/// 单个特征的一次观测：像素坐标 + 当时相机在世界系下的位姿。
struct FeatureObservation {
  double stamp{0.0};
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
  Eigen::Matrix4d T_wc{Eigen::Matrix4d::Identity()};  ///< 相机 -> 世界
};

/// 特征轨迹：同一空间特征的多帧观测。
struct FeatureTrack {
  int id{0};
  std::vector<FeatureObservation> observations;       ///< 多帧观测（时间升序）
  Eigen::Vector3d xyz_world{Eigen::Vector3d::Zero()}; ///< 三角化结果（可选）
  double quality{0.0};
  bool triangulated{false};
  bool valid{false};

  /// 追加一次观测（自动保持时间升序）。
  void addObservation(const FeatureObservation& observation);

  /// 计算轨迹内最大视差（度）。视差过小时三角化与航向约束都不可靠。
  double parallaxDeg() const;

  /// 轨迹是否具备进入多帧约束的基本条件。
  bool readyForConstraint(int min_observations, double min_parallax_deg) const;
};

}  // namespace superlivo_b
