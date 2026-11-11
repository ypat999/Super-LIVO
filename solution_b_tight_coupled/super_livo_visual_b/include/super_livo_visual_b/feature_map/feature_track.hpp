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

  /// 单条轨迹的观测史上限：保留首帧（最长基线，视差与三角化最有利）+ 最近若干帧。
  /// 不截断的话，长期存活的锚点观测数会涨到数百，视差扫描与 MSCKF 零空间都按 n² 膨胀。
  static constexpr std::size_t kMaxObservations = 16;

  /// 追加一次观测（自动保持时间升序，并按 kMaxObservations 裁剪过长历史）。
  void addObservation(const FeatureObservation& observation);

  /// 轨迹内最大视差（度）。视差过小时三角化与航向约束都不可靠。
  /// 观测数已被 kMaxObservations 限制，全量扫描最多 120 对；结果按缓存复用，追加观测时失效。
  double parallaxDeg() const;

  /// 轨迹是否具备进入多帧约束的基本条件。
  bool readyForConstraint(int min_observations, double min_parallax_deg) const;

 private:
  /// 最大视差缓存，-1 表示待重建。
  mutable double max_parallax_cache_{-1.0};
};

}  // namespace superlivo_b
