#pragma once

#include <algorithm>

#include <Eigen/Core>

namespace superlivo_common {

// ============================================================================
// 退化方向感知的各向异性位置先验
// ----------------------------------------------------------------------------
// 语义（A/B 完全一致）：LiDAR 退化时，沿退化方向 u 松开对 LIO 的信任（弱先验，
// 让视觉接管），垂直 u 方向收紧（强先验，视觉不许动），再按严重度与基线插值。
// 方案 A 用于 VisualUpdate 的法方程正则，方案 B 用于 buildLidarPrior。
// ============================================================================

/// info = (1 - sev)·base + sev·info_dir
/// info_dir = kd·(u·uᵀ) + kh·(I - u·uᵀ)，kd = 1/σ_degen²，kh = 1/σ_healthy²。
/// @param base          基线信息矩阵（通常为各向同性 1/σ_prior²·I）
/// @param severity      Super-LIO 退化严重度，0~1
/// @param dir_w         退化法向（世界系）；零向量表示无方向信息，不改变 base
/// @param sigma_healthy 垂直退化方向（LiDAR 看得住）的位置标准差
/// @param sigma_degen   沿退化方向的位置标准差
inline Eigen::Matrix3d directionalPositionInformation(
    const Eigen::Matrix3d& base, double severity,
    const Eigen::Vector3d& dir_w, double sigma_healthy, double sigma_degen) {
  const double sev = std::min(1.0, std::max(0.0, severity));
  if (sev <= 1e-6 || dir_w.norm() <= 1e-6) {
    return base;
  }
  const Eigen::Vector3d u = dir_w.normalized();
  const Eigen::Matrix3d Pu = u * u.transpose();
  const double kh = 1.0 / std::max(1e-12, sigma_healthy * sigma_healthy);
  const double kd = 1.0 / std::max(1e-12, sigma_degen * sigma_degen);
  const Eigen::Matrix3d info_dir =
      kd * Pu + kh * (Eigen::Matrix3d::Identity() - Pu);
  return (1.0 - sev) * base + sev * info_dir;
}

}  // namespace superlivo_common
