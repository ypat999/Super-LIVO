#pragma once

#include <cmath>

namespace superlivo_common {

// ============================================================================
// 2D-3D 重投影测量方差模型（像素²）
// ----------------------------------------------------------------------------
// 缺这一项等价于宣称每个特征都能把位姿约束到 1 px 且地图点是精确真值：
//   z≈4 m、40 个特征 → (f/z)²·40 ≈ 1e7 的信息量（~0.3 mm 等效精度），
//   一帧就把位置协方差压塌，之后 LiDAR 先验再也拉不动。
// 方案 A（ReprojectionConstraint）与方案 B（LidarVisualConstraint）必须同口径。
// ============================================================================

/// 像素噪声 ⊕ 锚点三维不确定度投影 (f·σ_anchor/z)²。
/// @param sigma_pix    像素噪声（px）
/// @param focal        焦距均值 0.5·(fx + fy)（px）
/// @param sigma_anchor 锚点三维不确定度（m）
/// @param inv_z        1 / 锚点深度（相机系 z，m）
inline double reprojectionVariance(double sigma_pix, double focal,
                                   double sigma_anchor, double inv_z) {
  const double anchor_pix = focal * sigma_anchor * inv_z;
  return sigma_pix * sigma_pix + anchor_pix * anchor_pix;
}

/// 栅格最近点关联的附加方差：锚点可能取到同一栅格里的另一个物理点，
/// 横向偏差最多 pixel_radius 像素，按均匀分布折算标准差 pixel_radius/√3。
inline double associationVariance(double pixel_radius) {
  const double assoc = pixel_radius / std::sqrt(3.0);
  return assoc * assoc;
}

}  // namespace superlivo_common
