#include "super_livo_visual_a/constraint/reprojection_constraint.hpp"

#include <cmath>

// Matrix4d::inverse() 需要 LU 模块的 Inverse 求值特化，否则运行期无限递归。
#include <Eigen/LU>

#include "super_livo_common/measurement_noise.hpp"

namespace superlivo_a {

namespace {
/// 反对称矩阵（李代数 so(3) 的 hat 算子）。
Eigen::Matrix3d skew(const Eigen::Vector3d& p) {
  Eigen::Matrix3d m;
  m << 0.0, -p.z(), p.y(),
       p.z(), 0.0, -p.x(),
       -p.y(), p.x(), 0.0;
  return m;
}

/// Huber 核权重：小残差保持二次，大残差降为线性。
double huberWeight(double norm, double delta) {
  if (delta <= 0.0 || norm <= delta) {
    return 1.0;
  }
  return delta / norm;
}
}  // namespace

ReprojectionConstraint::ReprojectionConstraint(const Options& options) : options_(options) {}

std::vector<VisualResidualBlock> ReprojectionConstraint::build(
    const std::vector<FeatureObs>& features,
    const std::vector<FeatureDepth>& depths,
    const Eigen::Matrix4d& T_wc) const {
  std::vector<VisualResidualBlock> blocks;
  if (options_.fx <= 0.0 || options_.fy <= 0.0) {
    return blocks;  // 内参未配置
  }

  // 残差按特征 id 配对（depth 与 feature 顺序可能不一致）。
  auto findDepth = [&depths](int id) -> const FeatureDepth* {
    for (const auto& d : depths) {
      if (d.id == id) {
        return d.valid ? &d : nullptr;
      }
    }
    return nullptr;
  };

  // T_CW = T_WC^{-1}
  const Eigen::Matrix4d T_cw = T_wc.inverse();
  const Eigen::Matrix3d R_cw = T_cw.block<3, 3>(0, 0);
  const Eigen::Vector3d t_cw = T_cw.block<3, 1>(0, 3);

  blocks.reserve(features.size());
  for (const auto& feature : features) {
    const FeatureDepth* depth = findDepth(feature.id);
    if (depth == nullptr) {
      continue;
    }

    // 世界点 -> 相机系
    const Eigen::Vector3d p_c = R_cw * depth->xyz_world + t_cw;
    if (p_c.z() <= 1e-3) {
      continue;
    }

    // 预测像素
    const double inv_z = 1.0 / p_c.z();
    const double u_pred = options_.fx * p_c.x() * inv_z + options_.cx;
    const double v_pred = options_.fy * p_c.y() * inv_z + options_.cy;

    // 残差 r = z - pi(...)
    const Eigen::Vector2d residual(feature.u - u_pred, feature.v - v_pred);
    const double pixel_error = residual.norm();
    if (pixel_error > options_.max_pixel_error) {
      continue;  // 外点（可能来自错误深度关联）
    }

    // d(pi)/d(P_C)
    Eigen::Matrix<double, 2, 3> dpi_dpc;
    dpi_dpc << options_.fx * inv_z, 0.0, -options_.fx * p_c.x() * inv_z * inv_z,
               0.0, options_.fy * inv_z, -options_.fy * p_c.y() * inv_z * inv_z;

    // 对相机位姿左扰动 δ = [φ; ρ]（旋转在前、平移在后）：
    //   ∂P_C/∂φ = R_cw·[P_W]×，∂P_C/∂ρ = -R_cw
    // 故 dr/dδ = -dpi_dPc · [∂P_C/∂φ, ∂P_C/∂ρ]
    Eigen::Matrix<double, 3, 6> dPc_ddelta;
    dPc_ddelta.block<3, 3>(0, 0) = R_cw * skew(depth->xyz_world);
    dPc_ddelta.block<3, 3>(0, 3) = -R_cw;
    const Eigen::Matrix<double, 2, 6> jacobian = -dpi_dpc * dPc_ddelta;

    VisualResidualBlock block;
    block.id = feature.id;
    block.residual = residual;
    block.jacobian = jacobian;
    block.weight = huberWeight(pixel_error, options_.huber_delta);
    // 测量方差：像素噪声 ⊕ 深度关联锚点不确定度投影（f·σ_anchor/z）。
    // 缺这一项等价于宣称每个特征能把位姿约束到 1 px（z=5 m 处仅 0.8 cm），
    // 80 个特征叠加后位姿"精度"达毫米级，视觉会盖过 LiDAR 先验、把位姿
    // 直接拉到拟合地图上，实测表现为修正量忽大忽小、不可信。
    const double f_mean = 0.5 * (options_.fx + options_.fy);
    block.variance = superlivo_common::reprojectionVariance(
        options_.sigma_pix, f_mean, options_.sigma_anchor, inv_z);
    block.valid = true;
    blocks.push_back(block);
  }

  // 约束太少时不参与更新，避免病态解。
  if (static_cast<int>(blocks.size()) < options_.min_constraints) {
    return {};
  }
  return blocks;
}

void ReprojectionConstraint::accumulate(const std::vector<VisualResidualBlock>& blocks,
                                        Eigen::MatrixXd& H,
                                        Eigen::VectorXd& b,
                                        double& cost) const {
  constexpr int kDim = 6;  // 相机位姿 se3 维度
  H = Eigen::MatrixXd::Zero(kDim, kDim);
  b = Eigen::VectorXd::Zero(kDim);
  cost = 0.0;

  for (const auto& block : blocks) {
    if (!block.valid) {
      continue;
    }
    // 注意：jacobian 是 dr/dδ，法方程使用 J^T R^-1 J 与 J^T R^-1 r。
    // Huber 权重仍在像素空间计算（语义不变），噪声归一化单独按 1/variance 施加。
    const double w = block.weight / std::max(1e-9, block.variance);
    H += w * block.jacobian.transpose() * block.jacobian;
    b += w * block.jacobian.transpose() * block.residual;
    cost += block.weight * block.residual.squaredNorm();
  }
}

const ReprojectionConstraint::Options& ReprojectionConstraint::options() const { return options_; }

}  // namespace superlivo_a
