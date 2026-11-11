#include "super_livo_visual_a/update/visual_update.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>

namespace superlivo_a {

VisualUpdate::VisualUpdate(const Options& options) : options_(options) {}

VisualUpdateOutput VisualUpdate::apply(const VisualUpdateInput& input,
                                       const GateDecision& decision) const {
  VisualUpdateOutput output;

  // 总开关关闭，或门控判定视觉不参与更新。
  if (!options_.enable) {
    return output;
  }
  if (decision.mode == VisualMode::kTrackingOnly || decision.mode == VisualMode::kRejected) {
    return output;
  }
  if (decision.visual_weight <= 0.0) {
    return output;
  }

  // 输入合法性检查。
  if (input.H.rows() <= 0 || input.H.rows() != input.H.cols() ||
      input.b.size() != input.H.rows()) {
    return output;
  }

  // 权重 = 门控权重 * 视觉质量（质量限制在 [0.1, 1.0]，避免完全失效）。
  const double quality = std::min(1.0, std::max(0.1, input.quality));
  const double weight = decision.visual_weight * quality;

  Eigen::MatrixXd H = weight * input.H;
  Eigen::VectorXd b = weight * input.b;

  // LiDAR 先验正则：把"当前位姿即 LiDAR 位姿"当成一个信息量为
  // diag(1/σ_rot², 1/σ_pos²) 的高斯先验（残差为 0）叠加进来。
  // 没有它，dx 就是无正则最小二乘解，门控权重 w 会被左右同时约掉、
  // 视觉一旦接受就一步全量接管位姿。有了它，w 才真正插值两种传感器。
  if (H.rows() >= 6) {
    const double kp_rot = 1.0 / std::max(1e-12, options_.prior_rot_sigma *
                                                    options_.prior_rot_sigma);
    const double kp_pos = 1.0 / std::max(1e-12, options_.prior_pos_sigma *
                                                    options_.prior_pos_sigma);
    for (int i = 0; i < 3; ++i) {
      H(i, i) += kp_rot;
    }

    // 位置先验：健康场景各向同性；退化时沿退化方向松开、垂直方向收紧。
    Eigen::Matrix3d info_pos = Eigen::Matrix3d::Identity() * kp_pos;
    const double sev = std::min(1.0, std::max(0.0, input.degen_severity));
    if (options_.directional_prior_en && sev > 1e-6 &&
        input.degen_dir_w.norm() > 1e-6) {
      const Eigen::Vector3d u = input.degen_dir_w.normalized();
      const Eigen::Matrix3d Pu = u * u.transpose();
      const double kh = 1.0 / std::max(1e-12, options_.prior_pos_sigma_healthy *
                                                options_.prior_pos_sigma_healthy);
      const double kd = 1.0 / std::max(1e-12, options_.prior_pos_sigma_degen *
                                                options_.prior_pos_sigma_degen);
      // 沿 u：kd（弱先验，视觉可接管）；垂直 u：kh（强先验，视觉不许动）
      const Eigen::Matrix3d info_dir =
          kd * Pu + kh * (Eigen::Matrix3d::Identity() - Pu);
      info_pos = (1.0 - sev) * info_pos + sev * info_dir;
    }
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        H(3 + i, 3 + j) += info_pos(i, j);
      }
    }
  }

  // 阻尼项：防止退化方向上的病态求解。
  const double damping = 1e-6 * std::max(1.0, H.diagonal().cwiseAbs().maxCoeff());
  H.diagonal().array() += damping;

  // 求解 H * dx = -b（法方程形式）。
  Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
  if (ldlt.info() != Eigen::Success) {
    return output;
  }
  const Eigen::VectorXd dx = ldlt.solve(-b);
  if (ldlt.info() != Eigen::Success || !dx.allFinite()) {
    return output;
  }

  // 增量过大视为异常（可能来自错误匹配），拒绝本次更新。
  if (dx.norm() > options_.max_dx_norm) {
    return output;
  }

  output.accepted = true;
  output.dx = dx;
  output.cost = input.cost;
  return output;
}

const VisualUpdate::Options& VisualUpdate::options() const { return options_; }

}  // namespace superlivo_a
