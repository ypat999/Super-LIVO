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
