#include "super_livo_visual_b/estimator/unified_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <array>

// 协方差求逆需要 LU 模块的 Inverse 求值特化，否则运行期无限递归。
#include <Eigen/Geometry>
#include <Eigen/LU>

namespace superlivo_b {

namespace {
/// so(3) 指数映射。
Eigen::Matrix3d so3Exp(const Eigen::Vector3d& w) {
  const double theta = w.norm();
  if (theta < 1e-9) {
    Eigen::Matrix3d skew;
    skew << 0.0, -w.z(), w.y(),
            w.z(), 0.0, -w.x(),
            -w.y(), w.x(), 0.0;
    return Eigen::Matrix3d::Identity() + skew;
  }
  return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
}
}  // namespace

UnifiedEstimator::UnifiedEstimator(const Options& options) : options_(options) {
  filter_.state = State{};
  filter_.covariance = Eigen::MatrixXd::Identity(StateLayout::kSize, StateLayout::kSize) *
                       options.initial_covariance;
}

void UnifiedEstimator::propagate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,
                                 double dt) {
  if (dt <= 0.0) {
    return;
  }
  State& s = filter_.state;

  // 去掉零偏后的 IMU 测量。
  const Eigen::Vector3d omega = gyro - s.bg;
  const Eigen::Vector3d acc_world = s.R * (acc - s.ba) + s.g;

  // 简单欧拉积分（与 Super-LIO 的传播保持同一量级，具体精度由上层控制）。
  s.p += s.v * dt;
  s.v += acc_world * dt;
  s.R = s.R * so3Exp(omega * dt);

  // 过程噪声：只累加对角项，保证协方差正定且计算量可控。
  const int n = StateLayout::kSize;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
  q.segment<3>(StateLayout::kRot).setConstant(options_.gyro_noise * options_.gyro_noise * dt);
  q.segment<3>(StateLayout::kPos).setConstant(options_.acc_noise * options_.acc_noise * dt * dt);
  q.segment<3>(StateLayout::kVel).setConstant(options_.acc_noise * options_.acc_noise * dt);
  q.segment<3>(StateLayout::kBg).setConstant(options_.bias_gyro_noise * options_.bias_gyro_noise * dt);
  q.segment<3>(StateLayout::kBa).setConstant(options_.bias_acc_noise * options_.bias_acc_noise * dt);
  filter_.covariance.diagonal() += q;
}

void UnifiedEstimator::lidarUpdate(const Eigen::MatrixXd& H, const Eigen::VectorXd& b,
                                   double /*cost*/) {
  const int n = StateLayout::kSize;
  if (H.rows() != n || H.cols() != n || b.size() != n) {
    return;  // 维度不匹配，忽略本次更新
  }
  applyInformationUpdate(H, b, 1.0);
}

void UnifiedEstimator::visualUpdate(const VisualUpdateInput& input) {
  if (!options_.enable_visual) {
    return;
  }
  if (input.H.rows() != 6 || input.H.cols() != 6 || input.b.size() != 6) {
    return;  // 视觉法方程固定为 6 维相机位姿
  }

  const int n = StateLayout::kSize;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n);

  // 相机位姿 6 维（旋转 0..2, 平移 3..5）映射到统一状态的旋转块与位置块。
  // 说明：此处忽略 IMU-相机外参带来的耦合（外参变化很小），
  //       若后续把外参纳入更新，可在此扩展 block 映射。
  const std::array<int, 6> index = {StateLayout::kRot,     StateLayout::kRot + 1,
                                     StateLayout::kRot + 2, StateLayout::kPos,
                                     StateLayout::kPos + 1, StateLayout::kPos + 2};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      H(index[i], index[j]) += input.H(i, j);
    }
    b(index[i]) += input.b(i);
  }

  const double quality = std::min(1.0, std::max(0.1, input.quality));
  const double weight = options_.visual_weight * quality;
  applyInformationUpdate(H, b, weight);
}

void UnifiedEstimator::injectError(const Eigen::VectorXd& dx) {
  if (dx.size() != StateLayout::kSize) {
    return;
  }
  State& s = filter_.state;

  // 旋转用左扰动注入，其余为欧氏加法。
  s.R = so3Exp(dx.segment<3>(StateLayout::kRot)) * s.R;
  s.p += dx.segment<3>(StateLayout::kPos);
  s.v += dx.segment<3>(StateLayout::kVel);
  s.bg += dx.segment<3>(StateLayout::kBg);
  s.ba += dx.segment<3>(StateLayout::kBa);
  s.g += dx.segment<3>(StateLayout::kGrav);

  // 外参默认固定；仅在显式开启时更新（标定可靠时不建议在线估计）。
  if (options_.estimate_extrinsics) {
    const Eigen::Vector3d rot_ic = dx.segment<3>(StateLayout::kExtrinsic);
    const Eigen::Vector3d trans_ic = dx.segment<3>(StateLayout::kExtrinsic + 3);
    s.T_ic.block<3, 3>(0, 0) = so3Exp(rot_ic) * s.T_ic.block<3, 3>(0, 0);
    s.T_ic.block<3, 1>(0, 3) += trans_ic;

    const Eigen::Vector3d rot_il = dx.segment<3>(StateLayout::kExtrinsic + 6);
    const Eigen::Vector3d trans_il = dx.segment<3>(StateLayout::kExtrinsic + 9);
    s.T_il.block<3, 3>(0, 0) = so3Exp(rot_il) * s.T_il.block<3, 3>(0, 0);
    s.T_il.block<3, 1>(0, 3) += trans_il;
  }
}

const FilterState& UnifiedEstimator::filterState() const { return filter_; }

void UnifiedEstimator::resetState(const State& state) {
  filter_.state = state;
  filter_.covariance = Eigen::MatrixXd::Identity(StateLayout::kSize, StateLayout::kSize) *
                       options_.initial_covariance;
}

const UnifiedEstimator::Options& UnifiedEstimator::options() const { return options_; }

void UnifiedEstimator::applyInformationUpdate(const Eigen::MatrixXd& H,
                                              const Eigen::VectorXd& b,
                                              double weight) {
  if (weight <= 0.0) {
    return;
  }

  // 信息形式更新：P_new = (P^-1 + w·H)^-1, dx = -P_new·(w·b)
  const Eigen::MatrixXd information =
      filter_.covariance.inverse() + weight * H;
  const Eigen::MatrixXd covariance_new = information.inverse();
  if (!covariance_new.allFinite()) {
    return;
  }

  const Eigen::VectorXd dx = -covariance_new * (weight * b);
  if (!dx.allFinite()) {
    return;
  }

  // 保持协方差对称，避免数值漂移。
  filter_.covariance = 0.5 * (covariance_new + covariance_new.transpose());

  injectError(dx);
}

}  // namespace superlivo_b
