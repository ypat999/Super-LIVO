#include "super_livo_visual_b/constraint/msckf_constraint.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

namespace superlivo_b {

namespace {
constexpr double kPi = 3.14159265358979323846;

Eigen::Matrix3d skew(const Eigen::Vector3d& p) {
  Eigen::Matrix3d m;
  m << 0.0, -p.z(), p.y(),
       p.z(), 0.0, -p.x(),
       -p.y(), p.x(), 0.0;
  return m;
}

double huberWeight(double norm, double delta) {
  if (delta <= 0.0 || norm <= delta) {
    return 1.0;
  }
  return delta / norm;
}
}  // namespace

MsckfConstraint::MsckfConstraint(const Options& options) : options_(options) {}

bool MsckfConstraint::triangulate(const FeatureTrack& track, Eigen::Vector3d& xyz) const {
  if (options_.fx <= 0.0 || options_.fy <= 0.0) {
    return false;
  }
  if (track.observations.size() < 2) {
    return false;
  }
  if (track.parallaxDeg() < options_.min_parallax_deg) {
    return false;  // 视差太小，三角化不稳定
  }

  const Eigen::Matrix3d K = (Eigen::Matrix3d() << options_.fx, 0.0, options_.cx, 0.0, options_.fy,
                             options_.cy, 0.0, 0.0, 1.0)
                                .finished();

  // 多视图线性三角化：每个观测贡献两个方程 x·P3 - P1 = 0, y·P3 - P2 = 0。
  const std::size_t n = track.observations.size();
  Eigen::MatrixXd A(static_cast<long>(2 * n), 4);
  for (std::size_t i = 0; i < n; ++i) {
    const auto& obs = track.observations[i];
    const Eigen::Matrix4d T_cw = obs.T_wc.inverse();
    Eigen::Matrix<double, 3, 4> P;
    P.block<3, 3>(0, 0) = K * T_cw.block<3, 3>(0, 0);
    P.block<3, 1>(0, 3) = K * T_cw.block<3, 1>(0, 3);

    const double x = obs.pixel.x();
    const double y = obs.pixel.y();
    A.row(static_cast<long>(2 * i)) = x * P.row(2) - P.row(0);
    A.row(static_cast<long>(2 * i + 1)) = y * P.row(2) - P.row(1);
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d X = svd.matrixV().col(3);
  if (std::fabs(X(3)) < 1e-9) {
    return false;  // 无穷远点
  }
  xyz = X.head<3>() / X(3);

  // 一致性检查：所有观测都应看到该点在相机前方。
  for (const auto& obs : track.observations) {
    const Eigen::Matrix4d T_cw = obs.T_wc.inverse();
    const Eigen::Vector3d p_c = T_cw.block<3, 3>(0, 0) * xyz + T_cw.block<3, 1>(0, 3);
    if (p_c.z() <= 1e-3) {
      return false;
    }
  }
  return true;
}

MultiFrameNormalEquations MsckfConstraint::build(const std::vector<FeatureTrack>& tracks) const {
  MultiFrameNormalEquations result;
  result.H = Eigen::MatrixXd::Zero(6, 6);
  result.b = Eigen::VectorXd::Zero(6);

  if (options_.fx <= 0.0 || options_.fy <= 0.0) {
    return result;
  }

  for (const auto& track : tracks) {
    if (static_cast<int>(track.observations.size()) < options_.min_observations) {
      continue;
    }

    // 只保留最近 max_track_length 次观测，避免轨迹过长导致数值问题。
    FeatureTrack trimmed;
    trimmed.id = track.id;
    trimmed.observations = track.observations;
    if (static_cast<int>(trimmed.observations.size()) > options_.max_track_length) {
      trimmed.observations.erase(
          trimmed.observations.begin(),
          trimmed.observations.end() - static_cast<long>(options_.max_track_length));
    }

    Eigen::Vector3d xyz_world = Eigen::Vector3d::Zero();
    if (!triangulate(trimmed, xyz_world)) {
      continue;
    }

    const std::size_t n = trimmed.observations.size();
    const long rows = static_cast<long>(2 * n);

    Eigen::MatrixXd J_x = Eigen::MatrixXd::Zero(rows, 6);  // 当前位姿（最后一个观测）
    Eigen::MatrixXd J_f = Eigen::MatrixXd::Zero(rows, 3);  // 特征位置
    Eigen::VectorXd r = Eigen::VectorXd::Zero(rows);

    bool valid = true;
    for (std::size_t i = 0; i < n; ++i) {
      const auto& obs = trimmed.observations[i];
      const Eigen::Matrix4d T_cw = obs.T_wc.inverse();
      const Eigen::Vector3d p_c = T_cw.block<3, 3>(0, 0) * xyz_world + T_cw.block<3, 1>(0, 3);
      if (p_c.z() <= 1e-3) {
        valid = false;
        break;
      }
      const double inv_z = 1.0 / p_c.z();
      const double u_pred = options_.fx * p_c.x() * inv_z + options_.cx;
      const double v_pred = options_.fy * p_c.y() * inv_z + options_.cy;

      Eigen::Vector2d residual(obs.pixel.x() - u_pred, obs.pixel.y() - v_pred);
      const double norm = residual.norm();
      const double weight = huberWeight(norm, options_.huber_delta);

      Eigen::Matrix<double, 2, 3> dpi_dpc;
      dpi_dpc << options_.fx * inv_z, 0.0, -options_.fx * p_c.x() * inv_z * inv_z,
                 0.0, options_.fy * inv_z, -options_.fy * p_c.y() * inv_z * inv_z;

      const Eigen::Matrix<double, 2, 3> dr_df = -dpi_dpc * T_cw.block<3, 3>(0, 0);

      r.segment<2>(static_cast<long>(2 * i)) = weight * residual;
      J_f.block<2, 3>(static_cast<long>(2 * i), 0) = weight * dr_df;

      // 只有最后一个观测（当前位姿）参与状态更新，历史位姿视为固定。
      if (i + 1 == n) {
        // 左扰动 δ = [φ; ρ]：∂P_C/∂φ = R_cw·[X_W]×，∂P_C/∂ρ = -R_cw
        Eigen::Matrix<double, 3, 6> dPc_ddelta;
        dPc_ddelta.block<3, 3>(0, 0) = T_cw.block<3, 3>(0, 0) * skew(xyz_world);
        dPc_ddelta.block<3, 3>(0, 3) = -T_cw.block<3, 3>(0, 0);
        J_x.block<2, 6>(static_cast<long>(2 * i), 0) = weight * (-dpi_dpc * dPc_ddelta);
      }
    }
    if (!valid) {
      continue;
    }

    Eigen::MatrixXd J_x_proj = J_x;
    Eigen::VectorXd r_proj = r;

    if (options_.use_nullspace_projection) {
      // 用 Householder QR 求 J_f 列空间的正交基 Q1，再把残差投影到其正交补。
      Eigen::HouseholderQR<Eigen::MatrixXd> qr(J_f);
      const Eigen::MatrixXd Q = qr.householderQ() * Eigen::MatrixXd::Identity(rows, rows);
      const Eigen::MatrixXd Q1 = Q.leftCols(3);  // 3 个基向量
      const Eigen::MatrixXd P = Eigen::MatrixXd::Identity(rows, rows) - Q1 * Q1.transpose();
      J_x_proj = P * J_x;
      r_proj = P * r;
    }

    result.H += J_x_proj.transpose() * J_x_proj;
    result.b += J_x_proj.transpose() * r_proj;
    result.cost += r_proj.squaredNorm();
    result.used_features += 1;
  }

  result.valid = result.used_features > 0;
  return result;
}

const MsckfConstraint::Options& MsckfConstraint::options() const { return options_; }

}  // namespace superlivo_b
