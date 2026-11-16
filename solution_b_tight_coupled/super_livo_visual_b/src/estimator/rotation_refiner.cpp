#include "super_livo_visual_b/estimator/rotation_refiner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace superlivo_b {

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Matrix3d expSo3(const Eigen::Vector3d& w) {
  const double theta = w.norm();
  if (theta < 1e-8) {
    return Eigen::Matrix3d::Identity() + skew(w);
  }
  return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
}

/// so(3) 对数映射（弧度）。与 P0-F 的 log_b 同口径：小角用 0.5 的一阶系数，
/// 不做 arccos 截断以外的处理（trace 截断只防数值越界，不改变小角精度）。
Eigen::Vector3d logSo3(const Eigen::Matrix3d& R) {
  const double trace = std::max(-1.0, std::min(3.0, R.trace()));
  const double theta = std::acos(std::max(-1.0, std::min(1.0, 0.5 * (trace - 1.0))));
  Eigen::Vector3d w(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  const double s = (theta < 1e-8) ? 0.5 : theta / (2.0 * std::sin(theta));
  return w * s;
}

/// so(3) 左 Jacobian：d log(Exp(a)Exp(p))/da |_{a=0}。
Eigen::Matrix3d leftJacobian(const Eigen::Vector3d& p) {
  const double theta = p.norm();
  const Eigen::Matrix3d K = skew(p);
  double a;
  double b;
  if (theta < 1e-8) {
    a = 0.5 - theta * theta / 24.0;
    b = (1.0 - theta * theta / 20.0) / 6.0;
  } else {
    a = (1.0 - std::cos(theta)) / (theta * theta);
    b = (theta - std::sin(theta)) / (theta * theta * theta);
  }
  return Eigen::Matrix3d::Identity() - a * K + b * K * K;
}

/// 单窗口问题：残差与解析 Jacobian 的组装。
/// 变量按帧主序展平：u 的第 i 帧占 [3i, 3i+3)。
/// 残差分三段：视觉（3m）→ 一阶先验（3n）→ 二阶差分（3(n−2)）。
struct Window {
  std::vector<Eigen::Matrix3d> Rp;     ///< n，LIO body→world
  std::vector<Eigen::Matrix3d> RvisT;  ///< m，Exp(r_vis)ᵀ
  std::vector<Eigen::Matrix3d> Cw;     ///< m，白化矩阵
  std::vector<int> fidx;               ///< m，因子右端帧的局部下标（>=1）
  std::vector<double> fw;              ///< m，Huber 权重
  double inv_sp = 0.0;                 ///< 1/(σp·D2R)
  double inv_sd = 0.0;                 ///< 1/(σd·D2R)
  int n = 0;
  int m = 0;
  int nres = 0;
  int nvar = 0;
  bool has_smooth = false;
};

/// 计算残差；jac 非空时同时填充解析 Jacobian（可由中心差分自检）。
void evalWindow(const Window& w, const Eigen::VectorXd& u, Eigen::VectorXd& res,
                Eigen::MatrixXd* jac, std::vector<double>* chi) {
  res = Eigen::VectorXd::Zero(w.nres);
  if (jac != nullptr) {
    jac->setZero(w.nres, w.nvar);
  }

  std::vector<Eigen::Matrix3d> Ri(w.n);
  for (int i = 0; i < w.n; ++i) {
    Ri[i] = expSo3(u.segment<3>(3 * i)) * w.Rp[i];
  }

  // 视觉块：r = log( Exp(r_vis)ᵀ · (R'_{i+1}ᵀ R'_i) )
  // fidx 存的是因子**右端**帧（区间 (i, i+1) 的终点 i+1），左端为 j−1。
  for (int s = 0; s < w.m; ++s) {
    const int j = w.fidx[s];
    const int i = j - 1;
    const Eigen::Matrix3d E = Ri[j].transpose() * Ri[i];
    const Eigen::Vector3d r = logSo3(w.RvisT[s] * E);
    const Eigen::Vector3d cr = w.Cw[s] * r;
    res.segment<3>(3 * s) = w.fw[s] * cr;
    if (chi != nullptr) {
      (*chi)[s] = cr.norm();
    }
    if (jac == nullptr) {
      continue;
    }
    // 左乘扰动 d_world 与加法参数 d_u 的关系 d_world = Jl(u)ᵀ d_u；
    // 区间旋转对左右端位姿的偏导符号相反（左端 +、右端 −）。
    const Eigen::Matrix3d K = leftJacobian(r).inverse() * Ri[i].transpose();
    const Eigen::Matrix3d M = w.fw[s] * (w.Cw[s] * K);
    jac->block<3, 3>(3 * s, 3 * i) = M * leftJacobian(u.segment<3>(3 * i)).transpose();
    jac->block<3, 3>(3 * s, 3 * j) = -M * leftJacobian(u.segment<3>(3 * j)).transpose();
  }

  // 一阶先验块：把修正量锚在 0（即锚在 LIO 基线）。
  const int off_p = 3 * w.m;
  for (int i = 0; i < w.n; ++i) {
    res.segment<3>(off_p + 3 * i) = w.inv_sp * u.segment<3>(3 * i);
    if (jac != nullptr) {
      jac->block<3, 3>(off_p + 3 * i, 3 * i).diagonal().setConstant(w.inv_sp);
    }
  }

  // 二阶差分块：约束修正量自身的时间平滑（不是角速度/角加速度）。
  if (w.has_smooth) {
    const int off_d = off_p + 3 * w.n;
    for (int i = 2; i < w.n; ++i) {
      const Eigen::Vector3d d2 = u.segment<3>(3 * i) - 2.0 * u.segment<3>(3 * (i - 1)) +
                                 u.segment<3>(3 * (i - 2));
      const int row = off_d + 3 * (i - 2);
      res.segment<3>(row) = w.inv_sd * d2;
      if (jac != nullptr) {
        jac->block<3, 3>(row, 3 * i).diagonal().setConstant(w.inv_sd);
        jac->block<3, 3>(row, 3 * (i - 1)).diagonal().setConstant(-2.0 * w.inv_sd);
        jac->block<3, 3>(row, 3 * (i - 2)).diagonal().setConstant(w.inv_sd);
      }
    }
  }
}

/// Levenberg–Marquardt（与 P0-F 的 _lm 同策略：对角缩放阻尼、7 次降步、失败即停）。
Eigen::VectorXd solveLm(const Window& w, Eigen::VectorXd u, int iters, double tol) {
  if (w.m == 0 && !w.has_smooth) {
    return Eigen::VectorXd::Zero(u.size());  // 只剩先验 ⇒ 解为 0
  }
  double lam = 1e-4;
  Eigen::VectorXd r;
  Eigen::MatrixXd J;
  evalWindow(w, u, r, &J, nullptr);
  double cost = 0.5 * r.squaredNorm();

  for (int it = 0; it < iters; ++it) {
    const Eigen::MatrixXd H = J.transpose() * J;
    const Eigen::VectorXd g = J.transpose() * r;
    const Eigen::VectorXd hd = H.diagonal();
    bool accepted = false;
    Eigen::VectorXd d = Eigen::VectorXd::Zero(u.size());
    for (int t = 0; t < 7; ++t) {
      Eigen::MatrixXd A = H;
      for (int i = 0; i < A.rows(); ++i) {
        A(i, i) += lam * std::max(hd[i], 1e-12) + 1e-12;
      }
      const Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
      if (ldlt.info() != Eigen::Success) {
        lam *= 10.0;
        continue;
      }
      d = ldlt.solve(-g);
      if (!d.allFinite()) {
        lam *= 10.0;
        continue;
      }
      Eigen::VectorXd rn;
      evalWindow(w, u + d, rn, nullptr, nullptr);
      const double cn = 0.5 * rn.squaredNorm();
      if (cn < cost) {
        u += d;
        cost = cn;
        lam = std::max(lam * 0.25, 1e-12);
        accepted = true;
        break;
      }
      lam *= 8.0;
    }
    if (!accepted || d.norm() < tol) {
      break;
    }
    evalWindow(w, u, r, &J, nullptr);
  }
  return u;
}

/// 解后硬投影（P0-F 的 S1/S2）：
///   S1 绝对钳位   ||u_k|| <= clip
///   S2 旋转 slew  ||u_k − u_{k−1}|| <= slew_factor · clip
/// 注意 slew 约束的是**修正量的一阶差分**，不是角速度；clip 只保证"这一帧往回收多少"
/// 有界，相邻帧仍可朝相反方向各收回 clip —— 这正是 L3（符号交替）档要检验的失效模式。
Eigen::Vector3d projectBounds(const Eigen::Vector3d& u, double clip_rad, double slew_rad,
                              bool has_prev, const Eigen::Vector3d& prev) {
  Eigen::Vector3d out = u;
  const double norm = out.norm();
  if (norm > clip_rad) {
    out *= clip_rad / norm;
  }
  if (has_prev) {
    const Eigen::Vector3d d = out - prev;
    const double dn = d.norm();
    if (dn > slew_rad) {
      out = prev + d * (slew_rad / dn);
    }
  }
  return out;
}

}  // namespace

Eigen::Matrix3d RotationRefiner::isotropicWhitening(double sigma_deg) {
  const double sigma = std::max(sigma_deg, 1e-6) * kDeg2Rad;
  return Eigen::Matrix3d::Identity() / sigma;
}

RotationRefiner::RotationRefiner() : RotationRefiner(Options{}) {}

RotationRefiner::RotationRefiner(const Options& options) : options_(options) {
  options_.window = std::max(1, options_.window);
  options_.irls_rounds = std::max(1, options_.irls_rounds);
  options_.max_iters = std::max(1, options_.max_iters);
}

void RotationRefiner::reset() {
  frames_.clear();
  last_correction_.setZero();
  corrected_rotation_.setIdentity();
  last_usable_ = false;
}

Eigen::Vector3d RotationRefiner::addFrame(const Eigen::Matrix3d& R_body,
                                          const Eigen::Vector3d& r_vis_body,
                                          const Eigen::Matrix3d& Cw, bool usable) {
  Frame frame;
  frame.R = R_body;
  frame.usable = usable;
  if (usable) {
    frame.RvisT = expSo3(r_vis_body).transpose();
    frame.Cw = Cw;
  }
  frames_.push_back(frame);
  while (static_cast<int>(frames_.size()) > options_.window) {
    frames_.pop_front();
  }

  const int n = static_cast<int>(frames_.size());
  Window w;
  w.n = n;
  w.nvar = 3 * n;
  w.Rp.resize(n);
  for (int i = 0; i < n; ++i) {
    w.Rp[i] = frames_[i].R;
  }
  for (int j = 1; j < n; ++j) {
    if (!frames_[j].usable) {
      continue;
    }
    w.fidx.push_back(j);
    w.RvisT.push_back(frames_[j].RvisT);
    w.Cw.push_back(frames_[j].Cw);
    w.fw.push_back(1.0);
  }
  w.m = static_cast<int>(w.fidx.size());
  w.inv_sp = 1.0 / (options_.sigma_prior_deg * kDeg2Rad);
  w.has_smooth = (options_.sigma_accel_deg > 0.0 && n >= 3);
  w.inv_sd = w.has_smooth ? 1.0 / (options_.sigma_accel_deg * kDeg2Rad) : 0.0;
  w.nres = 3 * w.m + 3 * n + (w.has_smooth ? 3 * (n - 2) : 0);

  // 热启动：用各帧已发布（投影后）的修正量，与 P0-F 的 hist 映射一致。
  Eigen::VectorXd u(w.nvar);
  for (int i = 0; i < n; ++i) {
    u.segment<3>(3 * i) = frames_[i].u;
  }

  for (int round = 0; round < options_.irls_rounds; ++round) {
    u = solveLm(w, u, options_.max_iters, options_.tol);
    if (round + 1 < options_.irls_rounds && w.m > 0) {
      std::vector<double> chi(w.m, 0.0);
      Eigen::VectorXd unused;
      evalWindow(w, u, unused, nullptr, &chi);
      for (int s = 0; s < w.m; ++s) {
        w.fw[s] = std::min(1.0, options_.huber_c / std::max(chi[s], 1e-9));
      }
    }
  }

  Eigen::Vector3d uk = u.tail<3>();
  if (options_.enable_bounds) {
    const bool has_prev = (n >= 2);
    const Eigen::Vector3d prev = has_prev ? frames_[n - 2].u : Eigen::Vector3d::Zero();
    uk = projectBounds(uk, options_.clip_deg * kDeg2Rad,
                       options_.slew_factor * options_.clip_deg * kDeg2Rad, has_prev, prev);
  }

  frames_.back().u = uk;
  last_correction_ = uk;
  last_usable_ = usable;
  corrected_rotation_ = expSo3(uk) * R_body;
  return uk;
}

}  // namespace superlivo_b
