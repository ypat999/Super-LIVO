#include "super_livo_visual_b/estimator/rotation_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace superlivo_b {

namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Matrix3d expSo3(const Eigen::Vector3d& w) {
  const double theta = w.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity() + skew(w);
  }
  return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
}

/// 投影回 SO(3)：外参/拟合矩阵只有 ~1e-6 的正交精度，log/exp 前必须归一。
Eigen::Matrix3d projectSo3(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  q.normalize();
  return q.toRotationMatrix();
}

/// 正交 Procrustes：min Σ‖x2 − R x1‖²（det=+1）。X1/X2 为 3×N。
Eigen::Matrix3d procrustes(const Eigen::MatrixXd& X1, const Eigen::MatrixXd& X2) {
  const Eigen::Matrix3d C = X2 * X1.transpose();
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
  D(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
  return projectSo3(svd.matrixV() * D * svd.matrixU().transpose());
}

double median(std::vector<double> v) {
  if (v.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
  return (v.size() % 2 == 1) ? v[mid] : 0.5 * (v[mid - 1] + v[mid]);
}

/// numpy 默认（linear）分位数。
double percentile(std::vector<double> v, double q) {
  if (v.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(v.begin(), v.end());
  const double pos = q * static_cast<double>(v.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
  const std::size_t hi = std::min(lo + 1, v.size() - 1);
  const double t = pos - static_cast<double>(lo);
  return v[lo] * (1.0 - t) + v[hi] * t;
}

/// 最小二乘 q = argmin ‖X·q − lam‖（X 为 3×N，按 idx 取子集）。
Eigen::Vector3d solvePlane(const Eigen::MatrixXd& X, const Eigen::VectorXd& lam,
                           const std::vector<int>& idx) {
  Eigen::MatrixXd Xs(3, static_cast<int>(idx.size()));
  Eigen::VectorXd ls(static_cast<int>(idx.size()));
  for (std::size_t k = 0; k < idx.size(); ++k) {
    Xs.col(static_cast<int>(k)) = X.col(idx[k]);
    ls[static_cast<int>(k)] = lam[idx[k]];
  }
  const Eigen::Matrix3d A = Xs * Xs.transpose();
  const Eigen::Vector3d b = Xs * ls;
  return A.ldlt().solve(b);
}

/// IRLS 拟合 λ = q·x1（主导平面，迭代内点收缩），对应 Python fit_plane()。
Eigen::Vector3d fitPlane(const Eigen::MatrixXd& X, const Eigen::VectorXd& lam,
                         int iters, double k) {
  std::vector<int> good;
  for (int i = 0; i < X.cols(); ++i) {
    if (std::isfinite(lam[i])) {
      good.push_back(i);
    }
  }
  if (good.size() < 3) {
    return Eigen::Vector3d(0.0, 0.0, 0.2);
  }
  Eigen::Vector3d q = solvePlane(X, lam, good);
  for (int it = 0; it < iters; ++it) {
    std::vector<double> r;
    r.reserve(good.size());
    for (const int i : good) {
      r.push_back(lam[i] - X.col(i).dot(q));
    }
    const double med = median(r);
    std::vector<double> dev;
    dev.reserve(r.size());
    for (const double v : r) {
      dev.push_back(std::abs(v - med));
    }
    const double s = 1.4826 * median(dev) + 1e-12;
    std::vector<int> next;
    for (const int i : good) {
      if (std::abs(lam[i] - X.col(i).dot(q)) < k * s) {
        next.push_back(i);
      }
    }
    if (next.size() < 8) {
      break;
    }
    q = solvePlane(X, lam, next);
    good = std::move(next);
  }
  return q;
}

/// 像素 -> 单位视线（3×N，列为 x_i）。
Eigen::MatrixXd bearing(const RotationEstimator::Options& opt,
                        const std::vector<Eigen::Vector2d>& pts) {
  Eigen::MatrixXd X(3, static_cast<int>(pts.size()));
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const Eigen::Vector3d x((pts[i].x() - opt.cx) / opt.fx,
                            (pts[i].y() - opt.cy) / opt.fy, 1.0);
    X.col(static_cast<int>(i)) = x / x.norm();
  }
  return X;
}

/// 模型残差与解析 Jacobian。
/// e_i = ŷ_i − x2_i，ŷ = normalize(R x1_i + t (q·x1_i))
/// ∂e/∂δθ = −P [R x1]× ,  ∂e/∂q = P t x1ᵀ ,  P = (I − ŷŷᵀ)/‖u‖
void evalModel(const Eigen::VectorXd& p, const Eigen::Matrix3d& Ri,
               const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y,
               const Eigen::Vector3d& t, Eigen::VectorXd& res, Eigen::MatrixXd* jac) {
  const Eigen::Matrix3d R = expSo3(p.head<3>()) * Ri;
  const Eigen::Vector3d q = p.tail<3>();
  const int n = static_cast<int>(X.cols());
  res = Eigen::VectorXd::Zero(3 * n);
  if (jac != nullptr) {
    jac->setZero(3 * n, 6);
  }
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d x1 = X.col(i);
    const Eigen::Vector3d rx = R * x1;
    const Eigen::Vector3d u = rx + t * q.dot(x1);
    const double nu = std::max(u.norm(), 1e-12);
    const Eigen::Vector3d y = u / nu;
    res.segment<3>(3 * i) = y - Y.col(i);
    if (jac == nullptr) {
      continue;
    }
    const Eigen::Matrix3d P =
        (Eigen::Matrix3d::Identity() - y * y.transpose()) / nu;
    Eigen::Matrix<double, 3, 6> Jp;
    Jp.leftCols<3>() = P * (-skew(rx));
    Jp.rightCols<3>() = P * (t * x1.transpose());
    jac->block<3, 6>(3 * i, 0) = Jp;
  }
}

/// soft_l1 损失：Σ 2f²(√(1+(e/f)²)−1)（scipy 口径）。
double robustCost(const Eigen::VectorXd& e, double f_scale) {
  double c = 0.0;
  for (int i = 0; i < e.size(); ++i) {
    const double z = e[i] / f_scale;
    c += 2.0 * f_scale * f_scale * (std::sqrt(1.0 + z * z) - 1.0);
  }
  return c;
}

/// 阻尼 LM（缩放坐标 + 逐分量 IRLS 权重）。
/// @param robust true 用 soft_l1 权/代价（f_scale），false 用普通最小二乘
void solveLm(const Eigen::Matrix3d& Ri, const Eigen::MatrixXd& X,
             const Eigen::MatrixXd& Y, const Eigen::Vector3d& t, bool robust,
             double f_scale, int max_iters, Eigen::VectorXd& p) {
  auto costOf = [&](const Eigen::VectorXd& pp) {
    Eigen::VectorXd e;
    evalModel(pp, Ri, X, Y, t, e, nullptr);
    return robust ? robustCost(e, f_scale) : 0.5 * e.squaredNorm();
  };

  double cost = costOf(p);
  double damp = 1e-4;
  Eigen::VectorXd step = Eigen::VectorXd::Zero(6);
  for (int it = 0; it < max_iters; ++it) {
    Eigen::VectorXd e;
    Eigen::MatrixXd J;
    evalModel(p, Ri, X, Y, t, e, &J);
    Eigen::VectorXd ew = e;
    Eigen::MatrixXd Jw = J;
    if (robust) {
      for (int i = 0; i < e.size(); ++i) {
        const double z = e[i] / f_scale;
        const double w = 1.0 / std::sqrt(1.0 + z * z);
        ew[i] *= w;
        Jw.row(i) *= w;
      }
    }
    Eigen::Matrix<double, 6, 1> d;
    for (int j = 0; j < 6; ++j) {
      d[j] = std::max(Jw.col(j).norm(), 1e-12);
    }
    Eigen::MatrixXd A = Jw;
    for (int j = 0; j < 6; ++j) {
      A.col(j) /= d[j];
    }
    Eigen::Matrix<double, 6, 6> H = A.transpose() * A;
    const Eigen::Matrix<double, 6, 1> g = A.transpose() * ew;
    bool accepted = false;
    for (int trial = 0; trial < 10; ++trial) {
      Eigen::Matrix<double, 6, 6> Hd = H;
      Hd.diagonal().array() += damp + 1e-12;
      const Eigen::Matrix<double, 6, 1> dz = Hd.ldlt().solve(-g);
      if (!dz.allFinite()) {
        damp *= 8.0;
        continue;
      }
      step = dz.cwiseQuotient(d);
      const double cn = costOf(p + step);
      if (cn < cost) {
        p += step;
        cost = cn;
        damp = std::max(damp * 0.25, 1e-12);
        accepted = true;
        break;
      }
      damp *= 8.0;
    }
    if (!accepted || step.norm() < 1e-10) {
      break;
    }
  }
}

Eigen::Matrix<double, 6, 6> pseudoInverse(const Eigen::Matrix<double, 6, 6>& H) {
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H);
  if (es.info() != Eigen::Success) {
    return Eigen::Matrix<double, 6, 6>::Zero();
  }
  const Eigen::Matrix<double, 6, 1> ev = es.eigenvalues();
  const double max_ev = ev.cwiseAbs().maxCoeff();
  Eigen::Matrix<double, 6, 1> inv = Eigen::Matrix<double, 6, 1>::Zero();
  for (int i = 0; i < 6; ++i) {
    if (ev[i] > 1e-12 * std::max(max_ev, 1e-300)) {
      inv[i] = 1.0 / ev[i];
    }
  }
  return es.eigenvectors() * inv.asDiagonal() * es.eigenvectors().transpose();
}

}  // namespace

RotationEstimator::Result RotationEstimator::estimate(
    const std::vector<Eigen::Vector2d>& a, const std::vector<Eigen::Vector2d>& b,
    const Eigen::Vector3d& t_c) const {
  Result out;
  const int n = static_cast<int>(std::min(a.size(), b.size()));
  out.n_pairs = n;
  out.t_norm = t_c.norm();
  if (n < std::max(8, options_.min_inl)) {
    out.reason = "too_few_pairs";
    return out;
  }
  if (!t_c.allFinite()) {
    out.reason = "no_t";
    return out;
  }

  const Eigen::Vector3d t = t_c;
  const Eigen::MatrixXd X1 = bearing(options_, a);
  const Eigen::MatrixXd X2 = bearing(options_, b);

  // ---- 外点门：单应 RANSAC（单应本身含视差项，比纯旋转模型更适合当门）----
  std::vector<uchar> mask;
  {
    std::vector<cv::Point2f> pa(static_cast<std::size_t>(n));
    std::vector<cv::Point2f> pb(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
      pa[static_cast<std::size_t>(i)] =
          cv::Point2f(static_cast<float>(a[i].x()), static_cast<float>(a[i].y()));
      pb[static_cast<std::size_t>(i)] =
          cv::Point2f(static_cast<float>(b[i].x()), static_cast<float>(b[i].y()));
    }
    const cv::Mat Hm = cv::findHomography(pa, pb, cv::RANSAC, options_.seed_px, mask);
    int inl = 0;
    for (const uchar v : mask) {
      inl += (v != 0) ? 1 : 0;
    }
    if (Hm.empty() || static_cast<int>(mask.size()) != n || inl < options_.min_inl) {
      mask.assign(static_cast<std::size_t>(n), 1);
    }
  }

  std::vector<int> idx;
  idx.reserve(mask.size());
  for (int i = 0; i < n; ++i) {
    if (mask[static_cast<std::size_t>(i)] != 0) {
      idx.push_back(i);
    }
  }
  const int m = static_cast<int>(idx.size());
  Eigen::MatrixXd Xm(3, m);
  Eigen::MatrixXd Ym(3, m);
  for (int k = 0; k < m; ++k) {
    Xm.col(k) = X1.col(idx[k]);
    Ym.col(k) = X2.col(idx[k]);
  }

  // ---- 交替初值：λ ← 闭式；q ← IRLS 主导平面；R ← 加权 Procrustes ----
  Eigen::Matrix3d R = procrustes(Xm, Ym);
  Eigen::Vector3d q(0.0, 0.0, 0.2);
  for (int round = 0; round < options_.rounds; ++round) {
    Eigen::VectorXd lam(m);
    std::vector<int> ok;
    for (int i = 0; i < m; ++i) {
      const Eigen::Vector3d x2 = Ym.col(i);
      const Eigen::Vector3d c1 = x2.cross(R * Xm.col(i));
      const Eigen::Vector3d c2 = x2.cross(t);
      const double den = c2.squaredNorm();
      lam[i] = std::numeric_limits<double>::quiet_NaN();
      if (den > 1e-10) {
        const double v = -c1.dot(c2) / den;
        if (std::isfinite(v) && v > 5e-4 && v < 2.0) {
          lam[i] = v;
          ok.push_back(i);
        }
      }
    }
    if (static_cast<int>(ok.size()) < options_.min_inl) {
      break;
    }
    Eigen::MatrixXd Xo(3, static_cast<int>(ok.size()));
    Eigen::VectorXd lo(static_cast<int>(ok.size()));
    for (std::size_t k = 0; k < ok.size(); ++k) {
      Xo.col(static_cast<int>(k)) = Xm.col(ok[k]);
      lo[static_cast<int>(k)] = lam[ok[k]];
    }
    q = fitPlane(Xo, lo, 4, 2.5);

    const Eigen::VectorXd lam_full = Xm.transpose() * q;
    Eigen::MatrixXd Tgt(3, m);
    for (int i = 0; i < m; ++i) {
      const Eigen::Vector3d u = R * Xm.col(i) + t * lam_full[i];
      const double rho = u.dot(Ym.col(i));
      Tgt.col(i) = rho * Ym.col(i) - t * lam_full[i];
    }
    const Eigen::Matrix3d Rn = procrustes(Xm, Tgt);
    if (!Rn.allFinite()) {
      break;
    }
    const double conv = Eigen::AngleAxisd(Rn.transpose() * R).angle();
    R = Rn;
    if (conv < 1e-6) {
      break;
    }
  }
  if (!q.allFinite()) {
    q = Eigen::Vector3d(0.0, 0.0, 0.2);
  }

  // ---- 鲁棒 LM（soft_l1），再在保留点上做普通最小二乘取协方差 ----
  Eigen::VectorXd p(6);
  p << 0.0, 0.0, 0.0, q.x(), q.y(), q.z();
  solveLm(R, Xm, Ym, t, true, options_.f_scale, options_.max_iters, p);

  std::vector<double> ei(m, 0.0);
  {
    Eigen::VectorXd e;
    evalModel(p, R, Xm, Ym, t, e, nullptr);
    for (int i = 0; i < m; ++i) {
      ei[static_cast<std::size_t>(i)] = e.segment<3>(3 * i).norm();
    }
  }
  const double med_e = median(ei);
  const double thresh = std::max(4.0 * options_.f_scale, 3.0 * 1.4826 * med_e);
  std::vector<int> keep;
  for (int i = 0; i < m; ++i) {
    if (ei[static_cast<std::size_t>(i)] < thresh) {
      keep.push_back(i);
    }
  }
  if (static_cast<int>(keep.size()) < options_.min_inl) {
    const double p70 = percentile(ei, 0.7);
    keep.clear();
    for (int i = 0; i < m; ++i) {
      if (ei[static_cast<std::size_t>(i)] <= p70) {
        keep.push_back(i);
      }
    }
  }
  if (static_cast<int>(keep.size()) < 3) {
    out.reason = "too_few_inliers";
    return out;
  }

  Eigen::MatrixXd Xk(3, static_cast<int>(keep.size()));
  Eigen::MatrixXd Yk(3, static_cast<int>(keep.size()));
  for (std::size_t k = 0; k < keep.size(); ++k) {
    Xk.col(static_cast<int>(k)) = Xm.col(keep[k]);
    Yk.col(static_cast<int>(k)) = Ym.col(keep[k]);
  }
  solveLm(R, Xk, Yk, t, false, options_.f_scale, options_.max_iters, p);

  Eigen::VectorXd ek;
  Eigen::MatrixXd J;
  evalModel(p, R, Xk, Yk, t, ek, &J);
  const Eigen::Matrix3d Rf = projectSo3(expSo3(p.head<3>()) * R);
  const int dof = std::max(1, 2 * static_cast<int>(keep.size()) - 6);
  const Eigen::Matrix<double, 6, 6> cov6 =
      (ek.squaredNorm() / static_cast<double>(dof)) * pseudoInverse(J.transpose() * J);
  const Eigen::Matrix3d C3 = 0.5 * (cov6.topLeftCorner<3, 3>() +
                                    cov6.topLeftCorner<3, 3>().transpose());

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> esC(C3);
  const Eigen::Vector3d ev = esC.eigenvalues();  // 升序
  out.sigma_rad = ev.cwiseMax(0.0).cwiseSqrt();

  // 诊断量：内点重投影误差与中位深度
  const Eigen::Vector3d qf = p.tail<3>();
  std::vector<double> px;
  std::vector<double> depth;
  px.reserve(keep.size());
  depth.reserve(keep.size());
  for (int k = 0; k < static_cast<int>(keep.size()); ++k) {
    const Eigen::Vector3d x1 = Xk.col(k);
    const double lam = qf.dot(x1);
    const Eigen::Vector3d u = Rf * x1 + t * lam;
    const double uz = std::max(u.z(), 1e-9);
    const double pu = options_.fx * u.x() / uz + options_.cx;
    const double pv = options_.fy * u.y() / uz + options_.cy;
    const Eigen::Vector2d pb = b[static_cast<std::size_t>(idx[keep[k]])];
    px.push_back(std::hypot(pu - pb.x(), pv - pb.y()));
    if (lam > 1e-6) {
      depth.push_back(1.0 / lam);
    }
  }

  out.R = Rf;
  out.q = qf;
  out.cov = C3;
  out.th_deg = Eigen::AngleAxisd(Rf).angle() * 180.0 / 3.14159265358979323846;
  out.n_inl = static_cast<int>(keep.size());
  out.px_med = median(px);
  out.px_p90 = percentile(px, 0.9);
  out.d_med = median(depth);
  out.usable = true;
  if (out.t_norm < options_.min_baseline) {
    out.usable = false;
    out.reason = "baseline_too_short";
  }
  return out;
}

}  // namespace superlivo_b
