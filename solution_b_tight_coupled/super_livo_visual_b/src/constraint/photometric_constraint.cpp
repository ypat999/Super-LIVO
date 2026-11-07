#include "super_livo_visual_b/constraint/photometric_constraint.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

// Matrix4d::inverse() 需要 LU 模块的 Inverse 求值特化，否则运行期无限递归。
#include <Eigen/LU>

namespace superlivo_b {

namespace {
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

/// 双线性采样灰度（越界返回 -1 表示无效）。
double sampleGray(const cv::Mat& gray, double u, double v) {
  if (u < 0.0 || v < 0.0 || u >= gray.cols - 1 || v >= gray.rows - 1) {
    return -1.0;
  }
  const int x0 = static_cast<int>(std::floor(u));
  const int y0 = static_cast<int>(std::floor(v));
  const double ax = u - x0;
  const double ay = v - y0;
  const double v00 = gray.at<uchar>(y0, x0);
  const double v10 = gray.at<uchar>(y0, x0 + 1);
  const double v01 = gray.at<uchar>(y0 + 1, x0);
  const double v11 = gray.at<uchar>(y0 + 1, x0 + 1);
  return (1.0 - ay) * ((1.0 - ax) * v00 + ax * v10) + ay * ((1.0 - ax) * v01 + ax * v11);
}
}  // namespace

PhotometricConstraint::PhotometricConstraint(const Options& options) : options_(options) {}

std::vector<PhotometricResidual> PhotometricConstraint::build(
    const cv::Mat& image,
    const std::vector<FeatureTrack>& tracks) const {
  std::vector<PhotometricResidual> residuals;
  if (!options_.enable || image.empty() || options_.fx <= 0.0) {
    return residuals;
  }

  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }
  if (gray.type() != CV_8UC1) {
    cv::Mat converted;
    gray.convertTo(converted, CV_8UC1);
    gray = converted;
  }

  // 图像梯度（用于把像素位移误差转成灰度误差）。
  cv::Mat grad_x;
  cv::Mat grad_y;
  cv::Sobel(gray, grad_x, CV_32F, 1, 0, 3);
  cv::Sobel(gray, grad_y, CV_32F, 0, 1, 3);

  residuals.reserve(tracks.size());
  for (const auto& track : tracks) {
    if (track.observations.empty() || (!track.triangulated && !track.valid)) {
      continue;
    }
    const FeatureObservation& last = track.observations.back();
    const Eigen::Matrix4d T_cw = last.T_wc.inverse();
    const Eigen::Vector3d p_c = T_cw.block<3, 3>(0, 0) * track.xyz_world +
                                T_cw.block<3, 1>(0, 3);
    if (p_c.z() <= 1e-3) {
      continue;
    }
    const double inv_z = 1.0 / p_c.z();
    const double u_pred = options_.fx * p_c.x() * inv_z + options_.cx;
    const double v_pred = options_.fy * p_c.y() * inv_z + options_.cy;

    const double intensity_pred = sampleGray(gray, u_pred, v_pred);
    const double intensity_obs = sampleGray(gray, last.pixel.x(), last.pixel.y());
    if (intensity_pred < 0.0 || intensity_obs < 0.0) {
      continue;  // 越界
    }

    const double residual = intensity_pred - intensity_obs;
    if (std::fabs(residual) > options_.max_intensity_error) {
      continue;  // 外点（遮挡/纹理变化）
    }

    // 图像梯度（按双线性位置附近取整数像素梯度即可）。
    const int gx = std::min(std::max(static_cast<int>(std::lround(u_pred)), 0), gray.cols - 1);
    const int gy = std::min(std::max(static_cast<int>(std::lround(v_pred)), 0), gray.rows - 1);
    const Eigen::RowVector2d grad(grad_x.at<float>(gy, gx), grad_y.at<float>(gy, gx));

    Eigen::Matrix<double, 2, 3> dpi_dpc;
    dpi_dpc << options_.fx * inv_z, 0.0, -options_.fx * p_c.x() * inv_z * inv_z,
               0.0, options_.fy * inv_z, -options_.fy * p_c.y() * inv_z * inv_z;

    // 左扰动 δ = [φ; ρ]：∂P_C/∂φ = R_cw·[X_W]×，∂P_C/∂ρ = -R_cw
    Eigen::Matrix<double, 3, 6> dPc_ddelta;
    dPc_ddelta.block<3, 3>(0, 0) = T_cw.block<3, 3>(0, 0) * skew(track.xyz_world);
    dPc_ddelta.block<3, 3>(0, 3) = -T_cw.block<3, 3>(0, 0);

    PhotometricResidual out;
    out.feature_id = track.id;
    out.residual = residual;
    // r = I(π) - I_obs，故 dr/dδ = grad · (∂π/∂δ)。
    out.jacobian = grad * (dpi_dpc * dPc_ddelta);
    out.weight = huberWeight(std::fabs(residual), options_.huber_delta);
    out.valid = true;
    residuals.push_back(out);
  }
  return residuals;
}

const PhotometricConstraint::Options& PhotometricConstraint::options() const { return options_; }

}  // namespace superlivo_b
