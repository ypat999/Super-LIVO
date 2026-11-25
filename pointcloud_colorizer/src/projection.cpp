#include "pointcloud_colorizer/projection.hpp"

#include <limits>

namespace colorizer {

Projector::Projector(const CameraIntrinsics& intrinsics, const Options& options)
    : intrinsics_(intrinsics), options_(options) {
  const double fx = intrinsics_.fx > 1e-9 ? intrinsics_.fx : 1.0;
  const double fy = intrinsics_.fy > 1e-9 ? intrinsics_.fy : 1.0;
  const double m = static_cast<double>(options_.border_margin);
  if (intrinsics_.width > 0 && intrinsics_.height > 0) {
    // 与 project() 的视野判据严格同源：m <= u < width-m，m <= v < height-m。
    x_min_ = (m - intrinsics_.cx) / fx;
    x_max_ = (static_cast<double>(intrinsics_.width) - m - intrinsics_.cx) / fx;
    y_min_ = (m - intrinsics_.cy) / fy;
    y_max_ = (static_cast<double>(intrinsics_.height) - m - intrinsics_.cy) / fy;
  } else {
    // 内参还没配好时不裁剪，保持旧行为（交给 project() 判 in_fov）。
    x_min_ = y_min_ = -std::numeric_limits<double>::infinity();
    x_max_ = y_max_ = std::numeric_limits<double>::infinity();
  }
}

Eigen::Vector3d Projector::toCamera(const Eigen::Matrix4d& T_cl,
                                    const Eigen::Vector3d& point_world) const {
  // P_C = T_CL * P_W；P_W 已由外部按每点时间戳完成运动补偿。
  const Eigen::Vector3d p_cam = T_cl.block<3, 3>(0, 0) * point_world + T_cl.block<3, 1>(0, 3);
  return p_cam;
}

ProjectionResult Projector::project(const Eigen::Vector3d& point_camera) const {
  ProjectionResult result;

  // 只有正深度（相机前方）的点才可能被观测到。
  if (point_camera.z() <= 1e-6) {
    return result;
  }

  const double u = intrinsics_.fx * point_camera.x() / point_camera.z() + intrinsics_.cx;
  const double v = intrinsics_.fy * point_camera.y() / point_camera.z() + intrinsics_.cy;

  result.pixel = Eigen::Vector2d(u, v);
  result.depth = point_camera.z();

  // 视野检查：留出 border_margin 像素余量，避免边界处采样越界。
  const double m = static_cast<double>(options_.border_margin);
  result.in_fov = (u >= m && u < static_cast<double>(intrinsics_.width) - m &&
                   v >= m && v < static_cast<double>(intrinsics_.height) - m);
  return result;
}

bool Projector::insideFrustum(const Eigen::Vector3d& point_camera) const {
  const double z = point_camera.z();
  if (z <= 1e-6) {
    return false;  // 相机后方/零深度，不可能被观测到
  }
  // z > 0，故 x/z ∈ [x_min_, x_max_] 等价于 x ∈ [z·x_min_, z·x_max_]（乘法代替除法）。
  const double x = point_camera.x();
  const double y = point_camera.y();
  return x >= z * x_min_ && x <= z * x_max_ && y >= z * y_min_ && y <= z * y_max_;
}

const CameraIntrinsics& Projector::intrinsics() const { return intrinsics_; }

}  // namespace colorizer
