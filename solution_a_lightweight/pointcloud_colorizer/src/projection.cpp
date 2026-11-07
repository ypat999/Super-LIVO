#include "pointcloud_colorizer/projection.hpp"

namespace colorizer {

Projector::Projector(const CameraIntrinsics& intrinsics, const Options& options)
    : intrinsics_(intrinsics), options_(options) {}

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

const CameraIntrinsics& Projector::intrinsics() const { return intrinsics_; }

}  // namespace colorizer
