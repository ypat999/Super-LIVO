#include "pointcloud_colorizer/projection.hpp"

namespace colorizer {

Projector::Projector(const CameraIntrinsics& intrinsics, const Options& options)
    : intrinsics_(intrinsics), options_(options) {}

Eigen::Vector3d Projector::toCamera(const Eigen::Matrix4d& /*T_cl*/,
                                    const Eigen::Vector3d& /*point_lidar*/) const {
  // TODO(阶段 1): P_C = T_CL * T_LW(t_i) * P_L（外部先完成运动补偿）
  return Eigen::Vector3d::Zero();
}

ProjectionResult Projector::project(const Eigen::Vector3d& /*point_camera*/) const {
  // TODO(阶段 1): 正深度检查 + 针孔投影 + 图像边界检查
  return {};
}

const CameraIntrinsics& Projector::intrinsics() const { return intrinsics_; }

}  // namespace colorizer
