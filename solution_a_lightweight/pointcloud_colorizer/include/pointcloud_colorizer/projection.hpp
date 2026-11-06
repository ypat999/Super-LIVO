#pragma once

#include <Eigen/Core>

namespace colorizer {

/// 相机内参。
struct CameraIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};
};

/// 投影结果。
struct ProjectionResult {
  bool in_fov{false};
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
  double depth{0.0};
};

/// 坐标变换与针孔投影。
/// P_C = T_CL * T_LW(t_i) * P_L，pixel = pi(K * P_C)
class Projector {
 public:
  struct Options {
    int border_margin{2};
  };

  explicit Projector(const CameraIntrinsics& intrinsics, const Options& options);

  /// LiDAR 点 -> 相机系（T_cl 为 LiDAR 到相机的外参）。
  Eigen::Vector3d toCamera(const Eigen::Matrix4d& T_cl,
                           const Eigen::Vector3d& point_lidar) const;

  /// 相机系 -> 像素，含视野与正深度检查。
  ProjectionResult project(const Eigen::Vector3d& point_camera) const;

  const CameraIntrinsics& intrinsics() const;

 private:
  CameraIntrinsics intrinsics_;
  Options options_;
};

}  // namespace colorizer
