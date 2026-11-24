#pragma once

#include <Eigen/Core>

namespace colorizer {

// ============================================================================
// 坐标变换与针孔投影（对应 gpt双方案设计.md §2.4）
// ----------------------------------------------------------------------------
// 完整投影链：P_W = T_WL(t_i)·P_L（运动补偿，由外部先用 pose_interpolation 完成）
//            P_C = T_CL·P_W
//            [u;v;1] ~ K·P_C
// 本类负责后半段（P_W -> 相机系 -> 像素），并做视野与正深度检查。
// ============================================================================

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
  bool in_fov{false};                          ///< 是否在图像视野内（含边界余量）
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
  double depth{0.0};                            ///< 相机系深度（用于遮挡判断）
};

/// 坐标变换与针孔投影。
class Projector {
 public:
  struct Options {
    int border_margin{2};  ///< 视野边界余量（像素），越小越严
  };

  explicit Projector(const CameraIntrinsics& intrinsics, const Options& options);

  /// 世界系点 -> 相机系：P_C = T_CL * P_W。
  /// @param T_cl       LiDAR -> Camera 外参
  /// @param point_world 已做过运动补偿的世界系点（不是原始 LiDAR 点）
  Eigen::Vector3d toCamera(const Eigen::Matrix4d& T_cl,
                           const Eigen::Vector3d& point_world) const;

  /// 相机系 -> 像素，含正深度与视野检查。
  ProjectionResult project(const Eigen::Vector3d& point_camera) const;

  /// 视锥粗筛：只做归一化坐标比较、不做除法，比 project() 便宜。
  /// 用途：LiDAR 360° 视场里通常只有约 1/5 的点落在相机画面内，先筛出这些点，
  /// 再对它们做像素投影与遮挡写入，画面外的点不参与任何逐像素运算。
  bool insideFrustum(const Eigen::Vector3d& point_camera) const;

  const CameraIntrinsics& intrinsics() const;

 private:
  CameraIntrinsics intrinsics_;
  Options options_;
  /// x/z、y/z 的允许区间（由内参与边界余量预计算，逐点比较时不需要除法）。
  double x_min_{0.0};
  double x_max_{0.0};
  double y_min_{0.0};
  double y_max_{0.0};
};

}  // namespace colorizer