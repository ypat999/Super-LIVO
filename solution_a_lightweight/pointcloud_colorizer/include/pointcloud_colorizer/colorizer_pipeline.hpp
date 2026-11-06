#pragma once

#include <vector>

#include <Eigen/Core>

#include "pointcloud_colorizer/image_sync.hpp"
#include "pointcloud_colorizer/occlusion_check.hpp"
#include "pointcloud_colorizer/pose_interpolation.hpp"
#include "pointcloud_colorizer/projection.hpp"
#include "pointcloud_colorizer/rgb_assignment.hpp"

namespace colorizer {

/// 彩色点云流水线：同步 -> 逐点位姿插值 -> 投影 -> 遮挡检查 -> RGB 赋值。
/// 与定位主干完全解耦，即使视觉定位关闭也能独立工作。
class ColorizerPipeline {
 public:
  struct Options {
    CameraIntrinsics intrinsics;
    Eigen::Matrix4d T_cl{Eigen::Matrix4d::Identity()};  ///< LiDAR -> Camera
    ImageSync::Options sync;
    OcclusionChecker::Options occlusion;
    RgbAssigner::Options rgb;
  };

  explicit ColorizerPipeline(const Options& options);

  /// 输入一帧点云（含每点时间戳）、图像缓存与位姿缓存，输出带色点云。
  std::vector<ColoredPoint> process(const std::vector<Eigen::Vector3f>& points_lidar,
                                    const std::vector<double>& point_stamps,
                                    const std::vector<TimedImage>& images,
                                    const std::vector<PoseSample>& poses) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace colorizer
