#pragma once

#include <vector>

#include <Eigen/Core>

#include "pointcloud_colorizer/image_sync.hpp"
#include "pointcloud_colorizer/occlusion_check.hpp"
#include "pointcloud_colorizer/pose_interpolation.hpp"
#include "pointcloud_colorizer/projection.hpp"
#include "pointcloud_colorizer/rgb_assignment.hpp"

namespace colorizer {

// ============================================================================
// 彩色点云流水线（对应 gpt双方案设计.md §2.4 / §六.2）
// ----------------------------------------------------------------------------
// 目标：为每个 LiDAR 点赋予可靠的图像颜色，输出彩色点云。与视觉定位完全解耦，
//   即使视觉定位关闭也能独立工作（gpt §六.2 强调彩色点云与定位主干不强耦合）。
//
// 处理流程（阶段 1 实现）：
//   1. ImageSync             ：在图像缓存中挑选与扫描窗口对齐的一帧（曝光中点优先）
//   2. LidarPoseInterpolator ：按每点时间戳插值位姿（运动补偿，点云非瞬时完成）
//   3. Projector             ：P_W = T_WL(t_i)·P_L -> P_C = T_CL·P_W -> 针孔投影
//   4. OcclusionChecker      ：视野/深度/遮挡与深度边缘检查
//   5. RgbAssigner           ：从图像采样 RGB，并维护颜色观测质量
//
// 注意：与方案 B 共用本包，方案 B 直接复用，不做重复实现。
// ============================================================================

/// 彩色点云流水线：同步 -> 逐点位姿插值 -> 投影 -> 遮挡检查 -> RGB 赋值。
class ColorizerPipeline {
 public:
  struct Options {
    CameraIntrinsics intrinsics;                          ///< 相机内参
    Eigen::Matrix4d T_cl{Eigen::Matrix4d::Identity()};    ///< LiDAR -> Camera 外参
    ImageSync::Options sync;
    OcclusionChecker::Options occlusion;
    RgbAssigner::Options rgb;
    /// 位姿插值允许的最大相邻采样间隔（秒）。与 sync.max_offset（图像同步窗口）
    /// 是两件不同的事：/lio/odom 通常 10 Hz，用图像窗口反过来限制位姿间隔会让
    /// 所有点都因"采样间隔过大"被跳过（全部 quality=0）。
    double pose_max_gap{0.5};
  };

  explicit ColorizerPipeline(const Options& options);

  /// 输入一帧点云（含每点时间戳）、图像缓存与位姿缓存，输出带色点云。
  /// @param points_lidar 原始 LiDAR 点（LiDAR 系）
  /// @param point_stamps 与 points_lidar 一一对应的每点采集时间戳
  /// @param images       候选图像缓存（含曝光中点时间戳）
  /// @param poses        LiDAR 位姿采样缓存（用于运动补偿插值）
  std::vector<ColoredPoint> process(const std::vector<Eigen::Vector3f>& points_lidar,
                                    const std::vector<double>& point_stamps,
                                    const std::vector<TimedImage>& images,
                                    const std::vector<PoseSample>& poses) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace colorizer