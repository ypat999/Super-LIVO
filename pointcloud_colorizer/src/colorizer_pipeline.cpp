#include "pointcloud_colorizer/colorizer_pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace colorizer {

namespace {
/// 单个点在流水线中间阶段的计算结果。
struct PointProjection {
  Eigen::Vector3d point_world{Eigen::Vector3d::Zero()};
  ProjectionResult projection;
  bool pose_ok{false};
};
}  // namespace

ColorizerPipeline::ColorizerPipeline(const Options& options) : options_(options) {}

std::vector<ColoredPoint> ColorizerPipeline::process(
    const std::vector<Eigen::Vector3f>& points_lidar,
    const std::vector<double>& point_stamps,
    const std::vector<TimedImage>& images,
    const std::vector<PoseSample>& poses) const {
  std::vector<ColoredPoint> output;
  if (points_lidar.empty() || point_stamps.size() != points_lidar.size()) {
    return output;
  }

  // ---- 第 0 步：图像与扫描窗口时间对齐 --------------------------------------
  // 用点时间戳的范围作为扫描窗口。
  const auto [min_it, max_it] = std::minmax_element(point_stamps.begin(), point_stamps.end());
  const ImageSync sync(options_.sync);
  const TimedImage image = sync.match(images, *min_it, *max_it);
  if (!image.valid || image.image.empty()) {
    return output;  // 没有可用图像，整帧跳过
  }

  // ---- 第 1 步：构造逐点位姿插值器 -----------------------------------------
  LidarPoseInterpolator interpolator(LidarPoseInterpolator::Options{options_.pose_max_gap});
  for (const auto& pose : poses) {
    interpolator.addSample(pose);
  }

  const Projector projector(options_.intrinsics, Projector::Options{});
  OcclusionChecker occlusion(options_.occlusion);

  // ---- 第 2 步：逐点运动补偿 + 投影，并把深度写入 z-buffer -----------------
  std::vector<PointProjection> projected(points_lidar.size());
  occlusion.beginFrame(options_.intrinsics.width, options_.intrinsics.height);

  for (std::size_t i = 0; i < points_lidar.size(); ++i) {
    Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
    if (!interpolator.poseAt(point_stamps[i], T_wl)) {
      continue;  // 该时刻没有可靠位姿，跳过（quality 保持 0）
    }

    // P_W = T_WL(t_i) * P_L：用该点自己的采集时刻做运动补偿。
    const Eigen::Vector3d p_l(points_lidar[i].x(), points_lidar[i].y(), points_lidar[i].z());
    const Eigen::Vector3d p_w = T_wl.block<3, 3>(0, 0) * p_l + T_wl.block<3, 1>(0, 3);

    // P_C = T_CL * P_W
    const Eigen::Vector3d p_c = projector.toCamera(options_.T_cl, p_w);

    PointProjection item;
    item.point_world = p_w;
    item.projection = projector.project(p_c);
    item.pose_ok = true;
    projected[i] = item;

    if (item.projection.in_fov) {
      occlusion.insertDepth(item.projection.pixel, item.projection.depth);
    }
  }

  // ---- 第 3 步：遮挡/边缘判定 + RGB 采样 ----------------------------------
  const RgbAssigner assigner(options_.rgb);
  output.reserve(points_lidar.size());

  for (std::size_t i = 0; i < points_lidar.size(); ++i) {
    ColoredPoint point;
    point.xyz = points_lidar[i];

    const PointProjection& item = projected[i];
    bool colored = false;

    if (item.pose_ok && item.projection.in_fov &&
        occlusion.isVisible(item.projection, item.projection.depth) &&
        !occlusion.isDepthEdge(item.projection)) {
      // 质量评估：越靠近图像边界、越远越不可靠。
      const double u = item.projection.pixel.x();
      const double v = item.projection.pixel.y();
      const double margin =
          std::min(std::min(u, options_.intrinsics.width - u),
                   std::min(v, options_.intrinsics.height - v));
      const double border_factor = std::min(1.0, std::max(0.0, margin / 20.0));
      const double depth_factor = std::min(1.0, 1.0 / (1.0 + item.projection.depth / 50.0));
      const float quality = static_cast<float>(std::max(0.0, border_factor * depth_factor));
      colored = assigner.assign(image.image, item.projection.pixel, quality, point);
    }

    if (!colored) {
      point.rgb = Eigen::Vector3f::Zero();
      point.quality = 0.0F;  // 未能着色：保留几何，颜色标记为无效
    }
    output.push_back(point);
  }

  return output;
}

const ColorizerPipeline::Options& ColorizerPipeline::options() const { return options_; }

}  // namespace colorizer
