#include "pointcloud_colorizer/colorizer_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace colorizer {

namespace {
/// 一个通过视野粗筛的点（只保留投影所需的三个量，避免逐点保存 4x4/3D 中间量）。
struct FrustumHit {
  std::size_t index{0};          ///< 在输入点云中的下标
  Eigen::Vector2d pixel{0.0, 0.0};
  double depth{0.0};
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
  occlusion.beginFrame(options_.intrinsics.width, options_.intrinsics.height);

  // 输出先按"全部点、无色"铺好：落在画面外的点后面不会再被碰，
  // 只对画面内的点回填颜色（雷达点稀疏，画面外占大多数）。
  output.resize(points_lidar.size());
  for (std::size_t i = 0; i < points_lidar.size(); ++i) {
    output[i].xyz = points_lidar[i];
    output[i].rgb.setZero();
    output[i].quality = 0.0F;
  }

  // ---- 第 2 步：视野粗筛 + 运动补偿，只对画面内的点算像素与深度 -----------
  std::vector<FrustumHit> hits;
  hits.reserve(points_lidar.size() / 4 + 16);

  // 同一帧点云通常共用同一时刻（单线程调用方传入的就是 point_stamps 全等），
  // 逐点位姿插值（含 slerp）曾经是这一阶段的主要开销，这里按时间戳缓存复用。
  double cached_stamp = std::numeric_limits<double>::quiet_NaN();
  bool cached_ok = false;
  Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();

  for (std::size_t i = 0; i < points_lidar.size(); ++i) {
    if (point_stamps[i] != cached_stamp) {
      cached_stamp = point_stamps[i];
      cached_ok = interpolator.poseAt(cached_stamp, T_wl);
    }
    if (!cached_ok) {
      continue;  // 该时刻没有可靠位姿，跳过（quality 保持 0）
    }

    // P_W = T_WL(t_i) * P_L：用该点自己的采集时刻做运动补偿。
    const Eigen::Vector3d p_l(points_lidar[i].x(), points_lidar[i].y(), points_lidar[i].z());
    const Eigen::Vector3d p_w = T_wl.block<3, 3>(0, 0) * p_l + T_wl.block<3, 1>(0, 3);

    // P_C = T_CL * P_W
    const Eigen::Vector3d p_c = projector.toCamera(options_.T_cl, p_w);

    // 先用归一化坐标粗筛（无除法）：画面外的点直接跳过，不做除法/栅格写入。
    if (!projector.insideFrustum(p_c)) {
      continue;
    }
    const ProjectionResult projection = projector.project(p_c);
    if (!projection.in_fov) {
      continue;
    }
    hits.push_back(FrustumHit{i, projection.pixel, projection.depth});
  }

  // ---- 第 3 步：把画面内点的深度写入 z-buffer ------------------------------
  for (const FrustumHit& hit : hits) {
    occlusion.insertDepth(hit.pixel, hit.depth);
  }

  // ---- 第 4 步：遮挡/边缘判定 + RGB 采样（只遍历画面内的点） --------------
  const RgbAssigner assigner(options_.rgb);
  for (const FrustumHit& hit : hits) {
    ProjectionResult projection;
    projection.in_fov = true;
    projection.pixel = hit.pixel;
    projection.depth = hit.depth;

    if (!occlusion.isVisible(projection, hit.depth) || occlusion.isDepthEdge(projection)) {
      continue;
    }

    // 质量评估：越靠近图像边界、越远越不可靠。
    const double u = hit.pixel.x();
    const double v = hit.pixel.y();
    const double margin =
        std::min(std::min(u, options_.intrinsics.width - u),
                 std::min(v, options_.intrinsics.height - v));
    const double border_factor = std::min(1.0, std::max(0.0, margin / 20.0));
    const double depth_factor = std::min(1.0, 1.0 / (1.0 + hit.depth / 50.0));
    const float quality = static_cast<float>(std::max(0.0, border_factor * depth_factor));

    ColoredPoint& point = output[hit.index];
    if (!assigner.assign(image.image, hit.pixel, quality, point)) {
      point.rgb.setZero();
      point.quality = 0.0F;  // 未能着色：保留几何，颜色标记为无效
    }
  }

  return output;
}

const ColorizerPipeline::Options& ColorizerPipeline::options() const { return options_; }

}  // namespace colorizer
