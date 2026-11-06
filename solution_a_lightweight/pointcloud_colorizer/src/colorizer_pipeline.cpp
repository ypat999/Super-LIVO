#include "pointcloud_colorizer/colorizer_pipeline.hpp"

namespace colorizer {

ColorizerPipeline::ColorizerPipeline(const Options& options) : options_(options) {}

std::vector<ColoredPoint> ColorizerPipeline::process(
    const std::vector<Eigen::Vector3f>& /*points_lidar*/,
    const std::vector<double>& /*point_stamps*/,
    const std::vector<TimedImage>& /*images*/,
    const std::vector<PoseSample>& /*poses*/) const {
  // TODO(阶段 1): 串联 ImageSync -> LidarPoseInterpolator -> Projector
  //               -> OcclusionChecker -> RgbAssigner，逐点生成颜色
  return {};
}

const ColorizerPipeline::Options& ColorizerPipeline::options() const { return options_; }

}  // namespace colorizer
