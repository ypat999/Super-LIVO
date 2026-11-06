#include "super_livo_visual_a/constraint/depth_association.hpp"

namespace superlivo_a {

LidarDepthAssociation::LidarDepthAssociation(const Options& options) : options_(options) {}

std::vector<FeatureDepth> LidarDepthAssociation::associate(const std::vector<FeatureObs>& /*features*/,
                                                          const Eigen::Matrix4d& /*T_wc*/) const {
  // TODO(阶段 3): 在当前扫描/OctVox 局部地图中做邻域查询，按深度一致性赋予三维锚点；
  //               保留最近 3~5 帧的局部锚点，避免完全依赖已扭曲的全局地图。
  return {};
}

const LidarDepthAssociation::Options& LidarDepthAssociation::options() const { return options_; }

}  // namespace superlivo_a
