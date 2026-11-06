#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "super_livo_visual_a/frontend/klt_frontend.hpp"

namespace superlivo_a {

/// 由 LiDAR 提供的特征深度。
struct FeatureDepth {
  int id{0};
  Eigen::Vector3d xyz_world{Eigen::Vector3d::Zero()};
  double depth{0.0};
  bool valid{false};
  std::string source{"none"};  ///< "local_map" | "sweep" | "none"
};

/// 将图像特征与当前 LiDAR 扫描/局部 OctVox 地图关联，得到三维锚点。
/// 这是方案 A 的深度来源：不恢复视觉尺度，只借用 LiDAR 几何。
class LidarDepthAssociation {
 public:
  struct Options {
    double search_radius{0.3};   ///< 像素邻域对应的三维搜索半径
    double min_depth{0.5};
    double max_depth{80.0};
    int max_neighbors{5};
  };

  explicit LidarDepthAssociation(const Options& options);

  /// 对每个特征在局部地图中搜索近邻点，取深度一致的近邻作为锚点。
  std::vector<FeatureDepth> associate(const std::vector<FeatureObs>& features,
                                      const Eigen::Matrix4d& T_wc) const;

  const Options& options() const;

 private:
  Options options_;
  // TODO: 对 OctVoxMap / 当前扫描的邻域查询接口
};

}  // namespace superlivo_a
