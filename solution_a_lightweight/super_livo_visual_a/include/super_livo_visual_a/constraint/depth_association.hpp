#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "super_livo_visual_a/frontend/klt_frontend.hpp"

namespace superlivo_a {

// ============================================================================
// 方案 A · 深度关联（对应 gpt双方案设计.md §2.2）
// ----------------------------------------------------------------------------
// 把图像特征与 LiDAR 几何关联，得到三维锚点。深度来源优先级：
//   1. "local_map"     ：外部注入的 OctVox 局部地图点（主来源）
//   2. "sweep"         ：外部注入的当前 LiDAR 扫描点（局部地图稀疏时兜底）
//   3. "recent_anchor" ：最近 anchor_window 帧成功关联的锚点缓存（不依赖全局地图）
//
// 关联方式：把三维点投影到图像，按像素邻域（pixel_radius）收集候选，
// 用深度中位数 + search_radius 做一致性过滤，再取平均得到锚点。
// ============================================================================

/// 由 LiDAR 提供的特征深度。
struct FeatureDepth {
  int id{0};
  Eigen::Vector3d xyz_world{Eigen::Vector3d::Zero()};  ///< 世界系三维锚点
  double depth{0.0};                                   ///< 相机系沿光轴深度
  bool valid{false};
  std::string source{"none"};  ///< "local_map" | "sweep" | "recent_anchor" | "none"
};

/// 图像特征 <-> LiDAR 几何的深度关联。
class LidarDepthAssociation {
 public:
  struct Options {
    double fx{0.0};  ///< 相机内参（必须设置）
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    int image_width{0};
    int image_height{0};
    int pixel_radius{3};        ///< 像素邻域半径
    double search_radius{0.3};  ///< 深度一致性容差（米）
    double min_depth{0.5};
    double max_depth{80.0};
    int max_neighbors{5};
    int anchor_window{5};       ///< 最近锚点缓存帧数
  };

  explicit LidarDepthAssociation(const Options& options);

  /// 注入 OctVox 局部地图点（世界系）。每个 LiDAR 帧更新一次即可。
  void setLocalMap(std::vector<Eigen::Vector3d> points_world);

  /// 注入当前 LiDAR 扫描点（世界系），作为局部地图稀疏时的兜底。
  void setSweep(std::vector<Eigen::Vector3d> points_world);

  /// 对每个特征建立三维锚点。
  std::vector<FeatureDepth> associate(const std::vector<FeatureObs>& features,
                                      const Eigen::Matrix4d& T_wc) const;

  const Options& options() const;

 private:
  Options options_;
  std::vector<Eigen::Vector3d> local_map_;
  std::vector<Eigen::Vector3d> sweep_;

  /// 最近锚点缓存：feature_id -> (锚点, 帧号)。
  struct CachedAnchor {
    Eigen::Vector3d xyz_world{Eigen::Vector3d::Zero()};
    long frame{0};
  };
  mutable std::unordered_map<int, CachedAnchor> recent_anchors_;
  mutable long frame_index_{0};
};

}  // namespace superlivo_a
