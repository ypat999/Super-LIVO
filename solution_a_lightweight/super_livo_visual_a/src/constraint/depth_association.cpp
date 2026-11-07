#include "super_livo_visual_a/constraint/depth_association.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

// 注意：Matrix4d::inverse() 的求值依赖 LU 模块提供的 Inverse 表达式特化。
// 若只包含 <Eigen/Core>，代码仍可编译，但运行期会退化为无限递归并栈溢出。
#include <Eigen/LU>

namespace superlivo_a {

namespace {
/// 像素栅格边长（像素），用于加速"投影点 -> 特征"的邻域查询。
constexpr int kCellSize = 8;

/// 取中位数（会拷贝并排序，规模很小，可接受）。
double medianOf(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<long>(mid), values.end());
  return values[mid];
}
}  // namespace

LidarDepthAssociation::LidarDepthAssociation(const Options& options) : options_(options) {}

void LidarDepthAssociation::setLocalMap(std::vector<Eigen::Vector3d> points_world) {
  local_map_ = std::move(points_world);
}

void LidarDepthAssociation::setSweep(std::vector<Eigen::Vector3d> points_world) {
  sweep_ = std::move(points_world);
}

std::vector<FeatureDepth> LidarDepthAssociation::associate(const std::vector<FeatureObs>& features,
                                                           const Eigen::Matrix4d& T_wc) const {
  ++frame_index_;
  std::vector<FeatureDepth> results;
  results.reserve(features.size());

  // 投影每个候选来源，建立"像素坐标 -> 点索引"的栅格。
  // 这里重新计算像素而不是解析 source 字符串（保持结构体语义清晰）。
  const Eigen::Matrix4d T_cw = T_wc.inverse();
  const Eigen::Matrix3d R_cw = T_cw.block<3, 3>(0, 0);
  const Eigen::Vector3d t_cw = T_cw.block<3, 1>(0, 3);

  // 用一个内部结构同时保存像素与三维点。
  struct Projected {
    double u{0.0};
    double v{0.0};
    double depth{0.0};
    Eigen::Vector3d xyz_world{Eigen::Vector3d::Zero()};
  };

  auto projectAll = [&](const std::vector<Eigen::Vector3d>& cloud, std::vector<Projected>& out) {
    out.clear();
    if (options_.fx <= 0.0) {
      return;
    }
    out.reserve(cloud.size());
    for (const auto& p_w : cloud) {
      const Eigen::Vector3d p_c = R_cw * p_w + t_cw;
      if (p_c.z() < options_.min_depth || p_c.z() > options_.max_depth) {
        continue;
      }
      const double u = options_.fx * p_c.x() / p_c.z() + options_.cx;
      const double v = options_.fy * p_c.y() / p_c.z() + options_.cy;
      if (options_.image_width > 0 && options_.image_height > 0 &&
          (u < 0.0 || v < 0.0 || u >= options_.image_width || v >= options_.image_height)) {
        continue;
      }
      out.push_back(Projected{u, v, p_c.z(), p_w});
    }
  };

  auto buildGrid = [](const std::vector<Projected>& pts) {
    std::unordered_map<long long, std::vector<int>> grid;
    grid.reserve(pts.size() * 2);
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const long long key =
          (static_cast<long long>(pts[i].u) / kCellSize) * 100000 +
          (static_cast<long long>(pts[i].v) / kCellSize);
      grid[key].push_back(static_cast<int>(i));
    }
    return grid;
  };

  std::vector<Projected> map_proj;
  std::vector<Projected> sweep_proj;
  projectAll(local_map_, map_proj);
  projectAll(sweep_, sweep_proj);
  const auto map_grid = buildGrid(map_proj);
  const auto sweep_grid = buildGrid(sweep_proj);

  const double pixel_radius = std::max(0.5, static_cast<double>(options_.pixel_radius));

  // 在给定栅格中查询某个特征附近的候选点。
  auto queryNeighbors = [&](const std::unordered_map<long long, std::vector<int>>& grid,
                            const std::vector<Projected>& pts, const FeatureObs& feature,
                            std::vector<const Projected*>& neighbors) {
    const long long cx = static_cast<long long>(feature.u) / kCellSize;
    const long long cy = static_cast<long long>(feature.v) / kCellSize;
    for (long long dx = -1; dx <= 1; ++dx) {
      for (long long dy = -1; dy <= 1; ++dy) {
        const long long key = (cx + dx) * 100000 + (cy + dy);
        auto it = grid.find(key);
        if (it == grid.end()) {
          continue;
        }
        for (int index : it->second) {
          const Projected& p = pts[index];
          const double du = p.u - feature.u;
          const double dv = p.v - feature.v;
          if (std::sqrt(du * du + dv * dv) <= pixel_radius) {
            neighbors.push_back(&p);
          }
        }
      }
    }
  };

  // 从候选集合中按"深度一致性 + 数量上限"求锚点。
  auto makeDepth = [&](std::vector<const Projected*>& neighbors, const std::string& source,
                       FeatureDepth& out) -> bool {
    if (neighbors.empty()) {
      return false;
    }
    std::vector<double> depths;
    depths.reserve(neighbors.size());
    for (const auto* n : neighbors) {
      depths.push_back(n->depth);
    }
    const double med = medianOf(depths);

    // 只保留与中位数深度一致的近邻（过滤跨越前后景的候选）。
    std::vector<const Projected*> kept;
    for (const auto* n : neighbors) {
      if (std::fabs(n->depth - med) <= options_.search_radius) {
        kept.push_back(n);
      }
    }
    if (kept.empty()) {
      return false;
    }
    if (static_cast<int>(kept.size()) > options_.max_neighbors) {
      std::partial_sort(kept.begin(), kept.begin() + options_.max_neighbors, kept.end(),
                        [&](const Projected* a, const Projected* b) {
                          return std::fabs(a->depth - med) < std::fabs(b->depth - med);
                        });
      kept.resize(static_cast<std::size_t>(options_.max_neighbors));
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    double depth_sum = 0.0;
    for (const auto* n : kept) {
      sum += n->xyz_world;
      depth_sum += n->depth;
    }
    const double count = static_cast<double>(kept.size());
    out.xyz_world = sum / count;
    out.depth = depth_sum / count;
    out.valid = true;
    out.source = source;
    return true;
  };

  for (const auto& feature : features) {
    FeatureDepth fd;
    fd.id = feature.id;

    std::vector<const Projected*> neighbors;
    neighbors.reserve(16);

    // 来源 1：OctVox 局部地图（主来源）
    queryNeighbors(map_grid, map_proj, feature, neighbors);
    bool ok = makeDepth(neighbors, "local_map", fd);

    // 来源 2：当前扫描（局部地图稀疏时兜底）
    if (!ok) {
      neighbors.clear();
      queryNeighbors(sweep_grid, sweep_proj, feature, neighbors);
      ok = makeDepth(neighbors, "sweep", fd);
    }

    // 来源 3：最近 anchor_window 帧内的锚点缓存（不依赖全局地图，gpt §2.2）
    if (!ok) {
      auto it = recent_anchors_.find(feature.id);
      if (it != recent_anchors_.end() &&
          (frame_index_ - it->second.frame) <= static_cast<long>(options_.anchor_window)) {
        const Eigen::Vector3d p_c = R_cw * it->second.xyz_world + t_cw;
        if (p_c.z() >= options_.min_depth && p_c.z() <= options_.max_depth) {
          fd.xyz_world = it->second.xyz_world;
          fd.depth = p_c.z();
          fd.valid = true;
          fd.source = "recent_anchor";
          ok = true;
        }
      }
    }

    if (ok) {
      recent_anchors_[feature.id] = CachedAnchor{fd.xyz_world, frame_index_};
    }
    results.push_back(fd);
  }

  // 清理过期的锚点缓存。
  for (auto it = recent_anchors_.begin(); it != recent_anchors_.end();) {
    if (frame_index_ - it->second.frame > static_cast<long>(options_.anchor_window)) {
      it = recent_anchors_.erase(it);
    } else {
      ++it;
    }
  }

  return results;
}

const LidarDepthAssociation::Options& LidarDepthAssociation::options() const { return options_; }

}  // namespace superlivo_a
