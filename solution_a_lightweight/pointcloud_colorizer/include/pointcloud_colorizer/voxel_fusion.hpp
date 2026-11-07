#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace colorizer {

/// 体素级多视图颜色融合。
/// 背景：同一面被 10~30 帧重复观测，若逐帧 append 进地图，同一位置会以不同
/// 颜色并存多个点——这正是彩色"盐噪声"的来源（几何离群只差 30%，颜色噪声差 3.5 倍）。
/// 官方 FAST-LIVO2 的做法是每个位置只保留一个地图点、颜色被多帧光度残差持续刷新；
/// 这里用体素加权均值近似：权重 = 该帧着色 quality（已含边界×深度因子），
/// 位置取体素质心（顺带轻微平滑几何）。
class VoxelColorFusion {
 public:
  explicit VoxelColorFusion(double voxel_size = 0.0) : voxel_size_(voxel_size) {}

  bool enabled() const { return voxel_size_ > 0.0; }
  void set_voxel_size(double vs) { voxel_size_ = vs; }
  double voxel_size() const { return voxel_size_; }
  std::size_t size() const { return cells_.size(); }
  void clear() { cells_.clear(); }

  /// 插入一次观测。quality<=0 的未上色点应被调用方过滤，这里再兜底一次。
  void insert(const Eigen::Vector3f& p_world, const Eigen::Vector3f& rgb, float quality) {
    const double w = static_cast<double>(std::max(0.0F, quality));
    if (w <= 0.0) {
      return;
    }
    Cell& c = cells_[key(p_world)];
    c.sum_pos += p_world.cast<double>();
    c.sum_rgb += rgb.cast<double>() * w;
    c.sum_w += w;
    ++c.count;
  }

  /// 遍历融合结果：fn(const Eigen::Vector3d& pos, const Eigen::Vector3d& rgb)。
  template <typename Fn>
  void forEach(Fn&& fn) const {
    for (const auto& kv : cells_) {
      const Cell& c = kv.second;
      if (c.sum_w <= 0.0 || c.count == 0) {
        continue;
      }
      fn(c.sum_pos / static_cast<double>(c.count), c.sum_rgb / c.sum_w);
    }
  }

 private:
  struct Cell {
    Eigen::Vector3d sum_pos{0.0, 0.0, 0.0};
    Eigen::Vector3d sum_rgb{0.0, 0.0, 0.0};
    double sum_w{0.0};
    std::uint32_t count{0};
  };

  /// 体素索引 -> 64 位哈希键。大质数混合，千万级体素下碰撞概率可忽略；
  /// 万一碰撞也只是合并两个体素的颜色，属可视化可接受误差。
  std::int64_t key(const Eigen::Vector3f& p) const {
    const auto mix = [this](double v) {
      return static_cast<std::int64_t>(std::floor(v / voxel_size_));
    };
    const std::int64_t ix = mix(p.x());
    const std::int64_t iy = mix(p.y());
    const std::int64_t iz = mix(p.z());
    return (ix * 92837111) ^ (iy * 689287499) ^ (iz * 283923481);
  }

  double voxel_size_{0.0};
  std::unordered_map<std::int64_t, Cell> cells_;
};

}  // namespace colorizer
