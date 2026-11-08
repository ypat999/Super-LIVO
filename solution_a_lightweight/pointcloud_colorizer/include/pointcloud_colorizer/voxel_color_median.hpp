#pragma once
// 体素多视图颜色中位数（per-voxel temporal median）
//
// 与旧的 VoxelColorFusion 的根本区别：
//   旧：对体素内所有观测做空间加权平均 → 同一 3cm 格子里不同物理点混色 → 边缘模糊
//   新：对每个 1cm 体素保存最近 K 帧 RGB，输出时逐通道取时间中位数
//       中位数对椒盐天然鲁棒：6 帧中 2 帧错色，中位数仍然正确
//
// 体素内只有同一物理面的亚厘米级抖动，不是空间平均，不会糊。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>

#include <Eigen/Core>

namespace colorizer {

class VoxelColorMedian {
public:
  // K = 最近观测帧数（偶数），中位数 = 第 K/2 与第 K/2+1 排序元素的均值。
  static constexpr int K = 6;

  VoxelColorMedian() = default;
  explicit VoxelColorMedian(float voxel_size) { set_voxel_size(voxel_size); }

  void set_voxel_size(float vs) {
    enabled_ = (vs > 0.0f);
    inv_     = enabled_ ? 1.0f / vs : 0.0f;
    vs_      = vs;
  }

  bool   enabled() const { return enabled_; }
  size_t size()    const { return map_.size(); }
  void   clear()         { map_.clear(); }

  /// 记录一帧观测：pos 世界系坐标，rgb 分量范围 [0,255]。
  /// quality <= 0 时由调用方自行过滤，此处不再检查。
  void insert(const Eigen::Vector3f& pos, const Eigen::Vector3f& rgb) {
    if (!enabled_) return;
    Key key{
      static_cast<int32_t>(std::floor(pos.x() * inv_)),
      static_cast<int32_t>(std::floor(pos.y() * inv_)),
      static_cast<int32_t>(std::floor(pos.z() * inv_))
    };
    auto& e = map_[key];
    auto& r = e.rgbs[e.next];
    r[0] = clamp_u8(rgb.x());
    r[1] = clamp_u8(rgb.y());
    r[2] = clamp_u8(rgb.z());
    e.next  = static_cast<uint8_t>((e.next + 1) % K);
    if (e.count < K) ++e.count;
  }

  /// 查询某位置所在体素的时间中位色（rgb 分量 [0,255]）。
  /// 返回 false 表示该体素没有观测（调用方保留原点色）。
  /// 用途：几何点全部保留，只在落盘时用体素中位色覆盖颜色，
  ///       不改变点数与坐标，避免体素化导致点云变稀。
  bool colorAt(const Eigen::Vector3f& pos, Eigen::Vector3d& rgb) const {
    if (!enabled_) return false;
    const Key key{
      static_cast<int32_t>(std::floor(pos.x() * inv_)),
      static_cast<int32_t>(std::floor(pos.y() * inv_)),
      static_cast<int32_t>(std::floor(pos.z() * inv_))
    };
    const auto it = map_.find(key);
    if (it == map_.end() || it->second.count == 0) return false;
    rgb = Eigen::Vector3d(
        median_ch(it->second, 0),
        median_ch(it->second, 1),
        median_ch(it->second, 2));
    return true;
  }

private:
  struct Key {
    int32_t x, y, z;
    bool operator==(const Key& o) const {
      return x == o.x && y == o.y && z == o.z;
    }
  };
  struct KeyHash {
    size_t operator()(const Key& k) const {
      // 三维整型混合（splitmix64 变体）
      uint64_t h = static_cast<uint32_t>(k.x) * 0x9e3779b97f4a7c15ULL;
      h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.y)) * 0xc2b2ae3d27d4eb4fULL;
      h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.z)) * 0x165667b19e3779f9ULL;
      h ^= h >> 32;
      return static_cast<size_t>(h);
    }
  };

  struct Entry {
    uint8_t rgbs[K][3] = {};
    uint8_t count     = 0;
    uint8_t next      = 0;
  };

  static uint8_t clamp_u8(float v) {
    return static_cast<uint8_t>(
      std::min(255, std::max(0, static_cast<int>(v + 0.5f))));
  }

  double median_ch(const Entry& e, int ch) const {
    uint8_t buf[K];
    for (int i = 0; i < e.count; ++i) buf[i] = e.rgbs[i][ch];
    std::sort(buf, buf + e.count);
    const int mid = e.count / 2;
    if (e.count & 1) return static_cast<double>(buf[mid]);
    return (static_cast<double>(buf[mid - 1]) + buf[mid]) * 0.5;
  }

  float vs_      = 0.0f;
  float inv_     = 0.0f;
  bool  enabled_ = false;

  std::unordered_map<Key, Entry, KeyHash> map_;
};

}  // namespace colorizer
