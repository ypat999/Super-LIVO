#include "super_livo_visual_a/color/nn_color_transfer.hpp"

#include <algorithm>
#include <cmath>
#include <thread>
#include <vector>

#include <pcl/exceptions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace superlivo_a {

NNColorTransferStats transferNNColor(const pcl::PointCloud<pcl::PointXYZRGB>& colored,
                                     const pcl::PointCloud<pcl::PointXYZRGB>& uncolored,
                                     double max_dist,
                                     pcl::PointCloud<pcl::PointXYZRGB>& out) {
  NNColorTransferStats stats;
  out.clear();
  stats.total = uncolored.size();
  if (colored.empty() || uncolored.empty() || max_dist <= 0.0) {
    return stats;
  }

  // 空删除器包装 const 引用，避免把几百万点的彩色云整份拷进树。
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_ptr(
      const_cast<pcl::PointCloud<pcl::PointXYZRGB>*>(&colored),
      [](pcl::PointCloud<pcl::PointXYZRGB>*) {});
  pcl::KdTreeFLANN<pcl::PointXYZRGB> tree;
  tree.setInputCloud(colored_ptr);

  const float max_dist_sq = static_cast<float>(max_dist * max_dist);
  // nearestKSearch 是只读操作，树可跨线程共享；按点区间切块并行查询
  //（12M 点实测单线程 ~400 s，8 线程降到 ~50 s，避免撞服务退出超时）。
  const std::size_t total = uncolored.size();
  std::size_t nthreads = std::thread::hardware_concurrency();
  nthreads = std::max<std::size_t>(1, std::min<std::size_t>(nthreads, 8));
  nthreads = std::min(nthreads, total);
  // 每线程攒自己的借色块，join 后按线程序拼接：结果保持输入顺序且确定。
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> partials(nthreads);
  std::vector<std::thread> threads;
  threads.reserve(nthreads);
  const std::size_t chunk = (total + nthreads - 1) / nthreads;
  for (std::size_t t = 0; t < nthreads; ++t) {
    const std::size_t begin = t * chunk;
    const std::size_t end = std::min(total, begin + chunk);
    if (begin >= end) {
      break;
    }
    threads.emplace_back([&, begin, end, t] {
      std::vector<int> indices(1);
      std::vector<float> dists_sq(1);
      auto& buf = partials[t];
      buf.reserve((end - begin) / 4);
      for (std::size_t i = begin; i < end; ++i) {
        const auto& p = uncolored.points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          continue;
        }
        if (tree.nearestKSearch(p, 1, indices, dists_sq) <= 0) {
          continue;
        }
        if (dists_sq[0] > max_dist_sq) {
          continue;  // 半径内没有彩色点：视为仍未染色，不写入结果
        }
        auto q = p;
        const auto& c = colored.points[indices[0]];
        q.r = c.r;
        q.g = c.g;
        q.b = c.b;
        buf.push_back(q);
      }
    });
  }
  for (auto& th : threads) {
    th.join();
  }
  for (const auto& part : partials) {
    stats.colored += part.size();
  }
  out.reserve(stats.colored);
  for (auto& part : partials) {
    out.points.insert(out.points.end(), part.points.begin(), part.points.end());
  }
  out.width = static_cast<std::uint32_t>(out.size());
  out.height = 1;
  out.is_dense = false;
  return stats;
}

namespace {

/// 读 PCD 为 XYZRGB。require_rgb=false 时允许无 RGB 字段（按 XYZI 读入置灰）。
bool loadAsXYZRGB(const std::string& path, bool require_rgb,
                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::string& err) {
  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(path, blob) != 0) {
    err = "无法读取 PCD 文件: " + path;
    return false;
  }
  bool has_rgb = false;
  for (const auto& f : blob.fields) {
    if (f.name == "rgb" || f.name == "rgba") {
      has_rgb = true;
      break;
    }
  }
  try {
    if (has_rgb) {
      pcl::fromPCLPointCloud2(blob, *cloud);
      return true;
    }
    if (require_rgb) {
      err = "点云缺少 RGB 字段（需要彩色源）: " + path;
      return false;
    }
    pcl::PointCloud<pcl::PointXYZI> plain;
    pcl::fromPCLPointCloud2(blob, plain);
    cloud->reserve(plain.size());
    for (const auto& p : plain) {
      pcl::PointXYZRGB q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;
      q.r = q.g = q.b = 128;  // 与管线未染色地图同口径的灰色
      cloud->push_back(q);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return true;
  } catch (const pcl::PCLException& e) {
    err = "PCD 字段转换失败: " + path + "（" + e.what() + "）";
    return false;
  }
}

}  // namespace

bool transferNNColorFiles(const std::string& colored_pcd_path,
                          const std::string& uncolored_pcd_path,
                          const std::string& out_pcd_path,
                          double max_dist,
                          NNColorTransferStats& stats,
                          std::string& err) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr uncolored(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!loadAsXYZRGB(colored_pcd_path, true, colored, err)) {
    return false;
  }
  if (!loadAsXYZRGB(uncolored_pcd_path, false, uncolored, err)) {
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGB> dyed;
  stats = transferNNColor(*colored, *uncolored, max_dist, dyed);
  dyed.width = static_cast<std::uint32_t>(dyed.size());
  dyed.height = 1;
  dyed.is_dense = false;
  if (pcl::io::savePCDFileBinary(out_pcd_path, dyed) != 0) {
    err = "写出 PCD 失败: " + out_pcd_path;
    return false;
  }
  return true;
}

}  // namespace superlivo_a
