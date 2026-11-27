#include "super_livo_visual_a/color/nn_color_transfer.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <thread>
#include <utility>
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
  // nearestKSearch 是只读操作，树可跨线程共享。按固定区间切块会负载失衡
  //（扫描尾段常是室内稠密区，借色命中与树回溯都更贵，实测尾块慢 3 倍），
  // 改为共享原子游标 + 小 tile 动态领取：线程干完一块自己领下一块。
  const std::size_t total = uncolored.size();
  std::size_t nthreads = std::thread::hardware_concurrency();
  nthreads = std::max<std::size_t>(1, std::min<std::size_t>(nthreads, 8));
  nthreads = std::min(nthreads, total);
  const std::size_t tile = std::max<std::size_t>(4096, total / (nthreads * 32));
  std::atomic<std::size_t> cursor{0};
  struct Chunk {
    std::size_t begin;
    std::size_t end;
  };
  // 每线程攒自己的借色块，join 后按 (tile 序号, 线程序) 排序拼接保持输入顺序。
  std::vector<std::vector<std::pair<std::size_t, pcl::PointCloud<pcl::PointXYZRGB>>>>
      results(nthreads);
  std::vector<std::thread> threads;
  threads.reserve(nthreads);
  for (std::size_t t = 0; t < nthreads; ++t) {
    threads.emplace_back([&, t] {
      std::vector<int> indices(1);
      std::vector<float> dists_sq(1);
      while (true) {
        const std::size_t begin = cursor.fetch_add(tile);
        if (begin >= total) {
          break;
        }
        const std::size_t end = std::min(total, begin + tile);
        pcl::PointCloud<pcl::PointXYZRGB> buf;
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
        if (!buf.empty()) {
          results[t].emplace_back(begin, std::move(buf));
        }
      }
    });
  }
  for (auto& th : threads) {
    th.join();
  }
  std::vector<std::pair<std::size_t, const pcl::PointCloud<pcl::PointXYZRGB>*>> ordered;
  for (const auto& mine : results) {
    for (const auto& r : mine) {
      ordered.emplace_back(r.first, &r.second);
    }
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  for (const auto& r : ordered) {
    stats.colored += r.second->size();
  }
  out.reserve(stats.colored);
  for (const auto& r : ordered) {
    out.points.insert(out.points.end(), r.second->points.begin(), r.second->points.end());
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
