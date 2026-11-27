#pragma once

#include <cstddef>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace superlivo_a {

/// 最近邻染色的统计结果。
struct NNColorTransferStats {
  std::size_t total{0};    ///< 待染色（未染色）点数
  std::size_t colored{0};  ///< 在半径内找到彩色点、成功染色的点数
};

/// 对未染色点云的每个点，在彩色点云中查找距离最近的点：
/// 距离 <= max_dist（米）则采用该彩色点的 RGB 并写入 out；超界的点视为仍然
/// 未染色，**不写入 out**（out 只含成功借色的点，保持原输入顺序）。
/// colored 为空或 max_dist <= 0 时 out 为空。
NNColorTransferStats transferNNColor(const pcl::PointCloud<pcl::PointXYZRGB>& colored,
                                     const pcl::PointCloud<pcl::PointXYZRGB>& uncolored,
                                     double max_dist,
                                     pcl::PointCloud<pcl::PointXYZRGB>& out);

/// 文件版（独立运行入口）：读彩色 PCD + 未染色 PCD，最近邻染色后写 out_pcd_path
/// （只写借色成功的点）。未染色输入允许没有 RGB 字段（按 XYZI 读取）；
/// 彩色输入必须含 RGB 字段。失败返回 false 并填写 err。
bool transferNNColorFiles(const std::string& colored_pcd_path,
                          const std::string& uncolored_pcd_path,
                          const std::string& out_pcd_path,
                          double max_dist,
                          NNColorTransferStats& stats,
                          std::string& err);

}  // namespace superlivo_a
