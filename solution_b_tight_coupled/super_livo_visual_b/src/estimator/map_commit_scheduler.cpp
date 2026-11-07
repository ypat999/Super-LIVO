#include "super_livo_visual_b/estimator/map_commit_scheduler.hpp"

namespace superlivo_b {

MapCommitScheduler::MapCommitScheduler(const Options& options) : options_(options) {}

void MapCommitScheduler::stage(const PendingCloud& cloud) {
  if (!options_.enable) {
    return;  // 未启用延迟提交：由调用方直接插入地图
  }
  pending_.push_back(cloud);
  // 队列超限时丢弃最旧的点云，避免内存与延迟持续增长。
  while (static_cast<int>(pending_.size()) > options_.max_pending) {
    pending_.erase(pending_.begin());
  }
}

std::vector<PendingCloud> MapCommitScheduler::flush(const Eigen::Matrix4d& T_wl) {
  // 本轮更新完成后，用修正后的位姿重新赋值，保证地图点不会被写错位置。
  // 点本身保持 LiDAR 系坐标，只需替换采集位姿。
  for (auto& cloud : pending_) {
    cloud.T_wl = T_wl;
  }
  std::vector<PendingCloud> result = pending_;
  pending_.clear();
  return result;
}

void MapCommitScheduler::clear() { pending_.clear(); }

const MapCommitScheduler::Options& MapCommitScheduler::options() const { return options_; }

}  // namespace superlivo_b
