#include "super_livo_visual_b/estimator/map_commit_scheduler.hpp"

namespace superlivo_b {

MapCommitScheduler::MapCommitScheduler(const Options& options) : options_(options) {}

void MapCommitScheduler::stage(const PendingCloud& /*cloud*/) {
  // TODO(阶段 B1): 暂存点云，超过 max_pending 时丢弃最旧项
}

std::vector<PendingCloud> MapCommitScheduler::flush(const Eigen::Matrix4d& /*T_wl*/) {
  // TODO(阶段 B1): 用更新后的位姿刷新待提交点云并返回，供 OctVox 插入
  return {};
}

void MapCommitScheduler::clear() {
  // TODO(阶段 B1): 清空待提交队列
}

const MapCommitScheduler::Options& MapCommitScheduler::options() const { return options_; }

}  // namespace superlivo_b
