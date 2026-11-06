#include "super_livo_visual_a/frontend/klt_frontend.hpp"

namespace superlivo_a {

KltFrontend::KltFrontend(const Options& options) : options_(options) {}

TrackResult KltFrontend::process(const cv::Mat& /*gray*/, double /*stamp*/) {
  // TODO(阶段 2): 灰度化 -> 建金字塔 -> FAST/Shi-Tomasi 检测
  //               -> PyrLK 跟踪 -> 前后向一致性检查 -> 数量与分布筛选
  return {};
}

void KltFrontend::reset() {
  // TODO(阶段 2): 清空上一帧图像、金字塔、特征缓存与 ID 分配器
}

const KltFrontend::Options& KltFrontend::options() const { return options_; }

}  // namespace superlivo_a
