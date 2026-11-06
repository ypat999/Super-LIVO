#include "super_livo_visual_b/frontend/patch_tracker.hpp"

namespace superlivo_b {

PatchTracker::PatchTracker(const Options& options) : options_(options) {}

std::vector<PatchObservation> PatchTracker::process(const cv::Mat& /*gray*/, double /*stamp*/) {
  // TODO(阶段 B2): 建金字塔 -> 检测 -> LSSD/KLT 图像块跟踪 -> 输出观测与质量
  return {};
}

void PatchTracker::reset() {
  // TODO(阶段 B2): 清空图像块缓存与上一帧观测
}

const PatchTracker::Options& PatchTracker::options() const { return options_; }

}  // namespace superlivo_b
