#include "super_livo_visual_a/update/visual_update.hpp"

namespace superlivo_a {

VisualUpdate::VisualUpdate(const Options& options) : options_(options) {}

VisualUpdateOutput VisualUpdate::apply(const VisualUpdateInput& /*input*/,
                                       const GateDecision& /*decision*/) const {
  // TODO(阶段 3): 依据门控决策与质量评估决定是否接受；
  //               返回状态增量，由 Super-LIO ESIKF 负责注入与协方差更新。
  return {};
}

const VisualUpdate::Options& VisualUpdate::options() const { return options_; }

}  // namespace superlivo_a
