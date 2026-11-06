#include "super_livo_visual_a/constraint/degeneracy_gate.hpp"

namespace superlivo_a {

DegeneracyGate::DegeneracyGate(const Options& options) : options_(options) {}

GateDecision DegeneracyGate::evaluate(const GateInputs& /*inputs*/) const {
  // TODO(阶段 3): 结合 LiDAR 退化程度、跟踪数量、空间分布、重投影残差、
  //               雅可比条件数与运动方向可观测性，输出 VisualMode 与权重
  return {};
}

const DegeneracyGate::Options& DegeneracyGate::options() const { return options_; }

}  // namespace superlivo_a
