#include "super_livo_visual_a/constraint/degeneracy_gate.hpp"

namespace superlivo_a {

DegeneracyGate::DegeneracyGate(const Options& options) : options_(options) {}

GateDecision DegeneracyGate::evaluate(const GateInputs& inputs) const {
  GateDecision decision;

  // ---- 第 1 步：视觉质量门控 ---------------------------------------------
  // 任何一个指标不合格，都认为视觉不可信，直接拒绝（保持纯 LIO 行为）。
  if (inputs.tracked_features < options_.min_tracked_features) {
    decision.mode = VisualMode::kRejected;
    decision.visual_weight = 0.0;
    decision.reason = "too few tracked features";
    return decision;
  }
  if (inputs.spatial_spread < options_.min_spread) {
    decision.mode = VisualMode::kRejected;
    decision.visual_weight = 0.0;
    decision.reason = "features not spatially distributed";
    return decision;
  }
  if (inputs.reprojection_rmse > options_.max_reprojection_rmse) {
    decision.mode = VisualMode::kRejected;
    decision.visual_weight = 0.0;
    decision.reason = "reprojection rmse too large";
    return decision;
  }
  if (inputs.jacobian_condition > options_.max_condition) {
    decision.mode = VisualMode::kRejected;
    decision.visual_weight = 0.0;
    decision.reason = "visual jacobian ill-conditioned";
    return decision;
  }

  // ---- 第 2 步：LiDAR 退化程度决定参与方式 -------------------------------
  const double degeneracy = inputs.lidar_degeneracy;

  if (degeneracy < 0.3) {
    // LiDAR 正常：只跟踪，不更新（视觉权重为 0）。
    decision.mode = VisualMode::kTrackingOnly;
    decision.visual_weight = 0.0;
    decision.reason = "lidar healthy, tracking only";
    return decision;
  }

  if (degeneracy < 0.7) {
    decision.mode = VisualMode::kAssistUpdate;
    decision.visual_weight = options_.assist_weight;
    decision.reason = "lidar mildly degenerate, assist update";
  } else {
    decision.mode = VisualMode::kHighWeightUpdate;
    decision.visual_weight = options_.high_weight;
    decision.reason = "lidar severely degenerate, high weight update";
  }

  // ---- 第 3 步：运动方向可观测性降权 -------------------------------------
  // 特征集中在远处/单一平面时，对横向或航向约束很弱，不能因为"退化"就全信视觉。
  if (inputs.motion_observability < 0.2) {
    decision.visual_weight *= 0.5;
    decision.reason += " (low observability, weight halved)";
    if (decision.visual_weight <= 0.0) {
      decision.mode = VisualMode::kRejected;
      decision.reason = "unobservable motion direction";
    }
  }

  return decision;
}

const DegeneracyGate::Options& DegeneracyGate::options() const { return options_; }

}  // namespace superlivo_a
