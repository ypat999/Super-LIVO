#pragma once

#include <string>

namespace superlivo_a {

/// 视觉参与方式。
enum class VisualMode {
  kTrackingOnly,      ///< LiDAR 正常：只跟踪，不更新或低权重
  kAssistUpdate,      ///< LiDAR 轻度退化：视觉约束参与更新
  kHighWeightUpdate,  ///< LiDAR 严重退化：提高权重，但必须通过质量门控
  kRejected           ///< 视觉本身不可信：拒绝视觉更新
};

/// 门控输入：LiDAR 退化程度 + 视觉跟踪质量 + 可观测性。
struct GateInputs {
  double lidar_degeneracy{0.0};      ///< 0 正常 ~ 1 严重退化
  int tracked_features{0};
  double spatial_spread{0.0};        ///< 特征空间分布指标
  double reprojection_rmse{0.0};
  double jacobian_condition{0.0};    ///< 视觉约束雅可比条件数
  double motion_observability{0.0};  ///< 运动方向可观测性
};

/// 门控决策。
struct GateDecision {
  VisualMode mode{VisualMode::kTrackingOnly};
  double visual_weight{0.0};
  std::string reason;
};

/// 退化检测与视觉权重决策。
/// 注意：不是简单地在退化时无条件提高权重，需要同时检查可观测性。
class DegeneracyGate {
 public:
  struct Options {
    int min_tracked_features{30};
    double max_reprojection_rmse{2.0};
    double min_spread{0.15};
    double max_condition{1.0e3};
    double assist_weight{0.3};
    double high_weight{0.8};
  };

  explicit DegeneracyGate(const Options& options);

  GateDecision evaluate(const GateInputs& inputs) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace superlivo_a
