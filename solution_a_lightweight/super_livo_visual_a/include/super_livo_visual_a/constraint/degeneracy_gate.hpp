#pragma once

#include <string>

namespace superlivo_a {

// ============================================================================
// 方案 A · 退化检测与视觉权重门控（对应 gpt双方案设计.md §2.3）
// ----------------------------------------------------------------------------
// 目标：判断 LiDAR 是否退化，以及视觉应该在什么时机、以多大权重参与更新。
//
// 关键原则（源自 gpt 文档）：不是简单地在退化时无条件提高视觉权重，
// 而要同时检查视觉自身的可信度。尤其在狭窄走廊中，特征集中在前方远处时，
// 对横向运动或航向的约束可能并不强，需要一并考虑可观测性。
//
// 状态机（对应 gpt 表格）：
//   LiDAR 正常  -> kTrackingOnly   （只跟踪，不更新或低权重）
//   LiDAR 轻度退化 -> kAssistUpdate  （视觉约束参与更新）
//   LiDAR 严重退化 -> kHighWeightUpdate（提高权重，但必须通过质量门控）
//   视觉本身退化   -> kRejected       （拒绝视觉更新，保持纯 LIO 行为）
// ============================================================================

/// 视觉参与方式。
enum class VisualMode {
  kTrackingOnly,      ///< LiDAR 正常：只跟踪，不更新或低权重
  kAssistUpdate,      ///< LiDAR 轻度退化：视觉约束参与更新
  kHighWeightUpdate,  ///< LiDAR 严重退化：提高权重，但必须通过质量门控
  kRejected           ///< 视觉本身不可信：拒绝视觉更新
};

/// 门控输入：LiDAR 退化程度 + 视觉跟踪质量 + 可观测性。
struct GateInputs {
  double lidar_degeneracy{0.0};      ///< LiDAR 退化程度，0 正常 ~ 1 严重退化
  int tracked_features{0};           ///< 有效跟踪特征数量
  double spatial_spread{0.0};        ///< 特征空间分布指标（>min_spread 才有效）
  double reprojection_rmse{0.0};     ///< 重投影残差 RMS（像素）
  double jacobian_condition{0.0};    ///< 视觉约束雅可比条件数（<max_condition 才有效）
  double motion_observability{0.0};  ///< 运动方向可观测性（对应 gpt §六.1）
};

/// 门控决策。
struct GateDecision {
  VisualMode mode{VisualMode::kTrackingOnly};
  double visual_weight{0.0};  ///< 视觉约束在融合中的权重（0 表示不参与）
  std::string reason;         ///< 决策原因（调试/诊断用）
};

/// 退化检测与视觉权重决策。
class DegeneracyGate {
 public:
  struct Options {
    int min_tracked_features{30};    ///< 参与更新所需的最小特征数
    double max_reprojection_rmse{2.0}; ///< 允许的最大重投影 RMS（像素）
    double min_spread{0.15};         ///< 特征空间分布下限
    double max_condition{1.0e3};     ///< 雅可比条件数上限
    double assist_weight{0.3};       ///< 轻度退化时的视觉权重
    double high_weight{0.8};         ///< 严重退化时的视觉权重
  };

  explicit DegeneracyGate(const Options& options);

  GateDecision evaluate(const GateInputs& inputs) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace superlivo_a