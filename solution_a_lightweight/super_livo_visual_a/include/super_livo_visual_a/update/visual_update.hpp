#pragma once

#include <Eigen/Core>

#include "super_livo_visual_a/constraint/degeneracy_gate.hpp"

namespace superlivo_a {

// ============================================================================
// 方案 A · 视觉更新适配层（对应 gpt双方案设计.md §2.1 / §七）
// ----------------------------------------------------------------------------
// 目标：作为 Super-LIO ESIKF 的"可选视觉更新"入口。只做接口适配，
//   不拥有滤波器、不拥有地图，便于随时关闭（gpt §六.2 强调彩色点云与定位解耦，
//   视觉定位也应是可插拔模块）。
//
// gpt §七 的重要设计：视觉更新不只是一个位姿修正量，而应输出
//   带明确协方差与质量评估的观测，供 Super-LIO 按 LiDAR 退化程度、
//   视觉跟踪质量与方向可观测性决定相信哪种传感器。
// ============================================================================

/// 视觉更新输入：视觉约束汇总后的法方程与质量/cost。
struct VisualUpdateInput {
  Eigen::MatrixXd H;        ///< 视觉约束信息矩阵（状态空间，宿主 ESIKF 对齐）
  Eigen::VectorXd b;        ///< 视觉约束信息向量
  double quality{0.0};      ///< 视觉质量评估（0~1，由门控/残差/条件数综合）
  double cost{0.0};         ///< 累加的鲁棒代价（来自 ReprojectionConstraint::accumulate）
};

/// 视觉更新输出：状态增量交由宿主 ESIKF 注入，本模块不直接修改状态。
struct VisualUpdateOutput {
  bool accepted{false};     ///< 是否接受本次视觉更新
  Eigen::VectorXd dx;       ///< 状态增量（SE3 左扰动等）
  double cost{0.0};         ///< 本次更新的代价（回显自输入，诊断用）
};

/// Super-LIO ESIKF 的视觉更新适配层。
class VisualUpdate {
 public:
  struct Options {
    bool enable{false};         ///< 视觉更新开关（默认关闭，先保证 A1/A2 可验证）
    double max_dx_norm{0.5};    ///< 增量范数上限，超出则拒绝（防异常跳变）
  };

  explicit VisualUpdate(const Options& options);

  /// 依据门控决策与质量评估决定是否接受，并返回增量。
  /// @param input    视觉法方程 + 质量 + cost
  /// @param decision 门控决策（决定是否参与及权重）
  VisualUpdateOutput apply(const VisualUpdateInput& input,
                           const GateDecision& decision) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace superlivo_a