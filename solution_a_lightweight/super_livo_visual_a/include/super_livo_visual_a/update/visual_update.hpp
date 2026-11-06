#pragma once

#include <Eigen/Core>

#include "super_livo_visual_a/constraint/degeneracy_gate.hpp"

namespace superlivo_a {

/// 视觉更新输入：视觉约束汇总后的法方程与权重。
struct VisualUpdateInput {
  Eigen::MatrixXd H;
  Eigen::VectorXd b;
  double quality{0.0};
};

/// 视觉更新输出：状态增量交由宿主 ESIKF 注入，本模块不直接修改状态。
struct VisualUpdateOutput {
  bool accepted{false};
  Eigen::VectorXd dx;
  double cost{0.0};
};

/// Super-LIO ESIKF 的视觉更新适配层。
/// 只做“接口适配”，不拥有滤波器、不拥有地图，便于随时关闭。
class VisualUpdate {
 public:
  struct Options {
    bool enable{false};
    double max_dx_norm{0.5};
  };

  explicit VisualUpdate(const Options& options);

  /// 依据门控决策与质量评估决定是否接受，并返回增量。
  VisualUpdateOutput apply(const VisualUpdateInput& input,
                           const GateDecision& decision) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace superlivo_a
