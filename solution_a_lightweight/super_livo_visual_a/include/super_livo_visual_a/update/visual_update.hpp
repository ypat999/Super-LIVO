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
  /// LiDAR 退化到的世界系方向（单位向量）与严重度 0~1。
  /// dx 的平移块就是世界系左扰动（ReprojectionConstraint 里 ∂P_C/∂ρ = -R_cw），
  /// 与 Super-LIO 发布的 H_tt 最弱特征向量同一坐标系，可直接对齐，无需变基。
  Eigen::Vector3d degen_dir_w{Eigen::Vector3d::Zero()};
  double degen_severity{0.0};
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
    // ---- LiDAR 先验正则项 -------------------------------------------------
    // 原实现直接解 (w·H_vis)·dx = -(w·b_vis)：权重 w 在左右两边同时出现会**被约掉**，
    // 所以"轻度退化 0.3 / 严重退化 0.8"的权重其实完全不改变 dx 的大小，
    // 一旦接受就是无正则的最小二乘全量拟合地图，等于让视觉一步接管位姿。
    // 现在按贝叶斯写法加上先验信息 H_prior = diag(1/σ_rot², 1/σ_pos²)
    // （误差状态以当前 LiDAR 位姿为参考，故先验残差为 0）：
    //   (w·H_vis + H_prior)·dx = -w·b_vis
    // 这样 w 才真正在"信 LiDAR"与"信视觉"之间插值。
    double prior_rot_sigma{0.02};  ///< LiDAR 姿态先验标准差（rad）
    double prior_pos_sigma{0.05};  ///< LiDAR 位置先验标准差（m）
    // ---- 按方向的先验（退化场景专用）--------------------------------------
    // 各向同性先验在退化场景有两个毛病：退化方向上它太强（视觉抢不过漂移的 LiDAR），
    // 健康方向上它又太弱（视觉会把走廊两侧本来很好的约束也一起改掉）。
    // 因此退化时把位置先验做成各向异性：沿退化方向松开到 σ_degen，
    // 垂直方向收紧到 σ_healthy，让视觉"只在看不住的那个方向说话"。
    bool directional_prior_en{true};
    double prior_pos_sigma_healthy{0.005};  ///< 非退化方向的位置先验标准差（m）
    double prior_pos_sigma_degen{1.0};      ///< 退化方向的位置先验标准差（m）
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