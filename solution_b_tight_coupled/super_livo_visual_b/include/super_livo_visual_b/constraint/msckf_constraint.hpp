#pragma once

#include <vector>

#include <Eigen/Core>

#include "super_livo_visual_b/feature_map/feature_track.hpp"

namespace superlivo_b {

// ============================================================================
// 方案 B · 第二种视觉约束：MSCKF 多帧视觉轨迹约束（对应 gpt §3.3）
// ----------------------------------------------------------------------------
// 利用一个特征跨多帧的观测构造多帧重投影约束，并把特征位置作为临时变量消去
// （零空间投影），只保留位姿相关的法方程。
//
// 本实现的工程简化（已在文档中标注）：
//   - 历史关键帧位姿视为已固定，只把残差投影到"当前相机位姿"的 6 维空间；
//   - 若后续把位姿滑窗纳入状态，可扩展为完整的 MSCKF 边缘化。
// 输出的 H 为 6x6、b 为 6x1，与 LidarVisualConstraint 一致，便于统一注入。
// ============================================================================

/// 多帧约束汇聚结果（法方程形式）。
struct MultiFrameNormalEquations {
  Eigen::MatrixXd H;  ///< 6x6（当前相机位姿）
  Eigen::VectorXd b;  ///< 6x1
  double cost{0.0};
  int used_features{0};
  bool valid{false};
};

/// MSCKF 风格多帧视觉轨迹约束。
class MsckfConstraint {
 public:
  struct Options {
    double fx{0.0};  ///< 相机内参（必须设置）
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    int min_observations{4};
    int max_track_length{20};
    double min_parallax_deg{1.0};
    double huber_delta{1.0};
    bool use_nullspace_projection{true};
  };

  explicit MsckfConstraint(const Options& options);

  /// 多视图线性三角化（返回世界系坐标），视差不足或退化时返回 false。
  bool triangulate(const FeatureTrack& track, Eigen::Vector3d& xyz) const;

  /// 构造多帧重投影约束，并投影到特征雅可比的左零空间。
  MultiFrameNormalEquations build(const std::vector<FeatureTrack>& tracks) const;

  const Options& options() const;

 private:
  Options options_;
};

}  // namespace superlivo_b
