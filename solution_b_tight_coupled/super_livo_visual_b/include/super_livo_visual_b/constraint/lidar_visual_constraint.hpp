#pragma once

#include <vector>

#include <Eigen/Core>

#include "super_livo_visual_b/feature_map/feature_track.hpp"

namespace superlivo_b {

// ============================================================================
// 方案 B · 第一种视觉约束：LiDAR 地图辅助 2D-3D 重投影（对应 gpt §3.3）
// ----------------------------------------------------------------------------
// 三维锚点来自 OctVox 地图点（通过 setMapPoints 注入），投影到图像计算残差。
// 优势是尺度/深度明确；地图扭曲时会继承误差，因此需与 MSCKF 约束配合。
// 输出 6x6 法方程（相机位姿 se3），由 UnifiedEstimator 注入统一状态。
// ============================================================================

/// 视觉残差块。
struct VisualResidual {
  int feature_id{0};
  Eigen::Vector2d residual{Eigen::Vector2d::Zero()};
  Eigen::Matrix<double, 2, 6> jacobian{Eigen::Matrix<double, 2, 6>::Zero()};
  double weight{1.0};
  /// 该残差的测量方差（像素²）。accumulate 里按 1/variance 加权，
  /// 等价于 R = variance·I 的白化。默认 1.0 表示不缩放（光度残差用）。
  double variance{1.0};
  bool valid{false};
};

/// LiDAR 地图辅助的 2D-3D 重投影约束。
class LidarVisualConstraint {
 public:
  struct Options {
    double fx{0.0};  ///< 相机内参（必须设置）
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    int image_width{0};
    int image_height{0};
    int pixel_radius{3};
    double huber_delta{1.0};
    int min_constraints{8};
    double max_pixel_error{8.0};
    // ---- 测量噪声模型（此前完全没有，等价于 σ_pix = 1 px 且锚点无误差）----
    // 单像素特征跟踪/亚像素拟合噪声（px）。
    double sigma_pix{1.5};
    // 三维锚点（LiDAR 地图点）的横向位置不确定度（m）。它投影到像素上是
    // f·σ_anchor/z，在 z=4 m、σ_anchor=5 cm 时约 7.6 px，远大于 σ_pix。
    // 这一项才是"视觉不能一帧压塌协方差"的关键：地图点带有 LIO 漂移造成的
    // 系统性误差，绝不能当成精确真值。
    double sigma_anchor{0.05};
  };

  explicit LidarVisualConstraint(const Options& options);

  /// 注入 OctVox 地图点（世界系）。
  void setMapPoints(std::vector<Eigen::Vector3d> points_world);

  /// 对每条轨迹的最后一次观测建立 2D-3D 残差。
  std::vector<VisualResidual> build(const std::vector<FeatureTrack>& tracks,
                                    const Eigen::Matrix4d& T_wc) const;

  /// 汇总为 6 维法方程。
  void accumulate(const std::vector<VisualResidual>& residuals,
                  Eigen::MatrixXd& H,
                  Eigen::VectorXd& b,
                  double& cost) const;

  const Options& options() const;

 private:
  Options options_;
  std::vector<Eigen::Vector3d> map_points_;
};

}  // namespace superlivo_b
