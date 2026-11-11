#pragma once

#include <unordered_map>
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
    double max_pixel_error{8.0};        ///< 新建锚点首帧的离群门限（px）
    /// 持久锚点离群门限（px）。锚点固定后残差含真实漂移，必须给足余量，
    /// 否则纠正力全被门限挡掉；鲁棒核对大残差自然降权。
    double max_pixel_error_anchored{60.0};
    // ---- 测量噪声模型（此前完全没有，等价于 σ_pix = 1 px 且锚点无误差）----
    // 单像素特征跟踪/亚像素拟合噪声（px）。
    double sigma_pix{1.5};
    // 三维锚点（LiDAR 地图点）的横向位置不确定度（m）。它投影到像素上是
    // f·σ_anchor/z，在 z=4 m、σ_anchor=5 cm 时约 7.6 px，远大于 σ_pix。
    // 这一项才是"视觉不能一帧压塌协方差"的关键：地图点带有 LIO 漂移造成的
    // 系统性误差，绝不能当成精确真值。
    double sigma_anchor{0.05};
    /// 历史锚点存活时长（秒，按最后成功观测时刻计）。超过该时长未被任何轨迹
    /// 跟住的锚点才会被清除。温和漂移段的纠正力依赖健康期锚点长期存活。
    double anchor_keep_sec{20.0};
    /// 锚点总量上限，超出时按最久未观测的先淘汰（防御长跑内存膨胀）。
    int max_anchors{20000};
  };

  explicit LidarVisualConstraint(const Options& options);

  /// 注入 OctVox 地图点（世界系）。
  void setMapPoints(std::vector<Eigen::Vector3d> points_world);

  /// 对每条轨迹的最后一次观测建立 2D-3D 残差。
  /// 绝对锚定：每个 feature_id 的三维锚点只在首次观测帧建立一次（当时的位姿 +
  /// 当时的地图），随后跨帧持久复用，不再逐帧重新关联。这样估计位姿一旦相对
  /// 锚点建立时刻漂移，重投影残差就会如实增大，产生真实回拉力；逐帧重关联会让
  /// 锚点跟着漂移走，残差恒为 0（自洽），视觉约束等于不存在。
  /// 历史锚点保留：本帧未被跟住的特征其锚点不再立即丢弃，而是保留
  /// anchor_keep_sec 秒；新建锚点时优先关联到投影位置重合的历史锚点
  /// （而不是当前漂移地图），从而把健康期建立的绝对坐标传递下去。
  std::vector<VisualResidual> build(const std::vector<FeatureTrack>& tracks,
                                    const Eigen::Matrix4d& T_wc);

  /// 汇总为 6 维法方程。
  void accumulate(const std::vector<VisualResidual>& residuals,
                  Eigen::MatrixXd& H,
                  Eigen::VectorXd& b,
                  double& cost) const;

  const Options& options() const;

 private:
  /// 持久锚点：feature_id -> 首次观测时建立的世界系三维坐标。
  struct AnchorEntry {
    Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
    double stamp{0.0};       ///< 建立时刻
    double last_seen{0.0};   ///< 最后一次被成功观测并用于残差的时刻
  };

  Options options_;
  std::vector<Eigen::Vector3d> map_points_;
  std::unordered_map<int, AnchorEntry> anchors_;
};

}  // namespace superlivo_b
