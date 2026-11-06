#pragma once

#include <vector>

#include <Eigen/Core>

#include "super_livo_visual_b/feature_map/feature_track.hpp"

namespace superlivo_b {

/// 视觉残差块（对统一状态或相机位姿）。
struct VisualResidual {
  int feature_id{0};
  Eigen::Vector2d residual{Eigen::Vector2d::Zero()};
  Eigen::Matrix<double, 2, 6> jacobian{Eigen::Matrix<double, 2, 6>::Zero()};
  double weight{1.0};
  bool valid{false};
};

/// 第一种视觉约束：LiDAR 地图辅助的 2D-3D 重投影。
/// 优势是尺度与深度明确，但地图扭曲时会继承误差。
class LidarVisualConstraint {
 public:
  struct Options {
    double huber_delta{1.0};
    int min_constraints{8};
    double max_pixel_error{8.0};
  };

  explicit LidarVisualConstraint(const Options& options);

  std::vector<VisualResidual> build(const std::vector<FeatureTrack>& tracks,
                                    const Eigen::Matrix4d& T_wc) const;

  void accumulate(const std::vector<VisualResidual>& residuals,
                  Eigen::MatrixXd& H,
                  Eigen::VectorXd& b,
                  double& cost) const;

  const Options& options() const;

 private:
  Options options_;
  // TODO: OctVox 地图点查询接口
};

}  // namespace superlivo_b
