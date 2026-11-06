#include "pointcloud_colorizer/pose_interpolation.hpp"

namespace colorizer {

LidarPoseInterpolator::LidarPoseInterpolator(const Options& options) : options_(options) {}

void LidarPoseInterpolator::addSample(const PoseSample& /*sample*/) {
  // TODO(阶段 1): 按时间有序插入并裁剪过期采样
}

bool LidarPoseInterpolator::poseAt(double /*stamp*/, Eigen::Matrix4d& /*T_wl*/) const {
  // TODO(阶段 1): 找到时间上相邻的两个位姿，对位置线性插值、对旋转做球面插值
  return false;
}

void LidarPoseInterpolator::clear() {
  // TODO(阶段 1): 清空采样队列
}

const LidarPoseInterpolator::Options& LidarPoseInterpolator::options() const { return options_; }

}  // namespace colorizer
