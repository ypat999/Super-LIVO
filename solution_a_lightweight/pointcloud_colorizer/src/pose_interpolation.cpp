#include "pointcloud_colorizer/pose_interpolation.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace colorizer {

namespace {
/// 采样队列长度上限：只保留最近的一段插值窗口，避免缓存无限增长。
constexpr std::size_t kMaxSamples = 200;
}  // namespace

LidarPoseInterpolator::LidarPoseInterpolator(const Options& options) : options_(options) {}

void LidarPoseInterpolator::addSample(const PoseSample& sample) {
  // 按 stamp 升序插入，保证后续可以二分查找。
  auto it = std::lower_bound(samples_.begin(), samples_.end(), sample.stamp,
                             [](const PoseSample& a, double value) { return a.stamp < value; });
  samples_.insert(it, sample);

  // 裁剪过旧采样：只保留最近 kMaxSamples 个。
  while (samples_.size() > kMaxSamples) {
    samples_.erase(samples_.begin());
  }
}

bool LidarPoseInterpolator::poseAt(double stamp, Eigen::Matrix4d& T_wl) const {
  if (samples_.size() < 2) {
    return false;
  }

  // 找到第一个 stamp >= 查询时刻的采样。
  auto it = std::lower_bound(samples_.begin(), samples_.end(), stamp,
                             [](const PoseSample& a, double value) { return a.stamp < value; });

  if (it == samples_.begin()) {
    // 查询时刻早于最早采样：时间差足够小时直接用首个采样。
    if (it->stamp - stamp > options_.max_gap) {
      return false;
    }
    T_wl = it->T_wl;
    return true;
  }
  if (it == samples_.end()) {
    // 查询时刻晚于最新采样：时间差足够小时用最后一个采样。
    const auto& last = samples_.back();
    if (stamp - last.stamp > options_.max_gap) {
      return false;
    }
    T_wl = last.T_wl;
    return true;
  }

  const auto& s1 = *it;
  const auto& s0 = *(it - 1);
  const double dt_total = s1.stamp - s0.stamp;
  if (dt_total <= 0.0 || dt_total > options_.max_gap) {
    return false;  // 采样间隔过大，插值不可靠
  }
  const double alpha = (stamp - s0.stamp) / dt_total;

  // 平移：线性插值。
  const Eigen::Vector3d t = (1.0 - alpha) * s0.T_wl.block<3, 1>(0, 3) +
                            alpha * s1.T_wl.block<3, 1>(0, 3);

  // 旋转：四元数球面插值（slerp），注意 q 与 -q 表示同一旋转。
  Eigen::Quaterniond q0(s0.T_wl.block<3, 3>(0, 0));
  Eigen::Quaterniond q1(s1.T_wl.block<3, 3>(0, 0));
  q0.normalize();
  q1.normalize();
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() = -q1.coeffs();
  }
  const Eigen::Quaterniond q = q0.slerp(alpha, q1);

  T_wl = Eigen::Matrix4d::Identity();
  T_wl.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T_wl.block<3, 1>(0, 3) = t;
  return true;
}

void LidarPoseInterpolator::clear() { samples_.clear(); }

const LidarPoseInterpolator::Options& LidarPoseInterpolator::options() const { return options_; }

}  // namespace colorizer
