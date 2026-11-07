#include "super_livo_visual_b/feature_map/feature_track.hpp"

#include <algorithm>
#include <cmath>

// Matrix4d::inverse() 需要 LU 模块的 Inverse 求值特化，否则运行期无限递归。
#include <Eigen/Geometry>
#include <Eigen/LU>

namespace superlivo_b {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

void FeatureTrack::addObservation(const FeatureObservation& observation) {
  // 按 stamp 升序插入，保证多帧约束的时间顺序。
  auto it = std::lower_bound(observations.begin(), observations.end(), observation.stamp,
                             [](const FeatureObservation& a, double stamp) {
                               return a.stamp < stamp;
                             });
  observations.insert(it, observation);
  valid = observations.size() >= 2;
}

double FeatureTrack::parallaxDeg() const {
  if (observations.size() < 2) {
    return 0.0;
  }

  // 视差 = 同一空间点在两帧中的视线夹角。
  // 做法：把第 i 帧的视线方向变换到第 j 帧的相机系，再与第 j 帧的视线求夹角。
  // 注意必须包含相对平移（T_ji），只比较旋转会把纯平移运动的视差误判为 0。
  double max_angle = 0.0;
  for (std::size_t i = 0; i < observations.size(); ++i) {
    const Eigen::Vector3d ray_i(observations[i].pixel.x(), observations[i].pixel.y(), 1.0);
    const Eigen::Vector3d ray_i_normalized = ray_i.normalized();

    for (std::size_t j = i + 1; j < observations.size(); ++j) {
      // T_ji：第 i 帧相机系 -> 第 j 帧相机系
      const Eigen::Matrix4d T_ji = observations[j].T_wc.inverse() * observations[i].T_wc;
      const Eigen::Vector3d ray_i_in_j = T_ji.block<3, 3>(0, 0) * ray_i_normalized;
      const Eigen::Vector3d ray_j(observations[j].pixel.x(), observations[j].pixel.y(), 1.0);

      const double cos_angle =
          std::max(-1.0, std::min(1.0, ray_i_in_j.normalized().dot(ray_j.normalized())));
      max_angle = std::max(max_angle, std::acos(cos_angle) * 180.0 / kPi);
    }
  }
  return max_angle;
}

bool FeatureTrack::readyForConstraint(int min_observations, double min_parallax_deg) const {
  if (static_cast<int>(observations.size()) < min_observations) {
    return false;
  }
  return parallaxDeg() >= min_parallax_deg;
}

}  // namespace superlivo_b
