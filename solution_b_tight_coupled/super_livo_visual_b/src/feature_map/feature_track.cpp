#include "super_livo_visual_b/feature_map/feature_track.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

// Matrix4d::inverse() 需要 LU 模块的 Inverse 求值特化，否则运行期无限递归。
#include <Eigen/Geometry>
#include <Eigen/LU>

namespace superlivo_b {

namespace {
constexpr double kPi = 3.14159265358979323846;

/// 两帧观测的视线夹角（度）：把 a 的视线方向变到 b 的相机系，再与 b 的视线求夹角。
/// 只用旋转块（平移从未参与视差定义），且旋转乘单位向量仍是单位向量，只需 clamp 余弦。
double pairAngleDeg(const FeatureObservation& a, const FeatureObservation& b) {
  const Eigen::Vector3d ray_a =
      Eigen::Vector3d(a.pixel.x(), a.pixel.y(), 1.0).normalized();
  const Eigen::Vector3d ray_b =
      Eigen::Vector3d(b.pixel.x(), b.pixel.y(), 1.0).normalized();
  const Eigen::Vector3d ray_a_in_b =
      b.T_wc.block<3, 3>(0, 0).transpose() * a.T_wc.block<3, 3>(0, 0) * ray_a;
  const double cos_angle = std::max(-1.0, std::min(1.0, ray_a_in_b.dot(ray_b)));
  return std::acos(cos_angle) * 180.0 / kPi;
}
}  // namespace

void FeatureTrack::addObservation(const FeatureObservation& observation) {
  // 按 stamp 升序插入，保证多帧约束的时间顺序。
  auto it = std::lower_bound(observations.begin(), observations.end(), observation.stamp,
                             [](const FeatureObservation& a, double stamp) {
                               return a.stamp < stamp;
                             });
  observations.insert(it, observation);

  if (observations.size() > kMaxObservations) {
    // 超出上限：保留首帧（最长基线）+ 最近的 kMaxObservations-1 帧。
    std::vector<FeatureObservation> kept;
    kept.reserve(kMaxObservations);
    kept.push_back(observations.front());
    kept.insert(kept.end(), observations.end() - static_cast<std::ptrdiff_t>(kMaxObservations - 1),
                observations.end());
    observations.swap(kept);
  }
  // 视差缓存只做失效，不在这里读位姿：更新轨迹的路径上不碰 Eigen 数据，
  // 下一次 parallaxDeg() 按上限规模（最多 120 对）重建，成本可忽略。
  max_parallax_cache_ = -1.0;
  valid = observations.size() >= 2;
}

double FeatureTrack::parallaxDeg() const {
  if (observations.size() < 2) {
    return 0.0;
  }
  if (max_parallax_cache_ < 0.0) {
    // 缓存失效（新建轨迹或被裁剪）才走全量，规模已被 kMaxObservations 限制。
    double max_angle = 0.0;
    for (std::size_t i = 0; i < observations.size(); ++i) {
      for (std::size_t j = i + 1; j < observations.size(); ++j) {
        max_angle = std::max(max_angle, pairAngleDeg(observations[i], observations[j]));
      }
    }
    max_parallax_cache_ = max_angle;
  }
  return max_parallax_cache_;
}

bool FeatureTrack::readyForConstraint(int min_observations, double min_parallax_deg) const {
  if (static_cast<int>(observations.size()) < min_observations) {
    return false;
  }
  return parallaxDeg() >= min_parallax_deg;
}

}  // namespace superlivo_b
