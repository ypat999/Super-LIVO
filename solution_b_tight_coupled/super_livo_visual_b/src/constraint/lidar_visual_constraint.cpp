#include "super_livo_visual_b/constraint/lidar_visual_constraint.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

// Matrix4d::inverse() 需要 LU 模块的 Inverse 求值特化，否则运行期无限递归。
#include <Eigen/LU>

#include "super_livo_common/measurement_noise.hpp"

namespace superlivo_b {

namespace {
constexpr int kCellSize = 8;

Eigen::Matrix3d skew(const Eigen::Vector3d& p) {
  Eigen::Matrix3d m;
  m << 0.0, -p.z(), p.y(),
       p.z(), 0.0, -p.x(),
       -p.y(), p.x(), 0.0;
  return m;
}

double huberWeight(double norm, double delta) {
  if (delta <= 0.0 || norm <= delta) {
    return 1.0;
  }
  return delta / norm;
}
}  // namespace

LidarVisualConstraint::LidarVisualConstraint(const Options& options) : options_(options) {}

void LidarVisualConstraint::setMapPoints(std::vector<Eigen::Vector3d> points_world) {
  map_points_ = std::move(points_world);
}

std::vector<VisualResidual> LidarVisualConstraint::build(
    const std::vector<FeatureTrack>& tracks,
    const Eigen::Matrix4d& T_wc) {
  std::vector<VisualResidual> residuals;
  if (options_.fx <= 0.0 || options_.fy <= 0.0) {
    return residuals;  // 内参未配置
  }

  // 世界 -> 相机
  const Eigen::Matrix4d T_cw = T_wc.inverse();
  const Eigen::Matrix3d R_cw = T_cw.block<3, 3>(0, 0);
  const Eigen::Vector3d t_cw = T_cw.block<3, 1>(0, 3);

  // ---- 1. 把地图点投影到图像，建像素栅格 ---------------------------------
  struct Projected {
    double u{0.0};
    double v{0.0};
    double depth{0.0};
    Eigen::Vector3d xyz_world{Eigen::Vector3d::Zero()};
  };
  std::vector<Projected> projected;
  projected.reserve(map_points_.size());
  std::unordered_map<long long, std::vector<int>> grid;

  for (const auto& p_w : map_points_) {
    const Eigen::Vector3d p_c = R_cw * p_w + t_cw;
    if (p_c.z() <= 1e-3) {
      continue;
    }
    const double u = options_.fx * p_c.x() / p_c.z() + options_.cx;
    const double v = options_.fy * p_c.y() / p_c.z() + options_.cy;
    if (options_.image_width > 0 && options_.image_height > 0 &&
        (u < 0.0 || v < 0.0 || u >= options_.image_width || v >= options_.image_height)) {
      continue;
    }
    const long long key = (static_cast<long long>(u) / kCellSize) * 100000 +
                          (static_cast<long long>(v) / kCellSize);
    grid[key].push_back(static_cast<int>(projected.size()));
    projected.push_back(Projected{u, v, p_c.z(), p_w});
  }

  // ---- 2. 为每条轨迹建立残差 ---------------------------------------------
  const double pixel_radius = std::max(0.5, static_cast<double>(options_.pixel_radius));
  residuals.reserve(tracks.size());

  // 本帧的时刻基准：所有轨迹里最新的观测时刻，用于历史锚点过期判断。
  double frame_stamp = 0.0;
  for (const auto& track : tracks) {
    if (!track.observations.empty()) {
      frame_stamp = std::max(frame_stamp, track.observations.back().stamp);
    }
  }

  // 历史锚点栅格：把全部持久锚点（含本帧没跟住的）投影进图像。新建特征的锚点
  // 优先继承这些历史点——它们是在更早（往往更健康）的时刻建立的绝对坐标，
  // 而当前地图点已随 LIO 漂移。温和漂移段的纠正力就来自这种跨代传递。
  std::vector<Projected> hprojected;
  hprojected.reserve(anchors_.size());
  std::unordered_map<long long, std::vector<int>> hgrid;
  for (const auto& kv : anchors_) {
    const Eigen::Vector3d p_c = R_cw * kv.second.xyz + t_cw;
    if (p_c.z() <= 1e-3) {
      continue;
    }
    const double u = options_.fx * p_c.x() / p_c.z() + options_.cx;
    const double v = options_.fy * p_c.y() / p_c.z() + options_.cy;
    if (options_.image_width > 0 && options_.image_height > 0 &&
        (u < 0.0 || v < 0.0 || u >= options_.image_width || v >= options_.image_height)) {
      continue;
    }
    const long long key = (static_cast<long long>(u) / kCellSize) * 100000 +
                          (static_cast<long long>(v) / kCellSize);
    hgrid[key].push_back(static_cast<int>(hprojected.size()));
    hprojected.push_back(Projected{u, v, p_c.z(), kv.second.xyz});
  }

  for (const auto& track : tracks) {
    if (track.observations.empty()) {
      continue;
    }
    const FeatureObservation& last = track.observations.back();

    // 绝对锚定：优先复用持久锚点（首次观测帧建立，之后不再随当前位姿/地图重关联）。
    // 仅当没有持久锚点时才新建：优先用已有的三维锚点（三角化或 LiDAR 关联结果），
    // 否则在投影点栅格中找最近的地图点作为锚点。
    // 注意：track.valid 只表示轨迹本身可用，并不代表有三维锚点，
    // 这里必须看 triangulated（xyz_world 是否有效），否则会用 (0,0,0) 当锚点。
    Eigen::Vector3d anchor = track.xyz_world;
    bool has_anchor = track.triangulated;
    bool anchor_from_grid = false;
    bool anchored = false;

    auto saved = anchors_.find(track.id);
    if (saved != anchors_.end()) {
      anchor = saved->second.xyz;
      has_anchor = true;
      anchored = true;
    } else if (!has_anchor) {
      const long long cx = static_cast<long long>(last.pixel.x()) / kCellSize;
      const long long cy = static_cast<long long>(last.pixel.y()) / kCellSize;
      // 先试历史锚点：命中即继承其绝对坐标（按持久锚点走宽门限）。
      double best_distance = pixel_radius;
      for (long long dx = -1; dx <= 1; ++dx) {
        for (long long dy = -1; dy <= 1; ++dy) {
          auto it = hgrid.find((cx + dx) * 100000 + (cy + dy));
          if (it == hgrid.end()) {
            continue;
          }
          for (int index : it->second) {
            const Projected& p = hprojected[index];
            const double du = p.u - last.pixel.x();
            const double dv = p.v - last.pixel.y();
            const double distance = std::sqrt(du * du + dv * dv);
            if (distance <= best_distance) {
              best_distance = distance;
              anchor = p.xyz_world;
              has_anchor = true;
              anchored = true;
            }
          }
        }
      }
      if (!has_anchor) {
        // 再退回当前地图栅格关联（坐标来自漂移中的地图，仅在创建帧自洽）。
        best_distance = pixel_radius;
        for (long long dx = -1; dx <= 1; ++dx) {
          for (long long dy = -1; dy <= 1; ++dy) {
            auto it = grid.find((cx + dx) * 100000 + (cy + dy));
            if (it == grid.end()) {
              continue;
            }
            for (int index : it->second) {
              const Projected& p = projected[index];
              const double du = p.u - last.pixel.x();
              const double dv = p.v - last.pixel.y();
              const double distance = std::sqrt(du * du + dv * dv);
              if (distance <= best_distance) {
                best_distance = distance;
                anchor = p.xyz_world;
                has_anchor = true;
                anchor_from_grid = true;
              }
            }
          }
        }
      }
    }
    if (!has_anchor) {
      continue;
    }

    const Eigen::Vector3d p_c = R_cw * anchor + t_cw;
    if (p_c.z() <= 1e-3) {
      continue;
    }
    const double inv_z = 1.0 / p_c.z();
    const double u_pred = options_.fx * p_c.x() * inv_z + options_.cx;
    const double v_pred = options_.fy * p_c.y() * inv_z + options_.cy;

    // ---- 测量方差 R = variance·I（像素²） --------------------------------
    // 此前没有这一项，等价于假定 σ=1 px 且地图点是精确真值。43 个特征在
    // z≈4 m 处会给出 (f/z)²·43 ≈ 1e7 的信息量，即 ~0.3 mm 的等效位置精度，
    // 一帧就把位置协方差压塌，之后 LiDAR 先验再也拉不动（实测状态冻结在初值）。
    // 现在按"像素噪声 ⊕ 锚点三维不确定度投影"给出量级正确的 R。
    const double f_mean = 0.5 * (options_.fx + options_.fy);
    double variance = superlivo_common::reprojectionVariance(
        options_.sigma_pix, f_mean, options_.sigma_anchor, inv_z);
    if (anchor_from_grid || anchored) {
      // 栅格内最近点关联：锚点可能取到同一栅格里的另一个物理点，
      // 横向偏差最多 pixel_radius 个像素，按均匀分布折算标准差。
      // 持久锚点同样保留这份建立时刻的关联不确定度。
      variance += superlivo_common::associationVariance(options_.pixel_radius);
    }

    const Eigen::Vector2d residual(last.pixel.x() - u_pred, last.pixel.y() - v_pred);
    const double pixel_error = residual.norm();
    const double gate =
        anchored ? options_.max_pixel_error_anchored : options_.max_pixel_error;
    if (pixel_error > gate) {
      continue;  // 外点（锚点保留，下一帧再试）
    }

    Eigen::Matrix<double, 2, 3> dpi_dpc;
    dpi_dpc << options_.fx * inv_z, 0.0, -options_.fx * p_c.x() * inv_z * inv_z,
               0.0, options_.fy * inv_z, -options_.fy * p_c.y() * inv_z * inv_z;

    // 相机位姿左扰动 δ = [φ; ρ]：∂P_C/∂φ = R_cw·[P_W]×，∂P_C/∂ρ = -R_cw
    Eigen::Matrix<double, 3, 6> dPc_ddelta;
    dPc_ddelta.block<3, 3>(0, 0) = R_cw * skew(anchor);
    dPc_ddelta.block<3, 3>(0, 3) = -R_cw;

    VisualResidual out;
    out.feature_id = track.id;
    out.residual = residual;
    out.jacobian = -dpi_dpc * dPc_ddelta;
    out.weight = huberWeight(pixel_error, options_.huber_delta);
    out.variance = variance;
    out.valid = true;
    // 门限通过才算锚点"成功观测"：推进 last_seen 或提交新建。
    if (saved != anchors_.end()) {
      saved->second.last_seen = last.stamp;
    } else {
      anchors_.emplace(track.id, AnchorEntry{anchor, last.stamp, last.stamp});
    }
    residuals.push_back(out);
  }

  // 历史锚点过期：超过 anchor_keep_sec 未被任何轨迹成功观测的才清除；
  // 超过总量上限时按最久未观测的先淘汰（罕见，一次性 O(n) 即可）。
  const double keep = std::max(0.0, options_.anchor_keep_sec);
  for (auto it = anchors_.begin(); it != anchors_.end();) {
    if (frame_stamp - it->second.last_seen > keep) {
      it = anchors_.erase(it);
    } else {
      ++it;
    }
  }
  const int max_anchors = std::max(1, options_.max_anchors);
  if (static_cast<int>(anchors_.size()) > max_anchors) {
    std::vector<std::pair<double, int>> by_age;
    by_age.reserve(anchors_.size());
    for (const auto& kv : anchors_) {
      by_age.emplace_back(kv.second.last_seen, kv.first);
    }
    const auto cut = static_cast<std::ptrdiff_t>(by_age.size()) - max_anchors;
    std::nth_element(by_age.begin(), by_age.begin() + cut, by_age.end());
    for (std::ptrdiff_t i = 0; i < cut; ++i) {
      anchors_.erase(by_age[i].second);
    }
  }

  if (static_cast<int>(residuals.size()) < options_.min_constraints) {
    return {};
  }
  return residuals;
}

void LidarVisualConstraint::accumulate(const std::vector<VisualResidual>& residuals,
                                       Eigen::MatrixXd& H,
                                       Eigen::VectorXd& b,
                                       double& cost) const {
  constexpr int kDim = 6;
  H = Eigen::MatrixXd::Zero(kDim, kDim);
  b = Eigen::VectorXd::Zero(kDim);
  cost = 0.0;

  for (const auto& r : residuals) {
    if (!r.valid) {
      continue;
    }
    // Huber 权重仍在像素空间起作用（语义不变），噪声归一化单独按 1/variance
    // 施加，即 H = Σ w·Jᵀ R⁻¹ J、b = Σ w·Jᵀ R⁻¹ r。cost 保持像素²便于诊断。
    const double w = r.weight / std::max(1e-9, r.variance);
    H += w * r.jacobian.transpose() * r.jacobian;
    b += w * r.jacobian.transpose() * r.residual;
    cost += r.weight * r.residual.squaredNorm();
  }
}

const LidarVisualConstraint::Options& LidarVisualConstraint::options() const { return options_; }

}  // namespace superlivo_b
