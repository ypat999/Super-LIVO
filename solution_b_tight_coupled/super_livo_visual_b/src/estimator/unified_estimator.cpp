#include "super_livo_visual_b/estimator/unified_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <array>

// 协方差求逆需要 LU 模块的 Inverse 求值特化，否则运行期无限递归。
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

namespace superlivo_b {

namespace {
/// so(3) 指数映射。
Eigen::Matrix3d so3Exp(const Eigen::Vector3d& w) {
  const double theta = w.norm();
  if (theta < 1e-9) {
    Eigen::Matrix3d skew;
    skew << 0.0, -w.z(), w.y(),
            w.z(), 0.0, -w.x(),
            -w.y(), w.x(), 0.0;
    return Eigen::Matrix3d::Identity() + skew;
  }
  return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
}
}  // namespace

UnifiedEstimator::UnifiedEstimator(const Options& options) : options_(options) {
  filter_.state = State{};
  filter_.covariance = Eigen::MatrixXd::Identity(StateLayout::kSize, StateLayout::kSize) *
                       options.initial_covariance;
}

void UnifiedEstimator::propagate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,
                                 double dt) {
  if (dt <= 0.0) {
    return;
  }
  State& s = filter_.state;

  // 去掉零偏后的 IMU 测量。
  const Eigen::Vector3d omega = gyro - s.bg;
  const Eigen::Vector3d acc_meas = acc - s.ba;      // 机系比力（未加重力）
  const Eigen::Matrix3d R_old = s.R;                // 本区间起点的姿态（Φ 用它构造）
  const Eigen::Vector3d a_w = R_old * acc_meas;     // 世界系线加速度（不含重力）

  // 名义状态欧拉积分。
  s.p += s.v * dt;
  s.v += (a_w + s.g) * dt;
  s.R = s.R * so3Exp(omega * dt);

  // ---- 协方差传播 P ← Φ P Φᵀ + Q·dt ----------------------------------------
  // 关键修复：此前只累加对角噪声、没有 Φ 传播，导致 位置↔速度↔零偏↔重力 之间
  // 永不产生相关性。而 LiDAR 先验/视觉更新只约束 rot+pos，速度和零偏因此
  // 完全不可观：任何残余加速度误差都会让速度自由漂移，位置被先验反复拉扯 →
  // 滤波器指数发散（实测 z 轴 2s 掉 8m、最终爆到 km 级）。
  //
  // 误差状态推导（与 injectError 的左扰动约定一致：R_t = exp(δθ)·R_n，δθ 在世界系）：
  //   δθ̇ = -R·δbg                （左扰动下无 ω× 自项，两轨迹同输入时 δθ 不变）
  //   δṗ = δv
  //   δv̇ = -(R·a_meas)× δθ - R·δba + δg
  //   δḃg = δḃa = δġ = 0
  // 一阶离散 Φ = I + F·dt。
  const int n = StateLayout::kSize;
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(n, n);
  Eigen::Matrix3d skew_aw;
  skew_aw << 0.0, -a_w.z(), a_w.y(),
             a_w.z(), 0.0, -a_w.x(),
             -a_w.y(), a_w.x(), 0.0;
  Phi.block<3, 3>(StateLayout::kRot, StateLayout::kBg) = -R_old * dt;
  Phi.block<3, 3>(StateLayout::kPos, StateLayout::kVel) = Eigen::Matrix3d::Identity() * dt;
  Phi.block<3, 3>(StateLayout::kVel, StateLayout::kRot) = -skew_aw * dt;
  Phi.block<3, 3>(StateLayout::kVel, StateLayout::kBa) = -R_old * dt;
  Phi.block<3, 3>(StateLayout::kVel, StateLayout::kGrav) = Eigen::Matrix3d::Identity() * dt;
  filter_.covariance = Phi * filter_.covariance * Phi.transpose();

  // 过程噪声：对角项，保证正定且计算量可控。
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
  q.segment<3>(StateLayout::kRot).setConstant(options_.gyro_noise * options_.gyro_noise * dt);
  q.segment<3>(StateLayout::kPos).setConstant(options_.acc_noise * options_.acc_noise * dt * dt);
  q.segment<3>(StateLayout::kVel).setConstant(options_.acc_noise * options_.acc_noise * dt);
  q.segment<3>(StateLayout::kBg).setConstant(options_.bias_gyro_noise * options_.bias_gyro_noise * dt);
  q.segment<3>(StateLayout::kBa).setConstant(options_.bias_acc_noise * options_.bias_acc_noise * dt);
  filter_.covariance.diagonal() += q;

  // 数值防护：协方差必须对称有界；出现非有限值说明步长/噪声配置异常，
  // 立即放弃本次传播（宁可保持上一拍，也不要把 NaN 灌进状态）。
  filter_.covariance = 0.5 * (filter_.covariance + filter_.covariance.transpose());
  if (!filter_.covariance.allFinite()) {
    filter_.covariance = Eigen::MatrixXd::Identity(n, n) * options_.initial_covariance;
  }
}

void UnifiedEstimator::lidarUpdate(const Eigen::MatrixXd& H, const Eigen::VectorXd& b,
                                   double /*cost*/) {
  const int n = StateLayout::kSize;
  if (H.rows() != n || H.cols() != n || b.size() != n) {
    return;  // 维度不匹配，忽略本次更新
  }
  // 先验同样需要限幅：先验 b 只含 rot/pos，但 dx=-P_new·b 会经协方差耦合
  // 把跳变扩散到速度/零偏块。一旦协方差被其它更新污染（如视觉注入偏差），
  // 未限幅的先验更新正是"帧间 46m 跳变"的泄漏路径。
  // 正常工作时先验每帧修正量在 cm/0.1° 级，下面的上限远不会误伤。
  applyInformationUpdate(H, b, 1.0, 0.05, 0.1, 0.5, 1.0e-3, 1.0e-2, 0.01);
}

void UnifiedEstimator::visualUpdate(const VisualUpdateInput& input) {
  applied_.valid = false;
  if (!options_.enable_visual) {
    return;
  }
  if (input.H.rows() != 6 || input.H.cols() != 6 || input.b.size() != 6) {
    return;  // 视觉法方程固定为 6 维相机位姿
  }
  if (!input.H.allFinite() || !input.b.allFinite()) {
    return;
  }

  // 病态防护：低视差/退化几何下 6x6 法方程条件数可爆到 1e12+，
  // 求逆得到的 dx 是指向零空间的巨大跳变（实测正是 B 发散的注入源之一）。
  {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(input.H);
    if (es.info() != Eigen::Success) {
      return;
    }
    const double lmin = es.eigenvalues()(0);
    const double lmax = es.eigenvalues()(5);
    if (lmax <= 0.0 || lmin <= 1e-12 * lmax) {
      return;  // 近奇异，本帧视觉更新直接放弃
    }
  }

  const int n = StateLayout::kSize;

  // 相机位姿 6 维（旋转 0..2, 平移 3..5）映射到统一状态的旋转块与位置块。
  // 约定核对（勿再改动）：lidar_visual_constraint 的 dPc_ddelta=[R_cw·skew(p_w) | -R_cw]
  // 正是世界系左扰动 ξ 下的 ∂P_C/∂ξ，且 T_wc'=exp(ξ)T_wc 与 T_wi'=exp(ξ)T_wi 是同一个 ξ，
  // 因此 rot/pos 块可直接一一对应，不需要杠杆臂伴随矩阵（杠杆臂只存在于机体系参数化）。
  const Eigen::Matrix<double, 6, 6> H_cam = 0.5 * (input.H + input.H.transpose());
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
  const std::array<int, 6> index = {StateLayout::kRot,     StateLayout::kRot + 1,
                                     StateLayout::kRot + 2, StateLayout::kPos,
                                     StateLayout::kPos + 1, StateLayout::kPos + 2};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      H(index[i], index[j]) += H_cam(i, j);
    }
    b(index[i]) += input.b(i);
  }

  const double quality = std::min(1.0, std::max(0.1, input.quality));
  const double weight = options_.visual_weight * quality;

  // ---- 信息量上限（κ） ---------------------------------------------------
  // 残差方差修好之后 H_vis 的量级已经合理（~1e4），但它单帧仍可能比 LiDAR 先验
  // 强几个数量级——地图点/关键帧位姿的误差是系统性的（LIO 漂移），不会随特征数
  // 按 √N 平均掉。这里把视觉法方程整体缩放到不超过 κ·λmax(P⁻¹|rot+pos)：
  // 允许视觉在 LiDAR 先验之上再提高一倍信息量，但不允许把协方差压塌到先验之下。
  // 此前缺这一步的直接后果是一帧把位置协方差压到 1e-6，之后先验再也拉不动，
  // 状态冻结在初值（实测自身 odom 全零、帧间位移中位 0.000）。
  VisualDiagnostics diag;
  diag.scale = 1.0;
  // 退化时按 severity 在"正常上限"与"退化上限"之间插值，让视觉真正接管。
  const double sev = std::min(1.0, std::max(0.0, input.degen_severity));
  const double info_ratio =
      (1.0 - sev) * options_.visual_info_ratio + sev * options_.visual_info_ratio_degen;
  const double pos_floor = (1.0 - sev) * options_.visual_pos_sigma_floor +
                           sev * options_.visual_pos_sigma_floor_degen;
  if (info_ratio > 0.0) {
    // 分块限幅：旋转块与平移块各自不超过
    //   min( κ·λmax(P⁻¹|该块), 1/σ_floor² )
    // 两个上限缺一不可：
    //  - 只看整体 λmax 会被旋转方向主导（重力把 roll/pitch 约束得很紧，
    //    P⁻¹ 对角最大 1.7e6 对应 0.04°），平移方向完全不受保护；
    //  - 只用相对先验的 κ 会被自指正反馈击穿（P 越小 cap 越大）。
    Eigen::MatrixXd h_rot = H_cam.block<3, 3>(0, 0);
    Eigen::MatrixXd h_pos = H_cam.block<3, 3>(3, 3);
    h_rot = 0.5 * (h_rot + h_rot.transpose());
    h_pos = 0.5 * (h_pos + h_pos.transpose());
    const Eigen::MatrixXd p_inv = filter_.covariance.inverse();
    double lambda_rot = 0.0;
    double lambda_pos = 0.0;
    for (int i = 0; i < 3; ++i) {
      lambda_rot = std::max(lambda_rot, p_inv(StateLayout::kRot + i, StateLayout::kRot + i));
      lambda_pos = std::max(lambda_pos, p_inv(StateLayout::kPos + i, StateLayout::kPos + i));
    }
    const double cap_rot =
        std::min(info_ratio * std::max(1e-12, lambda_rot),
                 1.0 / std::max(1e-12, options_.visual_rot_sigma_floor *
                                           options_.visual_rot_sigma_floor));
    const double cap_pos =
        std::min(info_ratio * std::max(1e-12, lambda_pos),
                 1.0 / std::max(1e-12, pos_floor * pos_floor));

    double l_rot = 0.0;
    double l_pos = 0.0;
    {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_r(h_rot);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_p(h_pos);
      if (es_r.info() == Eigen::Success) {
        l_rot = es_r.eigenvalues()(2);
      }
      if (es_p.info() == Eigen::Success) {
        l_pos = es_p.eigenvalues()(2);
      }
    }
    diag.info_lambda = std::max(l_rot, l_pos);
    diag.cap_lambda = std::max(cap_rot, cap_pos);
    double scale = 1.0;
    if (l_rot > cap_rot) {
      scale = std::min(scale, cap_rot / l_rot);
    }
    if (l_pos > cap_pos) {
      scale = std::min(scale, cap_pos / l_pos);
    }
    if (scale < 1.0) {
      diag.scale = scale;
      H *= scale;
      b *= scale;
    }
  }

  // 记录本帧等效法方程（6 维相机位姿，含 info-cap 缩放与 weight），
  // 供桥接等价写回 Super-LIO 的 ESKF（H 在统一状态里是 30x30，这里只留位姿块）。
  applied_.valid = true;
  applied_.H = (weight * diag.scale) * H_cam;
  applied_.b = (weight * diag.scale) * input.b;

  // 步长限幅：单帧视觉修正不超过 0.5°/0.02 m（旋转/平移）。
  // 这是发散的最后防线：即使残差/Jacobian 有系统性错误，也不可能把状态一步甩飞。
  // （A 方案的 VisualUpdate 早已用 max_dx_norm 做了同样的事，B 之前漏了。）
  applyInformationUpdate(H, b, weight, kMaxVisualRotStep, kMaxVisualPosStep,
                         kMaxVisualVelStep, kMaxVisualBgStep, kMaxVisualBaStep,
                         kMaxVisualGravStep, options_.visual_pose_only, &diag);
  vis_diag_ = diag;
}

void UnifiedEstimator::injectError(const Eigen::VectorXd& dx) {
  if (dx.size() != StateLayout::kSize) {
    return;
  }
  State& s = filter_.state;

  // 旋转用左扰动注入，其余为欧氏加法。
  s.R = so3Exp(dx.segment<3>(StateLayout::kRot)) * s.R;
  s.p += dx.segment<3>(StateLayout::kPos);
  s.v += dx.segment<3>(StateLayout::kVel);
  s.bg += dx.segment<3>(StateLayout::kBg);
  s.ba += dx.segment<3>(StateLayout::kBa);
  // 重力状态冻结：初值已由静止段 mean_acc 定向（见节点初始化），不该再被估计。
  // 实测教训：dx[grav] 经 P_new 耦合项可每帧旋转重力方向（即使幅值被拉回 9.81），
  // 600 帧即可偏出几十度，使 a+g 无法抵消、速度 200Hz 积分失控（y 方向 -1253m）。
  // 若将来需要在线估重力，必须先修好视觉注入的符号/一致性再解冻。
  // s.g += dx.segment<3>(StateLayout::kGrav);

  // 重力硬防护：幅值必须保持在 9.81±0.5 m/s²，超界则保方向、拉回幅值。
  // 重力被踢飞后名义传播 v+=(a+g)dt 会在几帧内炸掉整个状态，这里做最后兜底。
  const double gn = s.g.norm();
  if (gn > 0.1 && std::abs(gn - 9.81) > 0.5) {
    s.g *= 9.81 / gn;
  }

  // 外参默认固定；仅在显式开启时更新（标定可靠时不建议在线估计）。
  if (options_.estimate_extrinsics) {
    const Eigen::Vector3d rot_ic = dx.segment<3>(StateLayout::kExtrinsic);
    const Eigen::Vector3d trans_ic = dx.segment<3>(StateLayout::kExtrinsic + 3);
    s.T_ic.block<3, 3>(0, 0) = so3Exp(rot_ic) * s.T_ic.block<3, 3>(0, 0);
    s.T_ic.block<3, 1>(0, 3) += trans_ic;

    const Eigen::Vector3d rot_il = dx.segment<3>(StateLayout::kExtrinsic + 6);
    const Eigen::Vector3d trans_il = dx.segment<3>(StateLayout::kExtrinsic + 9);
    s.T_il.block<3, 3>(0, 0) = so3Exp(rot_il) * s.T_il.block<3, 3>(0, 0);
    s.T_il.block<3, 1>(0, 3) += trans_il;
  }
}

double UnifiedEstimator::clampSpeed(double max_speed) {
  if (!(max_speed > 0.0)) {
    return -1.0;
  }
  Eigen::Vector3d& v = filter_.state.v;
  const double vn = v.norm();
  if (!std::isfinite(vn) || vn <= max_speed) {
    return -1.0;
  }
  v *= max_speed / vn;
  return max_speed;
}

const FilterState& UnifiedEstimator::filterState() const { return filter_; }

void UnifiedEstimator::resetState(const State& state) {
  filter_.state = state;
  filter_.covariance = Eigen::MatrixXd::Identity(StateLayout::kSize, StateLayout::kSize) *
                       options_.initial_covariance;
  // 重力/外参按固定参数处理：初协方差压到极小，配合 injectError 的重力冻结，
  // 避免 P⁻¹/P_new 的耦合项把它们当自由状态来回踢。
  filter_.covariance.block<3, 3>(StateLayout::kGrav, StateLayout::kGrav) *= 1.0e-8;
  filter_.covariance.block<12, 12>(StateLayout::kExtrinsic, StateLayout::kExtrinsic) *= 1.0e-8;
}

const UnifiedEstimator::Options& UnifiedEstimator::options() const { return options_; }

void UnifiedEstimator::applyInformationUpdate(const Eigen::MatrixXd& H,
                                              const Eigen::VectorXd& b,
                                              double weight,
                                              double max_rot,
                                              double max_pos,
                                              double max_vel,
                                              double max_bg,
                                              double max_ba,
                                              double max_grav,
                                              bool pose_only,
                                              VisualDiagnostics* diag) {
  if (weight <= 0.0) {
    return;
  }

  // 信息形式更新：P_new = (P^-1 + w·H)^-1, dx = -P_new·(w·b)
  const Eigen::MatrixXd information =
      filter_.covariance.inverse() + weight * H;
  const Eigen::MatrixXd covariance_new = information.inverse();
  if (!covariance_new.allFinite()) {
    return;
  }

  Eigen::VectorXd dx = -covariance_new * (weight * b);
  if (!dx.allFinite()) {
    return;
  }

  // 位姿限定：视觉更新只允许修改 rot/pos，速度/零偏/重力分量清零。
  // 原因：视觉残差存在系统性偏差时，速度块每帧哪怕只被限幅到 0.05 m/s，
  // 600 帧也能累积出 30 m/s 的系统性速度漂移（实测 ±70m 游走即此模式）。
  // 速度/零偏只应由 IMU 传播与 LiDAR 先验经 Φ 耦合来修正。
  if (pose_only) {
    dx.segment<3>(StateLayout::kVel).setZero();
    dx.segment<3>(StateLayout::kBg).setZero();
    dx.segment<3>(StateLayout::kBa).setZero();
    dx.segment<3>(StateLayout::kGrav).setZero();
  }

  // 步长限幅：对 rot/pos/vel/bg/ba/grav 各块分别检查，超限时 dx **整体**等比缩小
  // （保持修正方向不变、只缩步长，保守且不会引入方向性错误）。
  // 关键教训：
  //  1) 必须连速度/零偏一起限——视觉只约束位姿，但 dx 会通过协方差耦合
  //     把巨大跳变灌进速度/零偏块，先验每帧只拉位置，形成"拉扯式游走"。
  //  2) 必须连重力一起限！dx[grav] 不经任何观测约束、完全由 P_new 耦合项决定，
  //     一旦被踢飞，名义传播 v+=(a+g)dt 会在几帧内把状态整体炸掉（实测 y 方向 -1022m）。
  // max_* <= 0 表示该块不限幅。
  {
    const double limits[6][2] = {
        {max_rot, dx.segment<3>(StateLayout::kRot).norm()},
        {max_pos, dx.segment<3>(StateLayout::kPos).norm()},
        {max_vel, dx.segment<3>(StateLayout::kVel).norm()},
        {max_bg, dx.segment<3>(StateLayout::kBg).norm()},
        {max_ba, dx.segment<3>(StateLayout::kBa).norm()},
        {max_grav, dx.segment<3>(StateLayout::kGrav).norm()},
    };
    double scale = 1.0;
    for (const auto& l : limits) {
      if (l[0] > 0.0 && l[1] > l[0]) {
        scale = std::min(scale, l[0] / l[1]);
      }
    }
    dx *= scale;
  }

  // 保持协方差对称，避免数值漂移。
  filter_.covariance = 0.5 * (covariance_new + covariance_new.transpose());

  if (diag != nullptr) {
    diag->dx_rot = dx.segment<3>(StateLayout::kRot).norm();
    diag->dx_pos = dx.segment<3>(StateLayout::kPos).norm();
    diag->sigma_pos =
        std::sqrt(std::max(0.0, covariance_new(StateLayout::kPos, StateLayout::kPos)));
    diag->sigma_rot =
        std::sqrt(std::max(0.0, covariance_new(StateLayout::kRot, StateLayout::kRot)));
    diag->vel_norm = filter_.state.v.norm();
  }

  injectError(dx);
}

}  // namespace superlivo_b
