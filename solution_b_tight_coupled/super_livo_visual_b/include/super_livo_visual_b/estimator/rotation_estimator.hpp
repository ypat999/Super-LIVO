#ifndef SUPER_LIVO_VISUAL_B_ESTIMATOR_ROTATION_ESTIMATOR_HPP_
#define SUPER_LIVO_VISUAL_B_ESTIMATOR_ROTATION_ESTIMATOR_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace superlivo_b {

/// 深度辅助的相对旋转估计（P0-F `_p0_rot.py::estimate_rotation_d` 的 C++ 移植）。
///
/// 模型：像素对 (a, b) 与 LIO 给出的**相机系**帧间平移 t_c 已知，
///   λ_i = q·x1_i（主导平面逆深度），x2_i ∝ R·x1_i + λ_i·t_c，
/// 联合求 6 参数 (δθ, q)，其中 R = Exp(δθ)·R0。裸 Procrustes（纯旋转假设）在
/// |t|/d̄≈0.6° 的视差下偏差远超 0.05° 目标，故必须有该深度项。
///
/// 与 Python 的一致性口径（见 exp_records §36.19）：
///   * scipy `trf + soft_l1(f_scale) + x_scale='jac'` 用逐分量 IRLS 权重复现
///     （ρ'(z)/z = 1/√(1+z²)），极小点一致但非逐位一致；
///   * 协方差按 (SSE/dof)·pinv(JᵀJ) 取旋转 3×3 块（Python 同式）。
///
/// 约定：R 是**相机 2 ← 相机 1** 的相对旋转（R·x1 ≈ x2），t_c 是相机 1→2 的平移
/// 在相机 2 系下的坐标；node 侧由 LIO 位姿 + 外参换算后传入。
class RotationEstimator {
 public:
  struct Options {
    /// 相机内参（与 PatchTracker 同口径：bag 内 640x512 的半分辨率内参）。
    double fx = 646.78;
    double fy = 646.66;
    double cx = 313.46;
    double cy = 261.40;
    double seed_px = 2.0;        ///< 单应 RANSAC 的像素阈值（外点门）
    int min_inl = 25;            ///< 最少内点/像素对
    double f_scale = 3e-4;       ///< soft_l1 尺度（单位视线残差，≈0.017°）
    int rounds = 10;             ///< 交替初值轮数上限
    int max_iters = 200;         ///< 每段 LM 迭代上限（对齐 scipy max_nfev=200）
    double min_baseline = 0.008; ///< 帧间平移下限（米），低于此判不可用
  };

  struct Result {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();   ///< 相机 2 ← 相机 1
    Eigen::Vector3d q = Eigen::Vector3d(0.0, 0.0, 0.2);  ///< 平面逆深度系数
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();     ///< 旋转协方差（rad²，3×3）
    Eigen::Vector3d sigma_rad = Eigen::Vector3d::Zero();  ///< cov 特征值开根（升序）
    double t_norm = 0.0;      ///< |t_c|（米）
    double th_deg = 0.0;      ///< R 的旋转角（度）
    double px_med = 0.0;      ///< 内点重投影像素误差中位数
    double px_p90 = 0.0;
    double d_med = 0.0;       ///< 内点中位深度（米）
    int n_pairs = 0;
    int n_inl = 0;
    bool usable = false;
    std::string reason;       ///< 不可用原因（诊断）
  };

  RotationEstimator() = default;
  explicit RotationEstimator(const Options& options) : options_(options) {}

  const Options& options() const { return options_; }
  Options& mutableOptions() { return options_; }

  /// @param a,b  对应的像素点（同一特征在两帧上的观测，长度相同）
  /// @param t_c  相机 2 系下的帧间平移（米）
  Result estimate(const std::vector<Eigen::Vector2d>& a,
                  const std::vector<Eigen::Vector2d>& b,
                  const Eigen::Vector3d& t_c) const;

 private:
  Options options_;
};

}  // namespace superlivo_b

#endif  // SUPER_LIVO_VISUAL_B_ESTIMATOR_ROTATION_ESTIMATOR_HPP_
