#ifndef SUPER_LIVO_VISUAL_B_ESTIMATOR_ROTATION_REFINER_HPP_
#define SUPER_LIVO_VISUAL_B_ESTIMATOR_ROTATION_REFINER_HPP_

#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace superlivo_b {

/// 纯旋转修正器：因果滑窗 + 旋转因子图（M2，P0-F `19i_perturb_recovery.py` 的 C++ 移植）。
///
/// 待求量 u_k 是**世界系左乘**修正量：R'_k = Exp(u_k) · R_k。输出即 u_k。
///
/// 目标函数（与 P0-F 逐项一致）：
///   min_u Σ_k ||fw_k · Cw_k · r_k||²        // 视觉因子（Huber 权重默认全 1）
///       + Σ_k ||u_k||² / σp²                // 一阶先验：锚在 LIO 基线上
///       + Σ_k ||u_k − 2u_{k−1} + u_{k−2}||² / σd²   // 二阶差分：修正量自身平滑
///   r_k = log( Exp(r_vis_k)ᵀ · (R'_{k+1}ᵀ R'_k) )
///
/// 约定：r_vis_k 是 **body 系**区间旋转（R_{k+1,k}，即把上一帧 body 转到本帧 body），
/// 由调用方从视觉观测估计并换算到 body 系。因此本模块**不依赖相机外参**，
/// 也不读写任何 ROS 状态——它的使命是与 B 的其他支路做公平对比。
///
/// 噪声口径（重要）：Cw = Σ^{-1/2} 的量纲是 1/rad。M2 复现用 W3
/// （实测视觉本征协方差，见总账 §36.12：主轴 σ = [0.0235, 0.0278, 0.0567]°）。
/// **σ 与 λ0（sigma_prior_deg）必须成对缩放**：σ×s 而 λ0 不变等于改了先验强度与
/// 数据的相对权重（§36.12 已实测该平面上结果极平，但仍须按配对口径书写）。
///
/// 本类**刻意**保留 P0-F 的解后硬投影（clip / slew）。任务书 §4.1 要求正式实现
/// 改为显式信任域约束；那属于 M5（R8 修正量阶跃）的工程改造，此处不做，避免
/// 复现阶段与 Python 结果出现不可归因的差异。
class RotationRefiner {
 public:
  struct Options {
    int window = 15;                 ///< 因果滑窗帧数（P0-F 的 W_SLIDE）
    double sigma_prior_deg = 0.30;   ///< λ0：一阶先验 σ（deg）
    double sigma_accel_deg = 0.50;   ///< λ2：二阶差分 σ（deg）；<=0 关闭该因子
    double clip_deg = 0.30;          ///< 单帧修正量模长上限（deg）
    double slew_factor = 2.0;        ///< 帧间修正量差分上限 = slew_factor × clip
    double huber_c = 2.5;            ///< 视觉 Huber 阈值（归一化残差模长）
    int irls_rounds = 1;             ///< 1 = 不做 Huber 重加权（P0-F 滑窗档口径）
    int max_iters = 6;               ///< 每个窗口的 LM 迭代上限
    double tol = 1e-7;               ///< LM 步长收敛阈值（rad）
    bool enable_bounds = true;       ///< false = 只解不投影（消融对照用）
  };

  /// 各向同性白化矩阵 Cw = I / (σ·D2R)。σ 必须与 λ0 配对使用（见类注释）。
  static Eigen::Matrix3d isotropicWhitening(double sigma_deg);

  RotationRefiner();
  explicit RotationRefiner(const Options& options);

  /// 清空滑窗与已发布修正量。
  void reset();

  /// 追加一帧并求解。
  /// @param R_body      LIO 给出的 body→world 旋转（修正量左乘于此）
  /// @param r_vis_body  上一帧到本帧的 **body 系**区间旋转向量（rad）；
  ///                     Exp(r_vis) = R_{k+1,k}，与 P0-F 的 Rvis 同约定
  /// @param Cw          该区间视觉残差的白化矩阵（1/rad）
  /// @param usable      该区间视觉是否可用；false 时本帧不产生视觉因子
  /// @return 本帧修正量 u_k（世界系左乘，rad）
  Eigen::Vector3d addFrame(const Eigen::Matrix3d& R_body,
                           const Eigen::Vector3d& r_vis_body,
                           const Eigen::Matrix3d& Cw,
                           bool usable);

  /// 最近一次 addFrame 返回的修正量（rad）。
  const Eigen::Vector3d& lastCorrection() const { return last_correction_; }

  /// 最近一帧的修正后旋转 Exp(u)·R_lio。
  const Eigen::Matrix3d& correctedRotation() const { return corrected_rotation_; }

  /// 最近一帧是否有视觉因子（诊断用）。
  bool lastFactorUsable() const { return last_usable_; }

  /// 当前滑窗帧数（诊断用）。
  int windowSize() const { return static_cast<int>(frames_.size()); }

  const Options& options() const { return options_; }

 private:
  struct Frame {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();     ///< LIO body→world
    Eigen::Matrix3d RvisT = Eigen::Matrix3d::Identity();  ///< Exp(r_vis)ᵀ（仅 usable 有效）
    Eigen::Matrix3d Cw = Eigen::Matrix3d::Identity();     ///< 视觉白化矩阵
    bool usable = false;
    Eigen::Vector3d u = Eigen::Vector3d::Zero();          ///< 已发布（投影后）修正量，作热启动
  };

  Options options_;
  std::deque<Frame> frames_;
  Eigen::Vector3d last_correction_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d corrected_rotation_ = Eigen::Matrix3d::Identity();
  bool last_usable_ = false;
};

}  // namespace superlivo_b

#endif  // SUPER_LIVO_VISUAL_B_ESTIMATOR_ROTATION_REFINER_HPP_
