#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "super_livo_visual_b/constraint/lidar_visual_constraint.hpp"
#include "super_livo_visual_b/constraint/msckf_constraint.hpp"
#include "super_livo_visual_b/constraint/photometric_constraint.hpp"
#include "super_livo_visual_b/estimator/map_commit_scheduler.hpp"
#include "super_livo_visual_b/estimator/unified_estimator.hpp"
#include "super_livo_visual_b/feature_map/keyframe_manager.hpp"
#include "super_livo_visual_b/frontend/feature_lifecycle.hpp"
#include "super_livo_visual_b/frontend/patch_tracker.hpp"
#include "super_livo_visual_b/state/state_layout.hpp"
#include "pointcloud_colorizer/colorizer_pipeline.hpp"
#include "pointcloud_colorizer/voxel_color_median.hpp"
#include "super_livo_common/degeneracy_signature.hpp"

namespace superlivo_b {

// ============================================================================
// 方案 B · ROS2 节点（对应 gpt双方案设计.md §3.1）
// ----------------------------------------------------------------------------
// 把方案 B 的纯算法库接线成"可运行 ROS2 软件包"的节点层。定位主干仍是外部的
// Super-LIO，但视觉具备独立的多帧约束能力（关键帧滑窗 + 特征轨迹 + MSCKF），
// 通过 UnifiedEstimator 做 IMU 传播 + LiDAR 更新 + 视觉更新的统一融合。
//
// 与方案 A 的关键差异（gpt §四）：
//   * 视觉前端：Patch/LSSD（而非 KLT），并带特征生命周期管理；
//   * 视觉约束：LiDAR 2D-3D + MSCKF 多帧轨迹 + 可选 Patch 光度残差；
//   * 状态估计：统一 ESIKF（30 维状态，含外参可选在线估计）。
//
// 数据来源全部来自 Super-LIO 发布的 topic，运行顺序同方案 A：
//   先起 super_lio_node 处理 bag，再起本节点。
// ============================================================================

// ----------------------------------------------------------------------------
// 坐标系与外参约定（同方案 A，见 visual_a_node.hpp 顶部）：
//   T_wl : LiDAR -> 世界（lio/odom 提供，child_frame 标 "imu" 是命名误导）
//   T_lc : Camera -> LiDAR（从 Rcl/Pcl 构造）
//   T_wc : Camera -> 世界 = T_wl · T_lc
//
// 方案 B 额外约定（状态以 IMU 为中心，见 state_layout.hpp）：
//   State.R/p/v 是 IMU 姿态/位置/速度（T_wi），不是 LiDAR 位姿。
//   因此从 lio/odom 的 T_wl 得到 IMU 位姿需：T_wi = T_wl · T_li（T_li = LiDAR->IMU）。
//   T_il（IMU->LiDAR）在 State 里，T_ic（IMU->Camera）在 State 里；首版外参可固定
//   不参与在线估计（UnifiedEstimator::Options::estimate_extrinsics=false）。
//
// D2 决策（LiDAR 残差来源）：本节点不自实现点面匹配，而是把 lio/odom 的位姿当作
//   "LiDAR 观测"构造强先验（6 维 R/p 信息矩阵），注入 UnifiedEstimator::lidarUpdate
//   的 R/p 块。视觉约束负责在退化时修正这个先验。详见 visual_b_node.cpp。
// ----------------------------------------------------------------------------

/// 方案 B 紧耦合视觉节点。
class VisualBNode : public rclcpp::Node {
 public:
  /// @param options rclcpp 节点选项（launch 里透传参数文件）
  explicit VisualBNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// 退出时把累积的彩色地图再落盘一次（运行中用定时器周期保存）。
  ~VisualBNode() override;

 private:
  // ---- 订阅回调 ------------------------------------------------------------
  /// 图像回调：去畸变 -> 灰度 -> 驱动一帧主流程。
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  /// 里程计回调：更新 T_wl，并累积位姿采样（彩色点云运动补偿）。
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  /// 退化签名回调：布局 [cond6, axes, active, cond_t, evt_min, ux, uy, uz]
  void onDegeneracy(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg);

  /// @return false 表示没有新鲜的 LIO 退化签名（此时按"不退化"处理）
  bool degeneracySignature(double& severity, Eigen::Vector3d& dir_w) const;

  /// 点云回调：注入 LiDAR 地图点（世界系）供 2D-3D 约束 + 彩色点云。
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  /// 机体系点云回调：lio/body/cloud（LiDAR 系、已去畸变），用于彩色点云与延迟地图提交。
  void onBodyCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  /// IMU 回调：累积 IMU 样本（acc/gyro），供 UnifiedEstimator::propagate。
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

  // ---- 主流程 --------------------------------------------------------------
  /// 每帧图像触发：前端 -> 生命周期 -> 关键帧/轨迹 -> 视觉约束 -> 统一估计器。
  void processVisualFrame(const cv::Mat& gray, double stamp);

  /// 彩色点云流水线（复用 pointcloud_colorizer），同方案 A。
  ///
  /// 时间基准（与 A 完全一致，2026-11 修复）：
  ///   t_cloud = body_stamp_（扫描结束时刻，点已去畸变到该时刻）
  ///   t_pair  = 配对图像 header.stamp，只用于"选哪一帧图像"（|t_pair-t_cloud| < 7 ms）
  ///   t_img   = t_pair + img_time_offset_，只用于"取哪一时刻的相机位姿"
  /// 旧实现用 T_wl_（最新位姿）+ 整段 image_cache_ 交给流水线自选，既不是 t_cloud
  /// 也不是 t_img，且 sync_max_offset=0.3 会选到相邻帧图像，导致椒盐噪点。
  void runColorizer(const sensor_msgs::msg::PointCloud2& cloud_msg);

  /// 一帧待着色数据：点云 + 配对图像 + 两个时刻。
  struct PendingColorFrame {
    std::vector<Eigen::Vector3f> points_lidar;
    double t_cloud{0.0};
    double t_pair{0.0};
    double t_img{0.0};
    cv::Mat image;
    std::chrono::steady_clock::time_point queued_at;
  };

  /// 尝试为队列中的帧着色：位姿到齐的立即处理，等待超时的丢弃。
  void drainPendingColorization();

  /// 对单帧执行着色；位姿尚未到齐（未来时刻）时返回 false 表示"稍后重试"。
  bool colorizeFrame(const PendingColorFrame& frame);

  /// 按时间插值 T_wl；查询时刻晚于最新采样时返回 false（拒绝用最新位姿顶替未来时刻）。
  bool poseAtStamp(double stamp, Eigen::Matrix4d& T_wl) const;

  // ---- 辅助 ----------------------------------------------------------------
  /// 去畸变（保持输入分辨率，不做额外缩放；bag 图像已是 640x512）。
  cv::Mat rectifyBgr(const cv::Mat& bgr) const;
  Eigen::Matrix4d makeTlcFromRclPcl();                             ///< Rcl/Pcl -> T_lc（非 const）
  Eigen::Matrix4d applyLeftPerturbation(const Eigen::Matrix4d& T,
                                        const Eigen::VectorXd& dx) const;  ///< se3 左扰动
  /// 把 lio/odom 的 T_wl 先验构造成 30 维信息形式 H/b（只填充 R/p 块）。
  /// b = H·(x_cur ⊖ x_prior)，配合 applyInformationUpdate 的 dx = -P·b，
  /// 结果把当前状态拉回先验位姿。
  void buildLidarPrior(const Eigen::Matrix4d& T_wl,
                       Eigen::MatrixXd& H,
                       Eigen::VectorXd& b) const;
  /// 把本帧 patch 观测 + 相机位姿组装成 FeatureTrack（单观测），跨帧合并交给 KeyframeManager。
  std::vector<FeatureTrack> assembleTracks(
      const std::vector<PatchObservation>& obs, double stamp) const;
  void publishColoredCloud(const std::vector<colorizer::ColoredPoint>& points,
                           const builtin_interfaces::msg::Time& stamp);

  /// 把本帧上色点变换到世界系并累积到 colored_map_（供离线保存 PCD）。
  void accumulateColoredMap(const std::vector<colorizer::ColoredPoint>& points,
                            double stamp);

  /// 把已累积的世界系彩色地图写成二进制 PCD。
  void saveColoredMap();
  void publishOdomAndPath(const Eigen::Matrix4d& T_wl,
                          const builtin_interfaces::msg::Time& stamp);
  void loadParameters();

  // ---- 算法模块 ------------------------------------------------------------
  PatchTracker::Options patch_options_;
  std::unique_ptr<PatchTracker> frontend_;

  FeatureLifecycle::Options lifecycle_options_;
  std::unique_ptr<FeatureLifecycle> lifecycle_;

  KeyframeManager::Options keyframe_options_;
  std::unique_ptr<KeyframeManager> keyframe_manager_;

  LidarVisualConstraint::Options lidar_visual_options_;
  std::unique_ptr<LidarVisualConstraint> lidar_visual_;

  MsckfConstraint::Options msckf_options_;
  std::unique_ptr<MsckfConstraint> msckf_;

  PhotometricConstraint::Options photo_options_;
  std::unique_ptr<PhotometricConstraint> photometric_;

  UnifiedEstimator::Options estimator_options_;
  std::unique_ptr<UnifiedEstimator> estimator_;

  MapCommitScheduler::Options scheduler_options_;
  std::unique_ptr<MapCommitScheduler> scheduler_;

  colorizer::ColorizerPipeline::Options colorizer_options_;

  // ---- 相机/外参/时间 ------------------------------------------------------
  colorizer::CameraIntrinsics intrinsics_;
  std::vector<double> distortion_;
  double scale_{0.5};
  Eigen::Matrix4d T_lc_{Eigen::Matrix4d::Identity()};  ///< Camera->LiDAR
  Eigen::Matrix4d T_li_{Eigen::Matrix4d::Identity()};  ///< LiDAR->IMU（用于 T_wl -> T_wi）
  Eigen::Matrix4d T_wl_{Eigen::Matrix4d::Identity()};  ///< 当前 LiDAR 位姿
  /// 最新一条 lio/odom 自身的时刻：位姿回退到 T_wl_ 时，发布必须按这个时刻打戳。
  double last_odom_stamp_{0.0};
  /// 图像真实采集时刻相对 header.stamp 的偏移（秒），只用于取相机位姿。
  /// 本数据集实测最优 0.1 s（与官方 avia.yaml 一致），见 A 的 offset 扫描结论。
  double img_time_offset_{0.0};

  // ---- 数据缓存 ------------------------------------------------------------
  std::vector<colorizer::TimedImage> image_cache_;
  std::vector<colorizer::PoseSample> pose_cache_;
  std::vector<Eigen::Vector3d> map_points_;  ///< LiDAR 地图点（世界系）
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_cloud_;

  /// 当帧机体系点云（LiDAR 系，来自 lio/body/cloud）。
  std::vector<Eigen::Vector3f> body_points_;
  double body_stamp_{0.0};

  /// 上一帧参与跟踪的特征 id（用于 FeatureLifecycle::onMissed 统计）。
  std::vector<int> prev_feature_ids_;
  int next_keyframe_id_{0};

  /// 去畸变映射表（lazy 构建）。
  mutable cv::Mat rectify_map1_;
  mutable cv::Mat rectify_map2_;
  mutable bool rectify_maps_ready_{false};

  // ---- 缓存容量 ------------------------------------------------------------
  std::size_t max_image_cache_{30};
  std::size_t max_pose_samples_{300};
  std::size_t max_map_points_{60000};
  std::size_t max_imu_samples_{500};  ///< ≈1s（IMU 200Hz）
  int map_append_stride_{3};

  // ---- LiDAR 先验（D2）参数 ------------------------------------------------
  double prior_pos_sigma_{0.05};  ///< 位置先验标准差（米）
  double prior_rot_sigma_{0.02};  ///< 旋转先验标准差（弧度）
  // ---- 退化感知的先验（关键改动）----------------------------------------
  // 各向同性先验在退化场景会把状态强行拉回正在漂移的 LiDAR 位姿，
  // 而且因为 σ 固定，视觉即使有能力也抢不过来。退化时把位置先验按方向拆开：
  //   沿退化方向 -> σ_degen（松，允许视觉接管）
  //   垂直退化方向 -> σ_healthy（紧，视觉不许改）
  double prior_pos_sigma_healthy_{0.005};
  double prior_pos_sigma_degen_{1.0};
  bool prior_directional_en_{true};
  // ---- 退化段守卫（Trust + 速度限幅）--------------------------------------
  // 完全退化时（Super-LIO n_effect=0 → cond=1e12）LiDAR 先验在退化方向被放松，
  // 速度只剩 IMU 积分，实测从 2.4 m/s 涨到 60 m/s。两道守卫都只在退化段生效。
  //  ① Trust：上游 lio/odom 单帧大跳变时本帧不注入激光先验（同 Super-LIO 判据）；
  //  ② 速度上限：|v| ≤ 健康段观测到的最大速度 × ratio（无健康样本时用 fallback）。
  bool guard_enable_{true};
  double guard_lio_jump_pos_{1.0};      ///< 单帧位置跳变阈值（m）
  double guard_lio_jump_rot_{0.2618};    ///< 单帧姿态跳变阈值（rad，15°）
  double guard_sev_threshold_{0.9};      ///< 触发速度守卫的退化严重度
  double guard_vel_ratio_{1.5};          ///< 速度上限 = 健康段最大速度 × ratio
  double guard_vel_fallback_{3.0};       ///< 无健康样本时的速度上限（m/s）
  double guard_vel_abs_cap_{3.0};        ///< 物理绝对速度上限（m/s），防 vel_ref 被 spike 污染
  double vel_ref_{0.0};                  ///< 健康段观测到的最大速度（m/s）
  bool vel_ref_valid_{false};
  Eigen::Matrix4d last_trust_T_wl_{Eigen::Matrix4d::Identity()};
  double last_trust_stamp_{-1.0};
  std::size_t stat_prior_veto_{0};       ///< Trust 否决激光先验的次数
  std::size_t stat_vel_clamp_{0};        ///< 速度限幅触发次数

  /// IMU 样本缓存 {stamp, acc, gyro}，propagate 时取与图像最近的样本。
  struct ImuSample {
    double stamp{0.0};
    Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  };
  std::vector<ImuSample> imu_buffer_;

  /// 加计单位尺度：本数据集 IMU 输出为 g（静止模长≈1），需在初始化时按
  /// G_m_s2 / |mean_acc| 归一化到 m/s²，否则 propagate 的重力抵消失效导致 z 自由落体发散。
  double imu_acc_scale_{1.0};

  // ---- 订阅器 / 发布器 -----------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr body_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr degen_sub_;

  // ---- LiDAR 退化签名（来自 Super-LIO 的观测 Hessian） ---------------------
  // 判退化与给方向都由 Super-LIO（ros2 分支）已验证的检测负责，本节点只消费。
  mutable std::mutex degen_mutex_;
  std::string degen_topic_;
  superlivo_common::DegeneracySignature degen_sig_;  ///< 最新签名（解析/映射见 common）
  double degen_recv_time_{-1.0};
  bool degen_stale_warned_{false};
  double gate_cond_lo_{superlivo_common::DegeneracyScale{}.cond_lo};
  double gate_cond_hi_{superlivo_common::DegeneracyScale{}.cond_hi};
  double degen_timeout_{superlivo_common::DegeneracyScale{}.timeout_sec};
  /// 本帧图像时刻的 LiDAR 位姿（插值得到），避免"打图像戳、用最新 odom"的滞后。
  Eigen::Matrix4d T_wl_frame_{Eigen::Matrix4d::Identity()};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // ---- 运行状态 ------------------------------------------------------------
  bool params_loaded_{false};
  bool imu_initialized_{false};
  bool estimator_initialized_{false};  ///< 是否已用首帧 lio/odom 初始化统一状态
  double last_imu_stamp_{0.0};
  Eigen::Vector3d last_imu_acc_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_imu_gyro_{Eigen::Vector3d::Zero()};
  nav_msgs::msg::Path path_msg_;

  // ---- 彩色地图落盘（PCD） -------------------------------------------------
  /// 世界系彩色点（直接累积成 PCL 点云，落盘时无需再复制一遍）。
  pcl::PointCloud<pcl::PointXYZRGB> colored_map_;
  /// 逐体素多帧 RGB 时间中位数（pcd.median_voxel_size>0 时启用，替代 append）。
  colorizer::VoxelColorMedian median_map_;
  std::string pcd_save_dir_{"/home/ywj/slam_data/pcd"};  ///< 输出目录
  double pcd_save_interval_sec_{30.0};                   ///< 周期保存间隔（秒）
  /// 地图点数上限：全量点云约 2 万点/帧，2000 万点内存约 320 MB。
  std::size_t max_colored_map_points_{20000000};
  bool pcd_capacity_warned_{false};
  std::size_t pcd_saved_points_{0};                      ///< 上次落盘时的点数（无新增则跳过）
  rclcpp::TimerBase::SharedPtr pcd_save_timer_;          ///< 周期落盘定时器

  // ---- 彩色着色调度 --------------------------------------------------------
  /// 待着色队列：t_img 时刻的位姿通常要等 0.1 s 才发布，收到后由 drainPendingColorization 处理。
  std::deque<PendingColorFrame> pending_color_;

  /// 待配对图像的 dense 云队列（大图回调比云慢约 1 帧，云先入队等图）。
  /// 命名注意：`PendingCloud` 已被 MapCommitScheduler 占用（命名空间级），这里用独立名字。
  struct PendingCloudWait {
    std::vector<Eigen::Vector3f> points_lidar;
    double t_cloud{0.0};
    std::chrono::steady_clock::time_point queued_at;
  };
  std::deque<PendingCloudWait> pending_clouds_;

  /// 遍历等待图像的 dense 云：能配到图的转入 pending_color_，超时的丢弃。
  void drainPendingClouds();

  // ---- 视觉帧调度 ----------------------------------------------------------
  /// 待处理图像：/lio/odom 相对图像采集时刻约有 0.2 s 滞后（Super-LIO 单帧处理耗时
  /// + 队列），图像回调时 t_pose 的位姿往往还没发布。若当场处理，只能退回"最新位姿"，
  /// 视觉残差与 LiDAR 先验都用了 0.2 s 前的位姿（实测轨迹恒定滞后 0.2 s）。
  /// 这里先排队，等位姿到齐再算。
  struct PendingVisualFrame {
    cv::Mat gray;
    double stamp{0.0};  ///< 图像 header 时刻（相机钟），t_pose = stamp + img_time_offset_
    std::chrono::steady_clock::time_point queued_at;
  };
  std::deque<PendingVisualFrame> pending_visual_;

  /// 处理所有位姿已到齐的待定图像；超时的丢弃。
  void drainPendingVisualFrames();

  /// 着色用哪条轨迹：false=/lio/odom（Super-LIO 原始），true=统一估计器修正后的 odom。
  bool color_use_own_odom_{false};
  /// 修正位姿缓存（按图像 header 时刻），color_use_own_odom_=true 时供插值。
  std::vector<colorizer::PoseSample> own_pose_cache_;

  double last_colored_cloud_stamp_{-1.0};  ///< 已着色或已入队的最近 dense 帧时刻
  std::size_t stat_colored_frames_{0};
  std::size_t stat_drop_no_image_{0};
  std::size_t stat_drop_pose_{0};
  std::size_t stat_deferred_{0};
};

}  // namespace superlivo_b
