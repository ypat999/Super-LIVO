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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "super_livo_visual_a/constraint/degeneracy_gate.hpp"
#include "super_livo_visual_a/constraint/depth_association.hpp"
#include "super_livo_visual_a/constraint/reprojection_constraint.hpp"
#include "super_livo_visual_a/frontend/klt_frontend.hpp"
#include "super_livo_visual_a/update/visual_update.hpp"
#include "pointcloud_colorizer/colorizer_pipeline.hpp"
#include "pointcloud_colorizer/voxel_color_median.hpp"
#include "super_livo_common/degeneracy_signature.hpp"
namespace superlivo_a {

// ============================================================================
// 方案 A · ROS2 节点（对应 gpt双方案设计.md §2.1）
// ----------------------------------------------------------------------------
// 这是把方案 A 的纯算法库接线成"可运行 ROS2 软件包"的节点层。定位主干仍是
// 外部的 Super-LIO（third_party/Super-LIO），本节点不改上游、不持有滤波器，
// 只做两件事：
//   1) 彩色点云：复用 pointcloud_colorizer，把 LiDAR 点染上图像颜色并发布；
//   2) 退化视觉辅助：在 LiDAR 退化时用 2D-3D 重投影给出位姿修正，发布修正后的
//      里程计（后处理式，不侵入 Super-LIO 的 ESIKF）。
//
// 数据来源全部来自 Super-LIO 发布的 topic（详见下）。因此运行顺序必须是：
//   先启动 super_lio_node（处理 bag 得到 T_wl + 世界系点云），再启动本节点。
// ============================================================================

// ----------------------------------------------------------------------------
// 坐标系与外参约定（两个方案节点一致，务必遵守）：
//   T_wl : LiDAR -> 世界，P_world = T_wl · P_lidar（Super-LIO 的 lio/odom 直接给）
//   T_lc : Camera -> LiDAR，P_lidar = T_lc · P_camera（从 avia_lvi.yaml 的 Rcl/Pcl 构造）
//   T_cl : LiDAR -> Camera，P_camera = T_cl · P_lidar = T_lc 的逆
//   T_wc : Camera -> 世界，P_world = T_wc · P_camera = T_wl · T_lc（视觉模块接口用它）
//   T_cw : 世界 -> Camera = T_wc 的逆（colorizer 的 Options.T_cl 按实现语义即 T_cw）
//
// 已知坑（供 flash 模型编码时注意）：
//   * Super-LIO 的 lio/odom 的 child_frame_id 标的是 "imu"，但状态实为 LiDAR 位姿
//     （IMU 数据被先旋到雷达系），所以把它当 T_wl 用，不要当 IMU 位姿。
//   * ColorizerPipeline::Options.T_cl 的注释写的是 "LiDAR->Camera"，但实现里它作用在
//     世界系点上（P_C = T_cl · P_W），语义实际是 T_cw（世界->相机）。若相机位姿逐帧
//     变化，需要每帧重建 ColorizerPipeline 或修正 colorizer 库，切勿当成固定外参。
// ----------------------------------------------------------------------------

/// 方案 A 视觉辅助节点。
class VisualANode : public rclcpp::Node {
 public:
  /// @param options rclcpp 节点选项（launch 里透传参数文件）
  explicit VisualANode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// 退出时把累积的彩色地图再落盘一次（运行中用定时器周期保存）。
  ~VisualANode() override;

 private:
  // ---- 订阅回调 ------------------------------------------------------------
  /// 图像回调：去畸变 -> 缩放 -> 灰度 -> 驱动一帧主流程。
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  /// 里程计回调：更新当前 T_wl，并累积位姿采样（供彩色点云运动补偿）。
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  /// 退化签名回调：Super-LIO 每帧观测 Hessian 的退化条件数与最弱方向（世界系）。
  /// 布局：[cond6, axes, active, cond_t, evt_min, ux, uy, uz]
  void onDegeneracy(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg);

  /// 点云回调：累积局部地图点（世界系），并保留当前扫描（供深度关联）。
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  /// 机体系（实为 LiDAR 系）点云回调：仅用于彩色点云。
  /// Super-LIO 的 lio/body/cloud 发布去畸变后的当帧扫描（PointXYZI，LiDAR 系），
  /// 比把 lio/cloud_world 反变换更直接，也避免重复运动补偿。
  void onBodyCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  // ---- 主流程 --------------------------------------------------------------
  /// 每帧图像触发一次：前端跟踪 -> 深度关联 -> 重投影 -> 门控 -> 视觉更新。
  void processVisualFrame(const cv::Mat& gray, double stamp);

  /// 彩色点云流水线：运动补偿 -> 投影 -> 遮挡 -> RGB 赋值 -> 发布。
  ///
  /// 时间基准约定（2026-11 修复，已用离线重放实验确认）：
  ///   t_cloud  = body_stamp_（扫描结束时刻，点已去畸变到该时刻）
  ///   t_pair   = 配对图像 header.stamp，只用于"选哪一帧图像"（实测 |t_pair-t_cloud| < 7 ms）
  ///   t_img    = t_pair + img_time_offset_，图像内容真正对应的采集时刻，
  ///              只用于"取哪一时刻的相机位姿"
  /// 三者不能混用：若把 offset 也加到配对时刻上，0.1 s 会超出同步窗口而整帧被丢弃。
  ///
  /// 由于 t_img 通常晚于 t_cloud（本数据集 0.1 s），调用本函数时该时刻的位姿往往尚未
  /// 收到，因此这里只负责"入队"，真正着色由 drainPendingColorization() 在位姿到齐后完成。
  void runColorizer(const sensor_msgs::msg::PointCloud2& cloud_msg);

  /// 一帧待着色数据：点云 + 配对图像 + 两个时刻。
  struct PendingColorFrame {
    std::vector<Eigen::Vector3f> points_lidar;  ///< LiDAR 系点（已去畸变到 t_cloud）
    double t_cloud{0.0};                        ///< 点云时刻（扫描结束）
    double t_pair{0.0};                         ///< 图像 header 时刻（配对用）
    double t_img{0.0};                          ///< 图像真实采集时刻（取位姿用）
    cv::Mat image;                              ///< 去畸变后的 BGR 图像（cv::Mat 引用计数，浅拷贝安全）
    std::chrono::steady_clock::time_point queued_at;  ///< 入队时刻，用于超时丢弃
  };

  /// 尝试为队列中的帧着色：位姿到齐的立即处理，等待超时的丢弃。
  void drainPendingColorization();

  /// 对单帧执行着色；位姿尚未到齐（未来时刻）时返回 false 表示"稍后重试"。
  bool colorizeFrame(const PendingColorFrame& frame);

  /// 在位姿缓存中按时间插值出 T_wl（与彩色流水线使用同一套插值规则）。
  /// 注意：不允许"用最新采样顶替未来时刻"，否则 t_img = t_cloud + 0.1 s 会退化成
  /// 用 t_cloud 的位姿，等于白改。故查询时刻晚于最新采样时直接返回 false。
  /// @return 缓存不足 / 相邻采样间隔超限 / 未来时刻无采样时返回 false
  bool poseAtStamp(double stamp, Eigen::Matrix4d& T_wl) const;

  /// 同 poseAtStamp 的插值规则，但只查修正轨迹缓存 own_pose_cache_
  /// （状态回灌 update.reinject=true 时用作视觉基准位姿）。
  bool poseAtStampCorrected(double stamp, Eigen::Matrix4d& T_wl) const;

  // ---- 辅助 ----------------------------------------------------------------
  /// 图像去畸变（保持输入分辨率）。
  /// 说明：bag 内的 /left_camera/image 已经是 640x512（录制时已按 scale=0.5 缩放），
  /// 而 intrinsics_ 也是 640x512 下的内参，所以这里只做 undistort、不再 resize。
  /// 内部用 initUndistortRectifyMap 缓存映射表，避免每帧重算。
  cv::Mat rectifyBgr(const cv::Mat& bgr) const;

  /// 从 Rcl/Pcl 构造 T_lc（Camera->LiDAR）。Rcl/Pcl 来自 avia_lvi.yaml。
  /// Rcl 按行主序读入 3x3；Pcl 为相机光心在 LiDAR 系下的坐标。
  /// 注意：非 const（内部需调用 declare_parameter 读取 Rcl/Pcl）。
  Eigen::Matrix4d makeTlcFromRclPcl();

  /// 位姿左扰动应用：T' = Exp(dx) · T，dx 为 6 维 se3（[φ; ρ]，φ 在前）。
  /// 旋转块 = Exp(φ)·R，平移块 = Exp(φ)·p + ρ（与 reprojection_constraint 的
  /// 雅可比约定一致：δ 为相机位姿左扰动，旋转在前）。
  Eigen::Matrix4d applyLeftPerturbation(const Eigen::Matrix4d& T,
                                        const Eigen::VectorXd& dx) const;

  /// 当前退化方向（世界系单位向量）与严重度，供按方向分配视觉权限。
  /// 数据源是 Super-LIO（ros2 分支）已验证的退化检测：degen_trans_only=1 模式下
  /// 对观测 Hessian 平移块做特征分解，最弱特征向量即退化法向。
  /// @return false 表示没有可用的 LIO 退化签名（超时/未开启检测）
  bool degeneracySignature(double& severity, Eigen::Vector3d& dir_w) const;

  /// 把 ColoredPoint 结果打包成 sensor_msgs::PointCloud2（字段 x/y/z/rgb）并发布。
  void publishColoredCloud(const std::vector<colorizer::ColoredPoint>& points,
                           const builtin_interfaces::msg::Time& stamp);

  /// 把本帧上色点变换到世界系并累积到 colored_map_（供离线保存 PCD）。
  /// 位姿取与彩色流水线一致的插值结果（body_stamp_ 为当帧扫描时间）。
  void accumulateColoredMap(const std::vector<colorizer::ColoredPoint>& points,
                            double stamp);

  /// 把已累积的世界系彩色地图写成二进制 PCD。
  void saveColoredMap();

  /// 发布修正后的里程计与轨迹。
  void publishOdomAndPath(const Eigen::Matrix4d& T_wl,
                          const builtin_interfaces::msg::Time& stamp);

  /// 持久偏移的"泄漏"：把 pose_offset_ 按时间做各向异性衰减（见 .cpp 说明）。
  /// 与退化程度和方向绑定：LiDAR 看得住的方向（垂直 u、以及健康时的所有方向）
  /// 衰减到 0，只有"沿退化法向且确实退化"时几乎不衰减（允许累积视觉约束）。
  /// @param dt        距上次视觉帧的时间（秒）
  /// @param severity  Super-LIO 给出的退化严重度 0~1
  /// @param dir_w     退化法向（世界系，可为零向量表示无方向信息）
  void decayPoseOffset(double dt, double severity, const Eigen::Vector3d& dir_w);

  // ---- 参数 ----------------------------------------------------------------
  /// 从参数服务器读取全部配置并初始化各算法模块。
  void loadParameters();

  // ---- 算法模块（库对象） --------------------------------------------------
  KltFrontend::Options klt_options_;
  std::unique_ptr<KltFrontend> frontend_;

  LidarDepthAssociation::Options depth_options_;
  std::unique_ptr<LidarDepthAssociation> depth_assoc_;

  ReprojectionConstraint::Options reproj_options_;
  std::unique_ptr<ReprojectionConstraint> reproj_;

  DegeneracyGate::Options gate_options_;
  std::unique_ptr<DegeneracyGate> gate_;

  VisualUpdate::Options update_options_;
  std::unique_ptr<VisualUpdate> visual_update_;

  colorizer::ColorizerPipeline::Options colorizer_options_;
  std::unique_ptr<colorizer::ColorizerPipeline> colorizer_;

  // ---- 相机/外参/时间 ------------------------------------------------------
  colorizer::CameraIntrinsics intrinsics_;  ///< 相机内参（scale 0.5 后）
  std::vector<double> distortion_;          ///< 畸变系数 [d0,d1,d2,d3]
  double scale_{0.5};                       ///< 图像缩放（1280x1024 -> 640x512）
  Eigen::Matrix4d T_lc_{Eigen::Matrix4d::Identity()};  ///< Camera->LiDAR 外参
  Eigen::Matrix4d T_wl_{Eigen::Matrix4d::Identity()};  ///< 当前 LiDAR 位姿
  /// 图像真实采集时刻相对 header.stamp 的偏移（秒）。本数据集实测最优值 0.1 s
  /// （与官方 avia.yaml 的 img_time_offset 一致）：offset 扫描下 1 cm 体素颜色标准差
  /// 在 0.10 处取极小值 7.1（0.00 时为 15.4，官方 PCD 为 5.9）。
  /// 只用于取相机位姿，不参与图像-点云配对。
  double img_time_offset_{0.0};

  // ---- 数据缓存 ------------------------------------------------------------
  /// 图像缓存（最近 N 帧），供彩色点云按扫描窗口对齐。
  std::vector<colorizer::TimedImage> image_cache_;
  /// 位姿采样缓存（T_wl），供彩色点云逐点运动补偿。
  std::vector<colorizer::PoseSample> pose_cache_;
  /// LiDAR 局部地图点（世界系），由 lio/cloud_world 累积。
  std::vector<Eigen::Vector3d> local_map_;
  /// 当前扫描点（世界系），局部地图稀疏时的兜底。
  std::vector<Eigen::Vector3d> sweep_;
  /// 当前扫描点（LiDAR 系），仅用于彩色点云（来自 lio/body/cloud）。
  std::vector<Eigen::Vector3f> body_points_;
  double body_stamp_{0.0};
  /// 最近一帧 PointCloud2（缓存给 runColorizer 使用）。
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_cloud_;

  /// 去畸变映射表（lazy 构建；cv::remap 需要非 const 输出，故声明 mutable）。
  mutable cv::Mat rectify_map1_;
  mutable cv::Mat rectify_map2_;
  mutable bool rectify_maps_ready_{false};

  // ---- 缓存容量（防止长序列运行内存/耗时膨胀） -----------------------------
  std::size_t max_image_cache_{30};   ///< 图像缓存帧数（colorizer 只需当前附近）
  std::size_t max_pose_samples_{300}; ///< 位姿采样数
  std::size_t max_map_points_{60000}; ///< 局部地图点上限（深度关联会逐点投影）
  int map_append_stride_{3};          ///< 局部地图抽稀步长（降计算量）

  // ---- 订阅器 / 发布器 -----------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr body_cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr degen_sub_;

  // ---- LiDAR 退化签名（来自 Super-LIO 的观测 Hessian） ----------------------
  // Super-LIO（ros2 分支）已有并验证过的退化检测负责"判退化 + 给方向"，
  // 本节点只消费 lio/degeneracy，不自建代理指标。
  mutable std::mutex degen_mutex_;
  std::string degen_topic_;        ///< 订阅的退化签名话题
  superlivo_common::DegeneracySignature degen_sig_;  ///< 最新签名（解析/映射见 common）
  double degen_recv_time_{-1.0};   ///< 本地接收时刻（秒），用于判超时
  bool degen_stale_warned_{false}; ///< 缺签名只告警一次
  bool degen_seen_{false};         ///< 首帧签名打印一次，便于确认链路
  double gate_cond_lo_{superlivo_common::DegeneracyScale{}.cond_lo};
  double gate_cond_hi_{superlivo_common::DegeneracyScale{}.cond_hi};
  double degen_timeout_{superlivo_common::DegeneracyScale{}.timeout_sec};

  /// 视觉修正的持久偏移：T_wl_out = pose_offset_ · T_wl_lio。
  /// 没有它，每帧修正都会被下一条 lio/odom 覆盖，视觉永远只是"单帧探针"。
  /// 注意它是**带泄漏**的累积器：只在 LiDAR 看不住的方向上长期存在，
  /// 否则会逐帧把视觉噪声积分成随机游走（见 decayPoseOffset）。
  Eigen::Matrix4d pose_offset_{Eigen::Matrix4d::Identity()};
  bool offset_decay_en_{true};      ///< 是否启用持久偏移的衰减
  /// 状态回灌：视觉 dx 写回 own_pose_cache_ 尾部采样，LIO 帧间增量挂接在修正轨迹上，
  /// 替代"持久偏移 pose_offset_ 左乘原始轨迹"的模式。
  bool reinject_{false};
  Eigen::Matrix4d T_lio_prev_raw_{
      Eigen::Matrix4d::Identity()};  ///< 上一条原始 lio/odom（增量挂链用）
  bool lio_prev_raw_valid_{false};
  double offset_decay_rate_{1.0};   ///< 健康方向/旋转的衰减率（1/s，τ≈1 s）
  double last_visual_stamp_{-1.0};  ///< 上次视觉帧时刻（算衰减 dt 用）
  double last_odom_stamp_{-1.0};    ///< 最新一条 lio/odom 的时刻（回退取位姿时用于打戳）

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // ---- 运行状态 ------------------------------------------------------------
  bool params_loaded_{false};
  nav_msgs::msg::Path path_msg_;  ///< 累积轨迹

  // ---- 彩色地图落盘（PCD） -------------------------------------------------
  /// 世界系彩色点（直接累积成 PCL 点云，落盘时无需再复制一遍）。
  pcl::PointCloud<pcl::PointXYZRGB> colored_map_;
  /// 逐体素多帧 RGB 时间中位数（pcd.median_voxel_size>0 时启用，替代 append）。
  /// 对同一 1cm 格子的最近 6 帧颜色取中位，椒盐噪声天然被压制，不模糊边缘。
  colorizer::VoxelColorMedian median_map_;
  std::string pcd_save_dir_{"/home/ywj/slam_data/pcd"};  ///< 输出目录
  double pcd_save_interval_sec_{30.0};                   ///< 周期保存间隔（秒）
  /// 地图点数上限：全量点云约 2 万点/帧、上色后约 1 万点/帧，1 万点/秒 →
  /// 2000 万点可覆盖约 30 分钟，内存约 320 MB（PointXYZRGB 16 B/点）。
  std::size_t max_colored_map_points_{20000000};
  bool pcd_capacity_warned_{false};
  std::size_t pcd_saved_points_{0};                      ///< 上次落盘时的点数（无新增则跳过）
  rclcpp::TimerBase::SharedPtr pcd_save_timer_;          ///< 周期落盘定时器

  // ---- 彩色时间基准诊断（2026-11 修复新增） -------------------------------
  /// 已着色过的 dense 帧时间戳：同一帧点云只允许被一帧图像着色一次
  /// （否则同一批点会被不同时刻的图像重复染色，颜色被"拖影"）。
  double last_colored_cloud_stamp_{-1.0};
  /// 待着色队列：t_img 时刻的位姿通常要等 0.1 s 才发布，收到后由 drainPendingColorization 处理。
  std::deque<PendingColorFrame> pending_color_;

  /// 待配对图像的 dense 云队列。
  /// bag2（1280x800 图像）实测图像回调比 dense 云慢约 1 帧（97.7 ms）：
  /// 云到达时对应图像还没进缓存，直接配对会整帧误判"无图像"丢弃。
  /// 改为云先入队，图像到齐后再配对（drainPendingClouds）。
  struct PendingCloud {
    std::vector<Eigen::Vector3f> points_lidar;
    double t_cloud{0.0};
    std::chrono::steady_clock::time_point queued_at;
  };
  std::deque<PendingCloud> pending_clouds_;

  /// 遍历等待图像的 dense 云：能配到图的转入 pending_color_，超时的丢弃。
  void drainPendingClouds();

  /// 着色用哪条轨迹：false=/lio/odom（Super-LIO 原始），true=本节点视觉修正后的 odom。
  bool color_use_own_odom_{false};
  /// 视觉链路是否用"图像曝光时刻"的插值位姿（visual.use_exposure_pose）。
  /// 关掉即回到旧行为（直接用最新一条 lio/odom），用于判别实验对比 rmse。
  bool use_exposure_pose_{true};
  /// 本节点修正位姿缓存（按图像 header 时刻），color_use_own_odom_=true 时供插值。
  std::vector<colorizer::PoseSample> own_pose_cache_;

  std::size_t stat_colorizer_calls_{0};  ///< runColorizer 调用次数
  std::size_t stat_colored_frames_{0};   ///< 真正完成着色的 dense 帧数
  std::size_t stat_drop_no_image_{0};    ///< 丢弃：等待超时仍没有匹配图像
  std::size_t stat_drop_pose_{0};        ///< 丢弃：等待超时后 t_cloud / t_img 位姿仍不可用
  std::size_t stat_deferred_{0};         ///< 因位姿未到齐而延后处理的帧数
  std::size_t stat_skip_duplicate_{0};   ///< 跳过：同一 dense 帧重复调用
  std::size_t stat_drop_accum_{0};       ///< 丢弃：累积到世界系时位姿插值失败（不再回退最新位姿）
};

}  // namespace superlivo_a
