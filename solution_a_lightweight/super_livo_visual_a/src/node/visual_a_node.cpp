#include "super_livo_visual_a/node/visual_a_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

namespace superlivo_a {

namespace {
/// 位姿历史缓冲允许的最大相邻采样间隔（秒）。
/// 注意：这与 color.sync_max_offset（图像-点云同步窗口）是两件事，不能混用。
/// /lio/odom 约 10 Hz（间隔 0.1 s），这里留 5 倍余量，仅用于拒绝真正的空洞。
constexpr double kPoseHistoryMaxGapSec = 0.5;

/// 待着色帧最长等待时间（秒）。t_img 一般比 t_cloud 晚 img_time_offset（0.1 s），
/// 等 1 s 还拿不到位姿说明确实没有后续 odom（例如 bag 结束），此时丢弃并计数。
constexpr double kPendingColorTimeoutSec = 1.0;

/// 待着色队列上限，防止 odom 长时间缺失时内存膨胀。
constexpr std::size_t kMaxPendingColorFrames = 8;

/// 等图像队列：上限与超时（秒）。图像回调比 dense 云慢约 1 帧属正常，
/// 队列 6 帧≈0.6 s 足够吸收；超过 1 s 配不到图说明相机真断流。
constexpr std::size_t kMaxPendingClouds = 6;
constexpr double kPendingCloudTimeoutSec = 1.0;

/// double 秒 -> ROS 时间戳（bag 的时间戳为传感器时钟，秒值 ~9.4e8，仍在 int32 范围内）。
builtin_interfaces::msg::Time toRosStamp(double sec) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(std::floor(sec));
  stamp.nanosec = static_cast<uint32_t>((sec - static_cast<double>(stamp.sec)) * 1e9);
  return stamp;
}

/// so(3) 反对称矩阵。
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return m;
}

/// so(3) 指数映射（罗德里格斯公式）。
Eigen::Matrix3d so3Exp(const Eigen::Vector3d& w) {
  const double theta = w.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity() + skew(w);
  }
  return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
}
}  // namespace

VisualANode::VisualANode(const rclcpp::NodeOptions& options)
    : Node("super_livo_visual_a", options) {
  // ---- 先声明 topic 参数（订阅/发布器创建前需要） -------------------------
  const std::string image_topic = this->declare_parameter<std::string>("image_topic", "/left_camera/image");
  const std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "lio/odom");
  const std::string cloud_topic = this->declare_parameter<std::string>("cloud_topic", "lio/cloud_world");
  const std::string body_cloud_topic =
      this->declare_parameter<std::string>("body_cloud_topic", "lio/cloud_body_dense");

  // ---- 创建订阅器 ---------------------------------------------------------
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&VisualANode::onImage, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&VisualANode::onOdom, this, std::placeholders::_1));
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&VisualANode::onCloud, this, std::placeholders::_1));
  body_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      body_cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&VisualANode::onBodyCloud, this, std::placeholders::_1));
  degen_topic_ = this->declare_parameter<std::string>("degen_topic", "lio/degeneracy");
  degen_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      degen_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&VisualANode::onDegeneracy, this, std::placeholders::_1));

  // ---- 创建发布器 ---------------------------------------------------------
  colored_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "super_livo_visual_a/colored_cloud", rclcpp::SensorDataQoS());
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "super_livo_visual_a/odom", rclcpp::SensorDataQoS());
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "super_livo_visual_a/path", rclcpp::SensorDataQoS());

  // ---- 读取其余参数并初始化算法模块 ---------------------------------------
  loadParameters();

  // 周期性把累积的彩色地图落盘（默认 10 s 一次，覆盖写同一个文件）。
  // 落盘放在独立定时器里，避免在图像回调中做文件 IO 而拖慢视觉帧率。
  // <=0 视为"只在退出时存一次"：中位数地图的落盘是全表遍历，
  // 若每秒执行会占满单线程 executor，导致图像/点云回调积压。
  if (pcd_save_interval_sec_ > 0.0) {
    pcd_save_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(pcd_save_interval_sec_)),
        std::bind(&VisualANode::saveColoredMap, this));
  }

  path_msg_.header.frame_id = "world";
  RCLCPP_INFO(this->get_logger(), "VisualANode 初始化完成");
}

VisualANode::~VisualANode() {
  // 收尾：bag 结束时队列里可能还有等图像/等位姿的帧，尽量把它们补完再落盘。
  drainPendingClouds();
  drainPendingColorization();
  // 退出（Ctrl+C / SIGTERM 触发 rclcpp::spin 返回）时再存一次，保证短时长运行也有结果。
  saveColoredMap();
  // 时间基准诊断汇总：用于判断丢帧到底是"没图"还是"没位姿"。
  RCLCPP_INFO(this->get_logger(),
              "[彩色统计] 调用=%zu 实际着色帧=%zu 丢弃(无图像)=%zu 丢弃(等位姿超时)=%zu "
              "丢弃(累积无位姿)=%zu 跳过(重复帧)=%zu 延后等待次数=%zu 未完成=%zu 地图点数=%zu",
              stat_colorizer_calls_, stat_colored_frames_, stat_drop_no_image_,
              stat_drop_pose_, stat_drop_accum_, stat_skip_duplicate_, stat_deferred_,
              pending_color_.size(), colored_map_.size());
}

void VisualANode::loadParameters() {
  // ==========================================================================
  // 相机内参与畸变（来自 camera_pinhole.yaml，scale 0.5 后）
  // ==========================================================================
  scale_ = this->declare_parameter<double>("scale", 0.5);
  intrinsics_.width = this->declare_parameter<int>("cam_width", 640);
  intrinsics_.height = this->declare_parameter<int>("cam_height", 512);
  intrinsics_.fx = this->declare_parameter<double>("cam_fx", 646.78);
  intrinsics_.fy = this->declare_parameter<double>("cam_fy", 646.66);
  intrinsics_.cx = this->declare_parameter<double>("cam_cx", 313.46);
  intrinsics_.cy = this->declare_parameter<double>("cam_cy", 261.40);
  distortion_ = this->declare_parameter<std::vector<double>>(
      "cam_d", std::vector<double>{-0.076160, 0.123001, -0.00113, 0.000251});

  // ==========================================================================
  // 相机-LiDAR 外参（来自 avia_lvi.yaml 的 Rcl/Pcl）
  // ==========================================================================
  T_lc_ = makeTlcFromRclPcl();

  // 图像真实采集时刻 = header.stamp + 该偏移（秒），仅用于取相机位姿。
  // 默认值与官方 avia.yaml 一致（0.1 s），本数据集已通过 offset 扫描验证最优。
  img_time_offset_ = this->declare_parameter<double>("img_time_offset", 0.1);

  // ==========================================================================
  // 前端 KltFrontend
  // ==========================================================================
  klt_options_.max_features = this->declare_parameter<int>("klt.max_features", 150);
  klt_options_.pyramid_levels = this->declare_parameter<int>("klt.pyramid_levels", 3);
  klt_options_.min_distance = this->declare_parameter<double>("klt.min_distance", 10.0);
  klt_options_.fb_error_threshold = this->declare_parameter<double>("klt.fb_error_threshold", 1.0);
  klt_options_.min_track_length = this->declare_parameter<int>("klt.min_track_length", 3);
  klt_options_.trace_window = this->declare_parameter<int>("klt.trace_window", 5);
  klt_options_.use_fast = this->declare_parameter<bool>("klt.use_fast", true);
  klt_options_.fast_threshold = this->declare_parameter<int>("klt.fast_threshold", 20);
  klt_options_.quality_level = this->declare_parameter<double>("klt.quality_level", 0.01);
  klt_options_.lk_window = this->declare_parameter<int>("klt.lk_window", 21);
  frontend_ = std::make_unique<KltFrontend>(klt_options_);

  // ==========================================================================
  // 深度关联 LidarDepthAssociation
  // ==========================================================================
  depth_options_.fx = intrinsics_.fx;
  depth_options_.fy = intrinsics_.fy;
  depth_options_.cx = intrinsics_.cx;
  depth_options_.cy = intrinsics_.cy;
  depth_options_.image_width = intrinsics_.width;
  depth_options_.image_height = intrinsics_.height;
  depth_options_.pixel_radius = this->declare_parameter<int>("depth.pixel_radius", 3);
  depth_options_.search_radius = this->declare_parameter<double>("depth.search_radius", 0.3);
  depth_options_.min_depth = this->declare_parameter<double>("depth.min_depth", 0.5);
  depth_options_.max_depth = this->declare_parameter<double>("depth.max_depth", 80.0);
  depth_options_.max_neighbors = this->declare_parameter<int>("depth.max_neighbors", 5);
  depth_options_.anchor_window = this->declare_parameter<int>("depth.anchor_window", 5);
  depth_assoc_ = std::make_unique<LidarDepthAssociation>(depth_options_);

  // ==========================================================================
  // 重投影约束 ReprojectionConstraint
  // ==========================================================================
  reproj_options_.fx = intrinsics_.fx;
  reproj_options_.fy = intrinsics_.fy;
  reproj_options_.cx = intrinsics_.cx;
  reproj_options_.cy = intrinsics_.cy;
  reproj_options_.huber_delta = this->declare_parameter<double>("reproj.huber_delta", 1.0);
  reproj_options_.min_constraints = this->declare_parameter<int>("reproj.min_constraints", 8);
  reproj_options_.max_pixel_error = this->declare_parameter<double>("reproj.max_pixel_error", 8.0);
  // 测量噪声模型：没有它等价于 σ=1 px、且认为 LiDAR 深度关联完全准确。
  reproj_options_.sigma_pix = this->declare_parameter<double>("reproj.sigma_pix", 1.5);
  reproj_options_.sigma_anchor = this->declare_parameter<double>("reproj.sigma_anchor", 0.05);
  reproj_ = std::make_unique<ReprojectionConstraint>(reproj_options_);

  // ==========================================================================
  // 退化门控 DegeneracyGate
  // ==========================================================================
  gate_options_.min_tracked_features = this->declare_parameter<int>("gate.min_tracked_features", 30);
  gate_options_.max_reprojection_rmse = this->declare_parameter<double>("gate.max_reprojection_rmse", 2.0);
  gate_options_.min_spread = this->declare_parameter<double>("gate.min_spread", 0.15);
  gate_options_.max_condition = this->declare_parameter<double>("gate.max_condition", 1.0e3);
  gate_options_.assist_weight = this->declare_parameter<double>("gate.assist_weight", 0.3);
  gate_options_.high_weight = this->declare_parameter<double>("gate.high_weight", 0.8);
  // 退化阈值参数化：默认 0.3/0.7（原硬编码）；设 0.0 可让视觉常开做判别实验。
  gate_options_.assist_degeneracy = this->declare_parameter<double>("gate.assist_degeneracy", 0.3);
  gate_options_.high_degeneracy = this->declare_parameter<double>("gate.high_degeneracy", 0.7);
  gate_ = std::make_unique<DegeneracyGate>(gate_options_);

  // 退化严重度映射：cond_t（H_tt 最大/最小特征值）在对数刻度上线性映射到 [0,1]。
  // 本数据集实测：健康段 cond_t≈25-40，必须映射为 sev≈0，否则先验在健康段
  // 就被无端松开（旧默认 10/100 使 cond_t=33 映射成 sev≈0.5）。
  // 完全退化（n_effect=0，cond=1e12）由 axes/active 硬触发直接置 1，不走该映射。
  gate_cond_lo_ = this->declare_parameter<double>(
      "gate.cond_lo", superlivo_common::DegeneracyScale{}.cond_lo);
  gate_cond_hi_ = this->declare_parameter<double>(
      "gate.cond_hi", superlivo_common::DegeneracyScale{}.cond_hi);
  degen_timeout_ = this->declare_parameter<double>(
      "gate.degen_timeout", superlivo_common::DegeneracyScale{}.timeout_sec);

  // ==========================================================================
  // 视觉更新 VisualUpdate
  // ==========================================================================
  update_options_.enable = this->declare_parameter<bool>("update.enable", true);
  update_options_.max_dx_norm = this->declare_parameter<double>("update.max_dx_norm", 0.5);
  // LiDAR 先验信息量（贝叶斯正则），使门控权重 w 真正起作用。
  update_options_.prior_rot_sigma = this->declare_parameter<double>("update.prior_rot_sigma", 0.02);
  update_options_.prior_pos_sigma = this->declare_parameter<double>("update.prior_pos_sigma", 0.05);
  // 退化时把位置先验做成各向异性：退化方向松开、垂直方向收紧。
  update_options_.directional_prior_en =
      this->declare_parameter<bool>("update.directional_prior_en", true);
  update_options_.prior_pos_sigma_healthy =
      this->declare_parameter<double>("update.prior_pos_sigma_healthy", 0.005);
  update_options_.prior_pos_sigma_degen =
      this->declare_parameter<double>("update.prior_pos_sigma_degen", 1.0);
  // 持久偏移的衰减（健康方向 τ≈1/rate 秒）。视觉更新只在退化帧被接受，
  // 因此偏移若只增不减，健康期就永远冻结在最后一次退化时累积的值。
  offset_decay_en_ = this->declare_parameter<bool>("update.offset_decay_en", true);
  offset_decay_rate_ = this->declare_parameter<double>("update.offset_decay_rate", 1.0);
  // 状态回灌（取代"持久偏移"模式）：把每帧视觉 dx 直接写回修正轨迹采样，
  // 之后 LIO 帧间增量挂接在修正轨迹上，约束跨帧累积且不会被原始轨迹抹掉。
  reinject_ = this->declare_parameter<bool>("update.reinject", true);
  visual_update_ = std::make_unique<VisualUpdate>(update_options_);

  // ==========================================================================
  // 彩色点云 ColorizerPipeline
  // ==========================================================================
  colorizer_options_.intrinsics = intrinsics_;
  // 着色轨迹源：默认 false=/lio/odom；true=本节点视觉修正后的 odom（own_pose_cache_）。
  color_use_own_odom_ = this->declare_parameter<bool>("color.use_own_odom", false);
  // 视觉链路位姿时刻：true=图像曝光时刻插值（正确），false=最新一条 lio/odom（旧行为）。
  use_exposure_pose_ = this->declare_parameter<bool>("visual.use_exposure_pose", true);
  // TODO(flash)：colorizer 的 Options.T_cl 语义实为 T_cw（世界->相机，逐帧变化）。
  //  此处先置单位阵，runColorizer 里按每帧相机位姿重建 pipeline 或修正库后再填。
  colorizer_options_.T_cl = Eigen::Matrix4d::Identity();
  colorizer_options_.sync.max_offset = this->declare_parameter<double>("color.sync_max_offset", 0.02);
  colorizer_options_.sync.use_exposure_mid = this->declare_parameter<bool>("color.use_exposure_mid", true);
  colorizer_options_.occlusion.depth_tolerance = this->declare_parameter<double>("color.depth_tolerance", 0.1);
  colorizer_options_.occlusion.check_depth_edges = this->declare_parameter<bool>("color.check_depth_edges", true);
  colorizer_options_.occlusion.edge_threshold = this->declare_parameter<double>("color.edge_threshold", 0.5);
  colorizer_options_.rgb.bilinear = this->declare_parameter<bool>("color.bilinear", true);
  colorizer_options_.rgb.keep_best_quality = this->declare_parameter<bool>("color.keep_best_quality", true);
  colorizer_options_.rgb.min_quality = this->declare_parameter<float>("color.min_quality", 0.2F);
  colorizer_ = std::make_unique<colorizer::ColorizerPipeline>(colorizer_options_);

  // ==========================================================================
  // 彩色地图落盘（把跑完的彩色点云输出为 PCD）
  // ==========================================================================
  pcd_save_dir_ = this->declare_parameter<std::string>("pcd.save_dir", "/home/ywj/slam_data/pcd");
  pcd_save_interval_sec_ = this->declare_parameter<double>("pcd.save_interval_sec", 30.0);
  max_colored_map_points_ = static_cast<std::size_t>(
      this->declare_parameter<int>("pcd.max_points", 20000000));
  // 时间中位数体素边长（米）。>0 时对同一格子的最近 6 帧观测取逐通道中位数，
  // 替代直接 append；中位数对椒盐鲁棒（6 帧中 2 帧错色仍可正确恢复）。
  median_map_.set_voxel_size(
      static_cast<float>(this->declare_parameter<double>("pcd.median_voxel_size", 0.01)));

  params_loaded_ = true;
}

void VisualANode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  if (!params_loaded_) {
    return;
  }
  // ---- 图像 -> BGR -> 去畸变 ----------------------------------------------
  // 用 toCvCopy 拿到「拥有数据」的 Mat：toCvShare 得到的 Mat 只是引用了消息缓冲区，
  // 直接把 Mat 存进缓存会变成悬垂指针（消息释放后图像数据失效）。
  cv::Mat bgr;
  try {
    bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "cv_bridge 转换失败: %s", e.what());
    return;
  }
  if (bgr.empty()) {
    return;
  }
  const cv::Mat rectified = rectifyBgr(bgr);
  if (rectified.empty()) {
    return;
  }
  cv::Mat gray;
  cv::cvtColor(rectified, gray, cv::COLOR_BGR2GRAY);

  // 图像 header 时刻：与 dense 点云 header 处于同一时钟基准（实测差 < 7 ms），
  // 因此**只**用它做"图像-点云配对"。图像真实采集时刻（header + img_time_offset_）
  // 由 runColorizer 单独计算，仅用于取相机位姿，绝不能把两者混成一个时间戳。
  const double stamp = rclcpp::Time(msg->header.stamp).seconds();

  // 缓存图像帧，供彩色点云对齐使用（缓存去畸变后的彩色图，尺寸与内参一致）。
  colorizer::TimedImage timed;
  timed.image = rectified;
  timed.exposure_mid_stamp = stamp;
  timed.arrival_stamp = stamp;
  timed.valid = true;
  image_cache_.push_back(timed);
  if (image_cache_.size() > max_image_cache_) {
    image_cache_.erase(image_cache_.begin(),
                       image_cache_.begin() + (image_cache_.size() - max_image_cache_));
  }

  processVisualFrame(gray, stamp);
}

void VisualANode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  // ---- 解析 T_wl（child_frame_id 标 "imu"，但 Super-LIO 状态就是 LiDAR 位姿） --
  const auto& pose = msg->pose.pose;
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1e-9) {
    return;  // 无效四元数
  }
  q.normalize();
  Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
  T_wl.block<3, 3>(0, 0) = q.toRotationMatrix();
  T_wl(0, 3) = pose.position.x;
  T_wl(1, 3) = pose.position.y;
  T_wl(2, 3) = pose.position.z;
  T_wl_ = T_wl;
  last_odom_stamp_ = rclcpp::Time(msg->header.stamp).seconds();

  // 累积位姿采样供彩色点云运动补偿。
  colorizer::PoseSample sample;
  sample.stamp = last_odom_stamp_;
  sample.T_wl = T_wl;
  pose_cache_.push_back(sample);
  if (pose_cache_.size() > max_pose_samples_) {
    pose_cache_.erase(pose_cache_.begin(),
                      pose_cache_.begin() + (pose_cache_.size() - max_pose_samples_));
  }

  // 状态回灌：把本条 LIO 的增量挂接到修正轨迹尾部。
  // dw = T_lio(k) · T_lio(k-1)⁻¹（世界系左乘增量），own(k) = dw · own(k-1)。
  // 视觉 dx 写回 own 尾采样后，下一条 LIO 增量自然继承修正。
  if (reinject_) {
    colorizer::PoseSample own;
    own.stamp = last_odom_stamp_;
    if (lio_prev_raw_valid_ && !own_pose_cache_.empty()) {
      const Eigen::Matrix4d dw = T_wl * T_lio_prev_raw_.inverse();
      own.T_wl = dw * own_pose_cache_.back().T_wl;
    } else {
      own.T_wl = T_wl;  // 首条：直接以原始 LIO 位姿作种子
    }
    T_lio_prev_raw_ = T_wl;
    lio_prev_raw_valid_ = true;
    own_pose_cache_.push_back(own);
    if (own_pose_cache_.size() > max_pose_samples_) {
      own_pose_cache_.erase(own_pose_cache_.begin(),
                            own_pose_cache_.begin() +
                                (own_pose_cache_.size() - max_pose_samples_));
    }
  }
  // 注意：这里只更新位姿缓存，发布统一放在 processVisualFrame（图像帧），
  // 避免 path 里出现重复位姿、也保证发布的是视觉修正后的结果。

  // 新采样到齐后重试待着色帧：图像采集时刻 t_img = t_cloud + img_time_offset，
  // 通常要等到这一帧 odom 发布后才能插值出位姿。
  if (!pending_color_.empty()) {
    drainPendingColorization();
  }
}

void VisualANode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  // ---- 解析世界系点云 -> 当前扫描 + 局部地图 ------------------------------
  // lio/cloud_world 是 Super-LIO 用当帧位姿把 LiDAR 点变换到世界系的结果。
  pcl::PointCloud<pcl::PointXYZI> cloud;
  try {
    pcl::fromROSMsg(*msg, cloud);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "点云解析失败: %s", e.what());
    return;
  }

  sweep_.clear();
  sweep_.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    sweep_.emplace_back(p.x, p.y, p.z);
  }

  // 累积局部地图（抽稀 + 上限裁剪）：深度关联会逐点投影，必须控制规模。
  for (std::size_t i = 0; i < sweep_.size(); i += static_cast<std::size_t>(std::max(1, map_append_stride_))) {
    local_map_.push_back(sweep_[i]);
  }
  if (local_map_.size() > max_map_points_) {
    local_map_.erase(local_map_.begin(),
                     local_map_.begin() + (local_map_.size() - max_map_points_));
  }

  last_cloud_ = msg;
}

void VisualANode::onBodyCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  // lio/body/cloud：去畸变后的当帧扫描，点已表达在 LiDAR 系（Super-LIO 的 body 即雷达系）。
  pcl::PointCloud<pcl::PointXYZI> cloud;
  try {
    pcl::fromROSMsg(*msg, cloud);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "机体系点云解析失败: %s", e.what());
    return;
  }
  body_points_.clear();
  body_points_.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    body_points_.emplace_back(p.x, p.y, p.z);
  }
  body_stamp_ = rclcpp::Time(msg->header.stamp).seconds();

  // 新扫描到达即入队等图像/位姿（bag2 的图像回调比云慢约 1 帧，不能等图像回调才入队）。
  runColorizer(*msg);
}

void VisualANode::processVisualFrame(const cv::Mat& gray, double stamp) {
  // ==========================================================================
  // 主流程（对应 gpt §2.2）：前端跟踪 -> 深度关联 -> 重投影 -> 门控 -> 视觉更新
  // ==========================================================================
  const auto t_start = std::chrono::steady_clock::now();

  // 1) 前端跟踪
  const TrackResult track = frontend_->process(gray, stamp);
  const auto t_front = std::chrono::steady_clock::now();

  // 2) 计算当前相机位姿 T_wc = T_wl · T_lc
  //    关键：必须取**图像曝光时刻**的位姿。此前直接用 T_wl_（最新一条 lio/odom），
  //    两者相差可达 0.05~0.15 s；按 0.5 m/s 平移 + 中等角速度折算就是几厘米的
  //    系统性锚点偏移，在 z≈5 m 处对应 2~3 px，正好是重投影 RMSE 长期卡在
  //    2.8 px、门控（阈值 3.0）整帧全拒的主要原因。
  const double t_capture = stamp + img_time_offset_;
  Eigen::Matrix4d T_wl_img = Eigen::Matrix4d::Identity();
  // 实际用到的位姿所对应的时刻：插值成功就是 t_capture，否则是"最新采样自身的时刻"。
  // 发布时必须按这个时刻打戳，否则等于把旧位姿标成新时刻（隐含 0.1~0.2 s 的系统性
  // 时间错位，直接抬高 ATE）。实测在该数据集上，A 的轨迹与 lio/odom 的最佳对齐滞后
  // 恰为 img_time_offset_，就是这个回退路径造成的。
  double pose_stamp = t_capture;
  if (reinject_) {
    // 回灌模式：基准位姿直接取修正轨迹在曝光时刻的插值，不叠加 pose_offset_。
    if (!poseAtStampCorrected(t_capture, T_wl_img)) {
      if (!own_pose_cache_.empty()) {
        T_wl_img = own_pose_cache_.back().T_wl;
        pose_stamp = own_pose_cache_.back().stamp;
      } else {
        T_wl_img = T_wl_;  // 启动初期尚无采样
        if (last_odom_stamp_ > 0.0) {
          pose_stamp = last_odom_stamp_;
        }
      }
    }
  } else if (!use_exposure_pose_ || !poseAtStamp(t_capture, T_wl_img)) {
    T_wl_img = T_wl_;  // 关闭开关 / 启动初期缓存还不够 / 未来时刻无采样，退回最新值
    if (last_odom_stamp_ > 0.0) {
      pose_stamp = last_odom_stamp_;
    }
  }
  // 2.1) 叠加此前视觉修正的持久偏移：T_wl_base = pose_offset_ · T_wl_lio。
  //      没有这一步，每帧修正都会被下一条 lio/odom 覆盖，视觉永远只是"单帧探针"，
  //      在 LiDAR 退化时完全无法把约束累积成位姿。
  //      （回灌模式下修正已沉淀进 own_pose_cache_，不再维护持久偏移。）
  const Eigen::Matrix4d T_wl_base = reinject_ ? T_wl_img : pose_offset_ * T_wl_img;
  const Eigen::Matrix4d T_wc = T_wl_base * T_lc_;

  // 3) 深度关联：把局部地图与当前扫描喂给深度关联器
  depth_assoc_->setLocalMap(local_map_);
  depth_assoc_->setSweep(sweep_);
  const std::vector<FeatureDepth> depths = depth_assoc_->associate(track.tracked, T_wc);
  const auto t_depth = std::chrono::steady_clock::now();

  // 4) 重投影约束 -> 6x6 法方程
  const auto blocks = reproj_->build(track.tracked, depths, T_wc);
  Eigen::MatrixXd H;
  Eigen::VectorXd b;
  double cost = 0.0;
  reproj_->accumulate(blocks, H, b, cost);
  const auto t_reproj = std::chrono::steady_clock::now();

  // 5) 退化门控：同时评估 LiDAR 退化程度与视觉自身质量（gpt §2.3）
  //    退化程度只用 Super-LIO 的观测 Hessian 签名（ros2 分支已有并验证过的检测，
  //    A 侧不自建代理）。拿不到签名 => 视为不退化、不做方向性加权。
  double degen_severity = 0.0;
  Eigen::Vector3d degen_dir = Eigen::Vector3d::Zero();
  if (!degeneracySignature(degen_severity, degen_dir) && !degen_stale_warned_) {
    degen_stale_warned_ = true;
    RCLCPP_WARN(this->get_logger(),
                "未收到 %s 退化签名（超时 %.1fs）：请给 Super-LIO 开启 "
                "lio.observe.degeneracy_detect_en 且 degen_trans_only=1，"
                "否则退化门控恒不触发。",
                degen_topic_.c_str(), degen_timeout_);
  }
  GateInputs inputs;
  inputs.lidar_degeneracy = degen_severity;
  inputs.tracked_features = static_cast<int>(track.tracked.size());

  // 5.1 特征空间散布度：像素坐标标准差按图像尺寸归一化（0~0.5 量级）。
  double spread = 0.0;
  if (track.tracked.size() >= 2) {
    double mean_u = 0.0;
    double mean_v = 0.0;
    for (const auto& f : track.tracked) {
      mean_u += f.u;
      mean_v += f.v;
    }
    const double inv_n = 1.0 / static_cast<double>(track.tracked.size());
    mean_u *= inv_n;
    mean_v *= inv_n;
    double var_u = 0.0;
    double var_v = 0.0;
    for (const auto& f : track.tracked) {
      var_u += (f.u - mean_u) * (f.u - mean_u);
      var_v += (f.v - mean_v) * (f.v - mean_v);
    }
    const double w = std::max(1, intrinsics_.width);
    const double h = std::max(1, intrinsics_.height);
    spread = 0.5 * (std::sqrt(var_u * inv_n) / w + std::sqrt(var_v * inv_n) / h);
  }
  inputs.spatial_spread = spread;
  inputs.motion_observability = std::min(1.0, spread / std::max(1e-6, gate_options_.min_spread));

  // 5.2 重投影残差 RMS（像素），并分解"系统性偏差 ⊥ 随机噪声"。
  //     rmse² = bias² + noise²；bias 是全体残差的均值模长，
  //     它大说明存在时间戳/外参/深度关联一类的系统误差（可修），
  //     它小而 noise 大则是特征跟踪与地图噪声（只能靠多特征平均）。
  double sum_sq = 0.0;
  int valid_blocks = 0;
  double sum_du = 0.0;
  double sum_dv = 0.0;
  for (const auto& block : blocks) {
    if (block.valid) {
      sum_sq += block.residual.squaredNorm();
      sum_du += block.residual.x();
      sum_dv += block.residual.y();
      ++valid_blocks;
    }
  }
  double bias_px = 0.0;
  inputs.reprojection_rmse =
      valid_blocks > 0 ? std::sqrt(sum_sq / static_cast<double>(valid_blocks)) : 0.0;
  if (valid_blocks > 0) {
    const double inv_n = 1.0 / static_cast<double>(valid_blocks);
    bias_px = std::hypot(sum_du * inv_n, sum_dv * inv_n);
  }

  // 5.3 雅可比条件数：H 的特征值比（越大越病态）。
  //     必须在**无量纲**的矩阵上算：H 的旋转块单位是 1/rad²、平移块是 1/m²，
  //     直接求特征值比得到的是"单位之比"而不是几何病态程度（加了方差归一化后
  //     实测 cond 从 2e4 暴涨到 3e7，把几乎每一帧都误判成病态而拒绝）。
  //     按先验标准差各向缩放 H_norm = S·H·S（S=diag(σ_rot,σ_pos)）后，
  //     特征值就是"该方向相对先验的信息量"，条件数才真正反映某个方向没被约束。
  if (H.rows() == 6 && H.cols() == 6) {
    const double s_rot = std::max(1e-9, update_options_.prior_rot_sigma);
    const double s_pos = std::max(1e-9, update_options_.prior_pos_sigma);
    Eigen::Matrix<double, 6, 1> s;
    s << s_rot, s_rot, s_rot, s_pos, s_pos, s_pos;
    const Eigen::MatrixXd h_norm = (s.asDiagonal() * H * s.asDiagonal()).eval();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(h_norm);
    if (solver.info() == Eigen::Success) {
      const double lmax = std::max(1e-12, solver.eigenvalues()(5));
      const double lmin = std::max(1e-12, solver.eigenvalues()(0));
      inputs.jacobian_condition = lmax / lmin;
    }
  }
  const GateDecision decision = gate_->evaluate(inputs);
  const auto t_gate = std::chrono::steady_clock::now();

  // 6) 视觉更新
  VisualUpdateInput update_input;
  update_input.H = H;
  update_input.b = b;
  update_input.cost = cost;
  // 质量：跟踪成功率 + 残差水平综合（0~1），供 VisualUpdate 与门控权重相乘。
  // 修正：原来用 tracked/detected，而 detected 是"本帧**新检测**角点数"，
  // 稳态跟踪时常年为 0（特征数已满、不再补点），于是 inlier_ratio 恒为 0、
  // quality 被压到 0.5/(1+rmse)≈0.13，且随补点与否抖动 5 倍——权重完全是噪声。
  // 真正反映跟踪质量的是前后向一致性成功率 tracked/(tracked+rejected_fb)。
  const int attempted = static_cast<int>(track.tracked.size()) + track.rejected_fb;
  const double inlier_ratio =
      attempted > 0 ? std::min(1.0, static_cast<double>(track.tracked.size()) / attempted)
                    : 0.0;
  update_input.quality =
      std::clamp(0.5 * inlier_ratio + 0.5 / (1.0 + inputs.reprojection_rmse), 0.0, 1.0);
  update_input.degen_dir_w = degen_dir;
  update_input.degen_severity = degen_severity;
  const VisualUpdateOutput output = visual_update_->apply(update_input, decision);
  const auto t_update = std::chrono::steady_clock::now();

  // 7) 若接受：位姿左扰动修正 T_wc，并把修正沉淀成持久偏移。
  //    持久偏移让视觉在 LiDAR 退化时可以持续累积约束，而不是每帧被 lio/odom 抹掉。
  Eigen::Matrix4d T_wl_out = T_wl_base;
  if (output.accepted) {
    const Eigen::Matrix4d T_wc_corrected = applyLeftPerturbation(T_wc, output.dx);
    T_wl_out = T_wc_corrected * T_lc_.inverse();
    if (reinject_) {
      // 回灌核心：把当帧 dx 写回修正轨迹尾部采样。之后 onOdom 的 LIO 增量
      // 挂接在这个被修正过的位姿上，视觉约束跨帧累积、不被 lio/odom 抹掉。
      if (!own_pose_cache_.empty() &&
          std::abs(own_pose_cache_.back().stamp - pose_stamp) <= 0.15) {
        own_pose_cache_.back().T_wl = T_wl_out;
      }
    } else {
      const Eigen::Matrix4d T_lio_inv = T_wl_img.inverse();
      pose_offset_ = T_wl_out * T_lio_inv;
    }
  }
  // 7.1) 持久偏移的泄漏（见 decayPoseOffset）：即使本帧视觉没有被接受
  //      （LiDAR 健康 -> weight=0 -> apply 直接返回），也要按退化程度与方向
  //      对偏移做衰减，否则健康期偏移被永久冻结，视觉噪声被当成系统偏差带着走。
  {
    const double dt =
        last_visual_stamp_ < 0.0 ? 0.0 : std::clamp(stamp - last_visual_stamp_, 0.0, 0.5);
    last_visual_stamp_ = stamp;
    // 回灌模式下不维护持久偏移、也不衰减：修正已沉淀进 own_pose_cache_，
    // 再对单个采样"回拉"等于把历史帧改回 LIO 轨迹，破坏连续性。
    if (!reinject_) {
      decayPoseOffset(dt, degen_severity, degen_dir);
    }
  }
  if (!reinject_) {
    T_wl_ = T_wl_out;  // 回灌模式 T_wl_ 保持最新原始 lio/odom，供 onOdom 挂链接量
  }

  // 8) 发布修正后里程计与轨迹。时间基准：T_wl_out 对应 **实际取用的位姿时刻**
  //    pose_stamp（插值成功即曝光时刻 t_capture；回退到最新采样时就是该采样的时刻），
  //    必须按它打戳，不能用图像 header.stamp 也不能无条件用 t_capture。
  publishOdomAndPath(T_wl_out, toRosStamp(pose_stamp));
  const auto t_pub = std::chrono::steady_clock::now();

  // 8.5) 缓存本帧修正位姿（按 pose_stamp），供 color.use_own_odom=true 时着色插值。
  //      必须与查询时刻同一基准：poseAtStamp 是按 t_img = header.stamp + img_time_offset_
  //      查的，若这里按别的时刻存，取到的就是错位的位姿。
  //      回灌模式下 own_pose_cache_ 已由 onOdom 逐条 odom 维护，此处不再重复写入。
  if (!reinject_) {
    colorizer::PoseSample own;
    own.stamp = pose_stamp;
    own.T_wl = T_wl_;
    own_pose_cache_.push_back(own);
    if (own_pose_cache_.size() > max_pose_samples_) {
      own_pose_cache_.erase(own_pose_cache_.begin(),
                            own_pose_cache_.begin() +
                                (own_pose_cache_.size() - max_pose_samples_));
    }
  }

  // 9) 若缓存了机体系点云，触发彩色点云（与视觉定位解耦，独立工作）
  if (!body_points_.empty() && last_cloud_) {
    runColorizer(*last_cloud_);
  }
  const auto t_end = std::chrono::steady_clock::now();
  auto ms = [](const std::chrono::steady_clock::time_point& a,
               const std::chrono::steady_clock::time_point& b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };

  // 10) 诊断日志（1 Hz 节流，便于跑 bag 时确认各模块在工作与耗时分布）
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "视觉帧: tracked=%zu/%d n=%d spread=%.3f rmse=%.2f bias=%.2f cond=%.1f deg=%.2f mode=%d w=%.2f %s | "
      "耗时 ms: 前端=%.1f 深度=%.1f 重投影=%.1f 门控=%.1f 更新=%.1f 发布=%.1f 彩色=%.1f "
      "合计=%.1f",
      track.tracked.size(), track.detected, valid_blocks, inputs.spatial_spread,
      inputs.reprojection_rmse, bias_px, inputs.jacobian_condition, inputs.lidar_degeneracy,
      static_cast<int>(decision.mode), decision.visual_weight, decision.reason.c_str(),
      ms(t_start, t_front), ms(t_front, t_depth), ms(t_depth, t_reproj), ms(t_reproj, t_gate),
      ms(t_gate, t_update), ms(t_update, t_pub), ms(t_pub, t_end), ms(t_start, t_end));

  // 10.1) 退化/回灌状态（1 Hz）：确认门控是否真的打开、视觉修正是否在累积。
  //       沿 u 分量是关键：只有在退化法向上的修正才真正补上了 LiDAR 缺失的自由度。
  const double dir_norm = degen_dir.norm();
  const Eigen::Vector3d u_w =
      dir_norm > 1e-6 ? Eigen::Vector3d(degen_dir / dir_norm) : Eigen::Vector3d::Zero();
  const double corr_along_u = output.accepted ? 1e3 * output.dx.tail<3>().dot(u_w) : 0.0;
  // 诊断"累积偏移"：持久偏移模式直接读 pose_offset_；
  // 回灌模式 = 修正轨迹最新采样相对原始 LIO 最新采样的差。
  Eigen::Matrix4d off_disp = pose_offset_;
  if (reinject_ && !own_pose_cache_.empty() && !pose_cache_.empty()) {
    off_disp = own_pose_cache_.back().T_wl * pose_cache_.back().T_wl.inverse();
  }
  const double off_along_u = 1e3 * off_disp.block<3, 1>(0, 3).dot(u_w);
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "退化状态: sev=%.2f dir=(%+.2f %+.2f %+.2f) 修正=%.1fmm(沿u%+.1f) | "
      "持久偏移=%.1fmm(沿u%+.1f) %.2f°",
      degen_severity, degen_dir.x(), degen_dir.y(), degen_dir.z(),
      output.accepted ? 1e3 * output.dx.tail<3>().norm() : 0.0, corr_along_u,
      1e3 * off_disp.block<3, 1>(0, 3).norm(), off_along_u,
      off_disp.block<3, 3>(0, 0).eulerAngles(0, 1, 2).norm() * 180.0 / M_PI);
}

void VisualANode::runColorizer(const sensor_msgs::msg::PointCloud2& cloud_msg) {
  // ==========================================================================
  // 彩色点云（对应 gpt §2.4）：运动补偿 -> 投影 -> 遮挡 -> RGB 赋值
  // ==========================================================================
  // 输入约定：
  //   points_lidar : lio/cloud_body_dense 的当帧扫描（LiDAR 系，已由 Super-LIO 的
  //                  DenseThread 逐点去畸变到扫描结束时刻）
  //   point_stamps : 所有点都等于 body_stamp_（点云已去畸变，无需再做逐点补偿）
  //   images/poses : 本节点缓存（图像为去畸变后的 BGR，位姿为 T_wl）
  //
  // ---- 时间基准（本数据集已用 bag 实测确认，勿按变量名推断） ----------------
  //   t_cloud = body_stamp_ = dense 云 header.stamp = ESKF 状态时刻 ≈ 扫描结束时刻
  //   t_pair  = 图像 header.stamp，只用于"挑哪一帧图像"
  //   t_img   = t_pair + img_time_offset_ = 图像真实采集时刻，只用于"取相机位姿"
  //   实测 |t_pair - t_cloud| = 0.4 ~ 7.0 ms（P99 5.6 ms），所以配对必须在 header 时刻做；
  //   而位姿必须取 t_img。
  //
  //   正确处理：
  //     P_w        = T_wl(t_cloud) · P_l          // 流水线内部用 point_stamps 完成
  //     P_c(t_img) = [T_wl(t_img) · T_lc]^-1 · P_w
  //   即 T_cl 由 t_img 时刻的位姿构造。
  //
  //   为什么需要 offset（实测证据）：用官方轨迹离线重放本管线，对 img_time_offset 做
  //   0~0.15 s 扫描，1 cm 体素颜色标准差在 0.10 s 处取极小值（15.4 -> 7.1），
  //   且该设定下的整图投影 MAE = 23.6/255 与官方 PCD 完全相同；offset=0 时约 13 px
  //   的逐帧系统性错位，跨帧累积即表现为"椒盐噪点"。官方 avia.yaml 里同样是 0.1 s。
  //
  // 注意：ColorizerPipeline::Options.T_cl 的实现语义是 T_cw（P_C = T_cl · P_W）。
  // ==========================================================================
  (void)cloud_msg;
  ++stat_colorizer_calls_;

  if (body_points_.empty()) {
    return;
  }

  const double t_cloud = body_stamp_;

  // (1) 把当前 dense 云放入"等图像"队列（按时间戳去重）。
  //     1280x800 这类大图的回调比 dense 云慢约 1 帧（bag2 实测 97.7 ms），
  //     云到达时对应图像往往还没进缓存；立即配对会把绝大多数帧误判"无图像"丢弃。
  //     改为云先入队，图像/里程计到达时由 drainPendingClouds 补配对。
  const bool already_queued =
      std::abs(t_cloud - last_colored_cloud_stamp_) < 1e-6 ||
      std::any_of(pending_clouds_.begin(), pending_clouds_.end(),
                  [t_cloud](const PendingCloud& c) {
                    return std::abs(c.t_cloud - t_cloud) < 1e-6;
                  }) ||
      std::any_of(pending_color_.begin(), pending_color_.end(),
                  [t_cloud](const PendingColorFrame& f) {
                    return std::abs(f.t_cloud - t_cloud) < 1e-6;
                  });
  if (!already_queued) {
    PendingCloud pc;
    pc.points_lidar = body_points_;
    pc.t_cloud = t_cloud;
    pc.queued_at = std::chrono::steady_clock::now();
    pending_clouds_.push_back(std::move(pc));
    while (pending_clouds_.size() > kMaxPendingClouds) {
      pending_clouds_.pop_front();
      ++stat_drop_no_image_;
      RCLCPP_WARN(this->get_logger(), "彩色[丢弃] 等图像队列溢出，丢弃最早一帧");
    }
  }

  drainPendingClouds();
  drainPendingColorization();
}

void VisualANode::drainPendingClouds() {
  const auto now = std::chrono::steady_clock::now();
  for (auto it = pending_clouds_.begin(); it != pending_clouds_.end();) {
    // 显式配对：按 header.stamp（不是 t_img）与 t_cloud 的差值挑最接近的一帧图像。
    const colorizer::TimedImage* chosen = nullptr;
    double best_dt = std::numeric_limits<double>::max();
    for (const auto& image : image_cache_) {
      if (!image.valid || image.image.empty()) {
        continue;
      }
      const double dt = std::abs(image.arrival_stamp - it->t_cloud);
      if (dt < best_dt) {
        best_dt = dt;
        chosen = &image;
      }
    }
    if (chosen != nullptr && best_dt <= colorizer_options_.sync.max_offset) {
      PendingColorFrame frame;
      frame.points_lidar = it->points_lidar;
      frame.t_cloud = it->t_cloud;
      frame.t_pair = chosen->arrival_stamp;
      frame.t_img = frame.t_pair + img_time_offset_;
      frame.image = chosen->image;
      frame.queued_at = now;
      pending_color_.push_back(std::move(frame));
      while (pending_color_.size() > kMaxPendingColorFrames) {
        pending_color_.pop_front();
        ++stat_drop_pose_;
        RCLCPP_WARN(this->get_logger(), "彩色[丢弃] 待处理队列溢出，丢弃最早一帧");
      }
      it = pending_clouds_.erase(it);
      continue;
    }
    // 图像还没到：等下一帧图像/云/里程计再试；等太久（例如相机断流）才真丢弃。
    const double waited = std::chrono::duration<double>(now - it->queued_at).count();
    if (waited > kPendingCloudTimeoutSec) {
      ++stat_drop_no_image_;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "彩色[丢弃] 等待图像超时: t_cloud=%.4f 最近差=%.4f 阈值=%.4f 已等 %.2fs",
                           it->t_cloud, best_dt, colorizer_options_.sync.max_offset, waited);
      it = pending_clouds_.erase(it);
      continue;
    }
    ++it;
  }
}

void VisualANode::drainPendingColorization() {
  const auto now = std::chrono::steady_clock::now();
  for (auto it = pending_color_.begin(); it != pending_color_.end();) {
    if (colorizeFrame(*it)) {
      it = pending_color_.erase(it);
      continue;
    }
    // 位姿仍未到齐：等一会儿再来（由 onOdom 触发）。超过上限说明确实等不到。
    const double waited = std::chrono::duration<double>(now - it->queued_at).count();
    if (waited > kPendingColorTimeoutSec) {
      ++stat_drop_pose_;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "彩色[丢弃] 等待位姿超时: t_cloud=%.4f t_img=%.4f 已等 %.2fs",
                           it->t_cloud, it->t_img, waited);
      it = pending_color_.erase(it);
      continue;
    }
    ++stat_deferred_;
    ++it;
  }
}

bool VisualANode::colorizeFrame(const PendingColorFrame& frame) {
  const double t_cloud = frame.t_cloud;
  const double t_img = frame.t_img;

  // 两个时刻的位姿都必须由缓存插值得到，任一失败就稍后重试，绝不静默退回最新位姿。
  Eigen::Matrix4d T_wl_cloud = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_wl_img = Eigen::Matrix4d::Identity();
  if (!poseAtStamp(t_cloud, T_wl_cloud) || !poseAtStamp(t_img, T_wl_img)) {
    return false;
  }

  // 统一时间基准：相机位姿取 t_img，点云世界系变换取 t_cloud（由 point_stamps 决定）。
  colorizer::ColorizerPipeline::Options options = colorizer_options_;
  const Eigen::Matrix4d T_wc_img = T_wl_img * T_lc_;  // 图像采集时刻的相机位姿
  options.T_cl = T_wc_img.inverse();                  // 实现语义为 T_cw

  // 只把选中的这一帧图像交给流水线，并把它的时间戳按"配对时刻"给出（流水线只用它判断
  // 是否在同步窗口内；位姿已经由 options.T_cl 固定为 t_img 时刻，不再依赖图像时间）。
  colorizer::TimedImage image;
  image.image = frame.image;
  image.exposure_mid_stamp = frame.t_pair;
  image.arrival_stamp = frame.t_pair;
  image.valid = true;
  std::vector<colorizer::TimedImage> images_for_frame{std::move(image)};
  std::vector<double> point_stamps(frame.points_lidar.size(), t_cloud);

  const colorizer::ColorizerPipeline pipeline(options);
  const std::vector<colorizer::ColoredPoint> colored =
      pipeline.process(frame.points_lidar, point_stamps, images_for_frame, pose_cache_);
  if (colored.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "彩色点云为空（t_cloud=%.4f t_img=%.4f）", t_cloud, t_img);
  }

  std::size_t colored_count = 0;
  for (const auto& p : colored) {
    if (p.quality > 0.0F) {
      ++colored_count;
    }
  }
  ++stat_colored_frames_;
  last_colored_cloud_stamp_ = t_cloud;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "彩色: t_cloud=%.4f t_pair=%.4f t_img=%.4f(offset=%+.3f) "
                       "dt_pair_cloud=%+.4f dt_img_cloud=%+.4f "
                       "%zu/%zu 点上色 帧#%zu 待处理=%zu",
                       t_cloud, frame.t_pair, t_img, img_time_offset_,
                       frame.t_pair - t_cloud, t_img - t_cloud, colored_count,
                       colored.size(), stat_colored_frames_, pending_color_.size());

  // 累积到世界系彩色地图（定时器/析构时统一写 PCD）。
  accumulateColoredMap(colored, t_cloud);
  publishColoredCloud(colored, toRosStamp(t_cloud));
  return true;
}

bool VisualANode::poseAtStamp(double stamp, Eigen::Matrix4d& T_wl) const {
  // 关键：t_img 常常晚于当前已收到的最新 odom（offset 造成的"未来时刻"）。
  // Interpolator 在查询时刻晚于最新采样时会直接返回最后一个采样，那等于把
  // t_img 的位姿退化成 t_cloud 的位姿，offset 就白加了。这里显式拒绝未来查询，
  // 由调用方延后重试。
  // 轨迹源由 color.use_own_odom 决定：默认 /lio/odom；true 时用本节点视觉修正后的位姿。
  const std::vector<colorizer::PoseSample>& cache =
      color_use_own_odom_ ? own_pose_cache_ : pose_cache_;
  double newest = -std::numeric_limits<double>::max();
  for (const auto& sample : cache) {
    newest = std::max(newest, sample.stamp);
  }
  if (stamp > newest) {
    return false;
  }

  // 与彩色流水线 / 累积逻辑保持同一套插值规则和同一个采样间隔上限。
  colorizer::LidarPoseInterpolator interpolator(
      colorizer::LidarPoseInterpolator::Options{kPoseHistoryMaxGapSec});
  for (const auto& sample : cache) {
    interpolator.addSample(sample);
  }
  return interpolator.poseAt(stamp, T_wl);
}

bool VisualANode::poseAtStampCorrected(double stamp, Eigen::Matrix4d& T_wl) const {
  // 与 poseAtStamp 同一套插值规则，但只查修正轨迹缓存 own_pose_cache_（状态回灌模式）。
  double newest = -std::numeric_limits<double>::max();
  for (const auto& sample : own_pose_cache_) {
    newest = std::max(newest, sample.stamp);
  }
  if (own_pose_cache_.empty() || stamp > newest) {
    return false;
  }
  colorizer::LidarPoseInterpolator interpolator(
      colorizer::LidarPoseInterpolator::Options{kPoseHistoryMaxGapSec});
  for (const auto& sample : own_pose_cache_) {
    interpolator.addSample(sample);
  }
  return interpolator.poseAt(stamp, T_wl);
}

void VisualANode::publishColoredCloud(const std::vector<colorizer::ColoredPoint>& points,
                                      const builtin_interfaces::msg::Time& stamp) {
  // 用 pcl::PointXYZRGB 组包，pcl::toROSMsg 会自动产生 x/y/z/rgb 字段。
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.reserve(points.size());
  for (const auto& cp : points) {
    pcl::PointXYZRGB p;
    p.x = cp.xyz.x();
    p.y = cp.xyz.y();
    p.z = cp.xyz.z();
    const auto clamp_u8 = [](float v) {
      return static_cast<std::uint8_t>(std::clamp(v, 0.0F, 255.0F));
    };
    p.r = clamp_u8(cp.rgb.x());
    p.g = clamp_u8(cp.rgb.y());
    p.b = clamp_u8(cp.rgb.z());
    cloud.push_back(p);
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  // 点仍在 LiDAR 系；Super-LIO 把该坐标系命名为 "imu"（world->imu 有 TF），
  // 这样 RViz 里直接选 imu 就能显示，无需额外补 TF。
  msg.header.frame_id = "imu";
  colored_cloud_pub_->publish(msg);
}

void VisualANode::accumulateColoredMap(const std::vector<colorizer::ColoredPoint>& points,
                                       double stamp) {
  if (points.empty()) {
    return;
  }
  if (colored_map_.size() >= max_colored_map_points_) {
    if (!pcd_capacity_warned_) {
      pcd_capacity_warned_ = true;
      RCLCPP_WARN(this->get_logger(), "彩色地图点数达到上限 %zu，后续帧不再累积",
                  max_colored_map_points_);
    }
    return;
  }

  // 与彩色流水线保持一致：lio/body/cloud 已由 Super-LIO 去畸变，逐点时间都取
  // 扫描帧头，因此这里只需在 stamp 时刻取一次 T_wl，把点从 LiDAR 系搬到世界系。
  colorizer::LidarPoseInterpolator interp(
      colorizer::LidarPoseInterpolator::Options{kPoseHistoryMaxGapSec});
  for (const auto& sample : pose_cache_) {
    interp.addSample(sample);
  }
  Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
  if (!interp.poseAt(stamp, T_wl)) {
    // 禁止回退到最新位姿：那会把点云搬到错误时刻的世界系，产生跨帧错位的"重影"。
    // 这里直接丢弃整帧并计数，原因与着色阶段的插值失败区分统计。
    ++stat_drop_accum_;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "彩色[丢弃] 累积位姿插值失败: stamp=%.4f 缓存=%zu",
                         stamp, pose_cache_.size());
    return;
  }

  const Eigen::Matrix3d R = T_wl.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T_wl.block<3, 1>(0, 3);

  // ---- 时间中位数：只统计颜色，几何仍按原逻辑全量 append ----------------
  // 注意：这里不能 return —— 体素只用来算“该位置的多数色”，落盘时回填到
  //   每一个保留下来的几何点上。点数与坐标完全不变，不会把点云变稀。
  if (median_map_.enabled()) {
    for (const auto& cp : points) {
      if (cp.quality <= 0.0F) continue;
      const Eigen::Vector3f pw = (R * cp.xyz.cast<double>() + t).cast<float>();
      median_map_.insert(pw, cp.rgb);
    }
  }

  const std::size_t room = max_colored_map_points_ - colored_map_.size();
  // 注意：这里不能调用 reserve(size + room)，否则每帧都会强制按精确容量重分配
  //   （百万级点的整块拷贝），单帧耗时会被拖到 200 ms 以上。
  //   交给 std::vector 的几何增长即可，均摊 O(1)。

  const auto clamp_u8 = [](float v) {
    return static_cast<std::uint8_t>(std::clamp(v, 0.0F, 255.0F));
  };
  std::size_t added = 0;
  for (const auto& cp : points) {
    if (added >= room) {
      break;
    }
    if (cp.quality <= 0.0F) {
      continue;  // 只保留真正上过色的点
    }
    pcl::PointXYZRGB q;
    const Eigen::Vector3f pw = (R * cp.xyz.cast<double>() + t).cast<float>();
    q.x = pw.x();
    q.y = pw.y();
    q.z = pw.z();
    q.r = clamp_u8(cp.rgb.x());
    q.g = clamp_u8(cp.rgb.y());
    q.b = clamp_u8(cp.rgb.z());
    q.a = 255;
    colored_map_.push_back(q);
    ++added;
  }
}

void VisualANode::saveColoredMap() {
  if (colored_map_.empty() || colored_map_.size() == pcd_saved_points_) {
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories(pcd_save_dir_, ec);
  if (ec) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "无法创建 PCD 输出目录 %s: %s", pcd_save_dir_.c_str(),
                         ec.message().c_str());
    return;
  }

  const std::string path =
      (std::filesystem::path(pcd_save_dir_) / "super_livo_visual_a_colored.pcd").string();

  // 时间中位数：只回填颜色，几何/点数完全不动（避免体素化把点云变稀）。
  if (median_map_.enabled()) {
    const auto c8 = [](double v) {
      return static_cast<std::uint8_t>(std::clamp(v, 0.0, 255.0));
    };
    for (auto& p : colored_map_.points) {
      Eigen::Vector3d rgb;
      if (median_map_.colorAt(Eigen::Vector3f(p.x, p.y, p.z), rgb)) {
        p.r = c8(rgb.x());
        p.g = c8(rgb.y());
        p.b = c8(rgb.z());
      }
    }
  }

  colored_map_.width = static_cast<std::uint32_t>(colored_map_.size());
  colored_map_.height = 1;
  colored_map_.is_dense = false;
  if (pcl::io::savePCDFileBinary(path, colored_map_) == 0) {
    pcd_saved_points_ = colored_map_.size();
    RCLCPP_INFO(this->get_logger(), "彩色地图已保存: %s（%zu 点，世界系%s）", path.c_str(),
                colored_map_.size(),
                median_map_.enabled() ? "，时间中位数回填色" : "");
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "保存 PCD 失败: %s", path.c_str());
  }
}

void VisualANode::publishOdomAndPath(const Eigen::Matrix4d& T_wl,
                                     const builtin_interfaces::msg::Time& stamp) {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "world";
  odom.child_frame_id = "lidar";
  odom.pose.pose.position.x = T_wl(0, 3);
  odom.pose.pose.position.y = T_wl(1, 3);
  odom.pose.pose.position.z = T_wl(2, 3);
  const Eigen::Quaterniond q(T_wl.block<3, 3>(0, 0));
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom_pub_->publish(odom);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.header;
  pose.pose = odom.pose.pose;
  path_msg_.header.stamp = stamp;
  path_msg_.poses.push_back(pose);
  path_pub_->publish(path_msg_);
}

cv::Mat VisualANode::rectifyBgr(const cv::Mat& bgr) const {
  if (bgr.empty()) {
    return {};
  }
  const cv::Size size(bgr.cols, bgr.rows);
  if (!rectify_maps_ready_) {
    // K/D 与图像同分辨率（bag 里已经是 640x512，内参也按 0.5 缩放好了）。
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = intrinsics_.fx;
    K.at<double>(1, 1) = intrinsics_.fy;
    K.at<double>(0, 2) = intrinsics_.cx;
    K.at<double>(1, 2) = intrinsics_.cy;
    cv::Mat D = cv::Mat::zeros(1, static_cast<int>(distortion_.size()), CV_64F);
    for (std::size_t i = 0; i < distortion_.size(); ++i) {
      D.at<double>(0, static_cast<int>(i)) = distortion_[i];
    }
    // 目标相机矩阵仍用 K：去畸变后保持同一内参，便于视觉模块直接使用。
    cv::initUndistortRectifyMap(K, D, cv::Mat(), K, size, CV_32FC1,
                                rectify_map1_, rectify_map2_);
    rectify_maps_ready_ = true;
  }
  cv::Mat rectified;
  cv::remap(bgr, rectified, rectify_map1_, rectify_map2_, cv::INTER_LINEAR);
  return rectified;
}

Eigen::Matrix4d VisualANode::makeTlcFromRclPcl() {
  // 注意坐标方向：avia_lvi.yaml 的 Rcl/Pcl 与 FAST-LIVO2 用法一致，表示
  //   p_cam = Rcl · p_lidar + Pcl        （即 T_cam_lidar，LiDAR -> Camera）
  // 而本节点内部统一使用 T_lc（Camera -> LiDAR），所以这里要求逆。
  //   （若把 Rcl 直接当 T_lc，点会投到图像外，彩色点云会 0 点上色。）
  const auto rcl = this->declare_parameter<std::vector<double>>(
      "Rcl", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
  const auto pcl = this->declare_parameter<std::vector<double>>(
      "Pcl", std::vector<double>{0, 0, 0});
  Eigen::Matrix4d T_cl = Eigen::Matrix4d::Identity();
  if (rcl.size() >= 9) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        T_cl(r, c) = rcl[static_cast<std::size_t>(r * 3 + c)];
      }
    }
  }
  if (pcl.size() >= 3) {
    T_cl(0, 3) = pcl[0];
    T_cl(1, 3) = pcl[1];
    T_cl(2, 3) = pcl[2];
  }
  return T_cl.inverse();  // Camera -> LiDAR
}

Eigen::Matrix4d VisualANode::applyLeftPerturbation(const Eigen::Matrix4d& T,
                                                   const Eigen::VectorXd& dx) const {
  // dx = [φ(3); ρ(3)]，左扰动：R' = Exp(φ)·R，p' = Exp(φ)·p + ρ。
  if (dx.size() < 6) {
    return T;
  }
  const Eigen::Vector3d phi = dx.head<3>();
  const Eigen::Vector3d rho = dx.tail<3>();
  const Eigen::Matrix3d dR = so3Exp(phi);

  Eigen::Matrix4d T2 = T;
  T2.block<3, 3>(0, 0) = dR * T.block<3, 3>(0, 0);
  T2.block<3, 1>(0, 3) = dR * T.block<3, 1>(0, 3) + rho;
  return T2;
}

void VisualANode::decayPoseOffset(double dt, double severity,
                                  const Eigen::Vector3d& dir_w) {
  // ---------------------------------------------------------------------------
  // 为什么需要它：门控在 LiDAR 健康时给出 visual_weight=0，VisualUpdate::apply
  // 会直接返回，于是 pose_offset_ 在健康期**完全冻结**。而偏移本身是
  //   pose_offset_ ← Exp(dx) · pose_offset_
  // 的累乘，只增不减：退化期每帧 1~10 mm 的视觉修正会被无差别积分下去，
  // 退化结束后仍把这段"视觉说了算的偏差"一直带着（eee_03 实测末段 243 mm，
  // 其中沿退化法向 164 mm；同期纯 lio/odom 的 ATE 只有 133 mm）。
  //
  // 这里把它改成带泄漏的累积器，衰减率只用 Super-LIO 已验证的退化签名来定：
  //   * 旋转：trans_only 检测不给旋转方向，各向同性按 α_perp 衰减；
  //   * 平移垂直 u：LiDAR 一直看得住，按 α_perp 衰减；
  //   * 平移沿 u：α_perp·(1-sev)，完全退化（sev=1）时不衰减，允许视觉累积。
  // 于是"健康时回落到 LIO、只在看不住的方向长期保留视觉约束"同时成立。
  // ---------------------------------------------------------------------------
  if (!offset_decay_en_ || offset_decay_rate_ <= 0.0 || dt <= 0.0) {
    return;
  }
  const double sev = std::clamp(severity, 0.0, 1.0);
  const double alpha_perp = offset_decay_rate_;
  double alpha_par = alpha_perp;
  Eigen::Vector3d u = Eigen::Vector3d::Zero();
  if (dir_w.norm() > 1e-6) {
    u = dir_w.normalized();
    alpha_par = alpha_perp * (1.0 - sev);
  }

  const Eigen::Matrix3d R_off = pose_offset_.block<3, 3>(0, 0);
  const Eigen::AngleAxisd aa(R_off);
  const double ang = aa.angle();
  const Eigen::Vector3d phi =
      ang > 1e-9 ? Eigen::Vector3d(ang * aa.axis()) : Eigen::Vector3d::Zero();
  Eigen::Vector3d rho = pose_offset_.block<3, 1>(0, 3);

  const double f_perp = std::exp(-alpha_perp * dt);
  // 退化方向在上一步已经被归一化过（非单位则视为无方向）：
  // 平移：沿 u 用 f_par，垂直 u 用 f_perp。
  if (u.isZero(1e-9)) {
    rho *= f_perp;
  } else {
    const double r_par = rho.dot(u);
    rho = std::exp(-alpha_par * dt) * r_par * u + f_perp * (rho - r_par * u);
  }

  pose_offset_.setIdentity();
  pose_offset_.block<3, 3>(0, 0) = so3Exp(f_perp * phi);
  pose_offset_.block<3, 1>(0, 3) = rho;
}

void VisualANode::onDegeneracy(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(degen_mutex_);
  // 布局：[cond6, axes, active, cond_t, evt_min, ux, uy, uz]，解析见 common。
  if (!degen_sig_.fromArray(msg->data)) {
    return;
  }
  degen_recv_time_ = this->now().seconds();
  if (!degen_seen_) {
    degen_seen_ = true;
    RCLCPP_INFO(this->get_logger(),
                "收到退化签名: cond6=%.2f cond_t=%.2f axes=%d active=%d u=(%+.2f %+.2f %+.2f)",
                degen_sig_.cond6, degen_sig_.cond_t, degen_sig_.axes,
                degen_sig_.active ? 1 : 0, degen_sig_.dir.x(), degen_sig_.dir.y(),
                degen_sig_.dir.z());
  }
}

bool VisualANode::degeneracySignature(double& severity,
                                      Eigen::Vector3d& dir_w) const {
  severity = 0.0;
  dir_w.setZero();
  std::lock_guard<std::mutex> lock(degen_mutex_);
  if (degen_recv_time_ < 0.0 ||
      this->now().seconds() - degen_recv_time_ > degen_timeout_) {
    return false;  // 没有签名可用 -> 调用方按"不退化"处理
  }
  // cond=0 说明 Super-LIO 侧根本没跑检测（detect_en 关闭），或非 trans_only 模式
  // （此时不产出平移块判据/方向）。两种情况都不能当"健康"用，否则退化门控静默失效。
  if (!degen_sig_.valid()) {
    return false;
  }
  dir_w = degen_sig_.dir;
  severity = degen_sig_.severity(gate_cond_lo_, gate_cond_hi_);
  return true;
}

}  // namespace superlivo_a

// ----------------------------------------------------------------------------
// 入口：标准 ROS2 节点 main（spin 直至 SIGINT）。
// 若后续需要与 Super-LIO 组合到同一进程，可改为 rclcpp_components 可组合节点。
// ----------------------------------------------------------------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<superlivo_a::VisualANode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
