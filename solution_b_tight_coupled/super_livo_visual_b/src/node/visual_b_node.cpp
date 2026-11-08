#include "super_livo_visual_b/node/visual_b_node.hpp"

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
#include <Eigen/LU>

namespace superlivo_b {

namespace {
/// 位姿历史缓冲允许的最大相邻采样间隔（秒）。
/// 与 color.sync_max_offset（图像-点云同步窗口）是两件事，不能混用。
/// /lio/odom 约 10 Hz，这里留 5 倍余量，仅用于拒绝真正的空洞。
constexpr double kPoseHistoryMaxGapSec = 0.5;

/// 待着色帧最长等待时间（秒）：t_img 比 t_cloud 晚 img_time_offset，等 1 s 还拿不到
/// 位姿说明确实没有后续 odom（例如 bag 结束）。
constexpr double kPendingColorTimeoutSec = 1.0;

/// 待着色队列上限，防止 odom 长时间缺失时内存膨胀。
constexpr std::size_t kMaxPendingColorFrames = 8;

/// 等图像队列：上限与超时（秒）。
constexpr std::size_t kMaxPendingClouds = 6;
constexpr double kPendingCloudTimeoutSec = 1.0;

/// 标准重力幅值（m/s²），用于把加计归一化到 m/s²（对齐官方 G_m_s2）。
constexpr double kGravityNorm = 9.81;

/// double 秒 -> ROS 时间戳。
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

/// so(3) 指数映射。
Eigen::Matrix3d so3Exp(const Eigen::Vector3d& w) {
  const double theta = w.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity() + skew(w);
  }
  return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
}

/// so(3) 对数映射（返回旋转向量）。
Eigen::Vector3d so3Log(const Eigen::Matrix3d& R) {
  const Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}
}  // namespace

VisualBNode::VisualBNode(const rclcpp::NodeOptions& options)
    : Node("super_livo_visual_b", options) {
  // ---- 先声明 topic 参数（订阅/发布器创建前需要） -------------------------
  const std::string image_topic = this->declare_parameter<std::string>("image_topic", "/left_camera/image");
  const std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "lio/odom");
  const std::string cloud_topic = this->declare_parameter<std::string>("cloud_topic", "lio/cloud_world");
  const std::string body_cloud_topic =
      this->declare_parameter<std::string>("body_cloud_topic", "lio/cloud_body_dense");
  const std::string imu_topic = this->declare_parameter<std::string>("imu_topic", "/livox/imu");

  // ---- 创建订阅器 ---------------------------------------------------------
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&VisualBNode::onImage, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&VisualBNode::onOdom, this, std::placeholders::_1));
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&VisualBNode::onCloud, this, std::placeholders::_1));
  body_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      body_cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&VisualBNode::onBodyCloud, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&VisualBNode::onImu, this, std::placeholders::_1));

  // ---- 创建发布器 ---------------------------------------------------------
  colored_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "super_livo_visual_b/colored_cloud", rclcpp::SensorDataQoS());
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "super_livo_visual_b/odom", rclcpp::SensorDataQoS());
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "super_livo_visual_b/path", rclcpp::SensorDataQoS());

  loadParameters();

  // 周期性把累积的彩色地图落盘（默认 10 s 一次，覆盖写同一个文件）。
  // 注意：单线程 executor 下这里会阻塞所有订阅回调（实测 NTU 4M 点落盘一次 ~25 s，
  // 期间图像/IMU 全丢，恢复后按陈旧 acc 外推 25 s，速度直接冲到几十 m/s）。
  // 需要对 odom 精度做评测时把 pcd.save_interval_sec 设成 <=0，只在退出时存一次。
  if (pcd_save_interval_sec_ > 0.0) {
    pcd_save_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(pcd_save_interval_sec_)),
        std::bind(&VisualBNode::saveColoredMap, this));
  }

  path_msg_.header.frame_id = "world";
  RCLCPP_INFO(this->get_logger(), "VisualBNode 初始化完成");
}

VisualBNode::~VisualBNode() {
  // 收尾：bag 结束时队列里可能还有等图像/等位姿的帧，尽量补完再落盘。
  drainPendingClouds();
  drainPendingColorization();
  // 退出（Ctrl+C / SIGTERM 触发 rclcpp::spin 返回）时再存一次，保证短时长运行也有结果。
  saveColoredMap();
}

void VisualBNode::loadParameters() {
  // ---- 相机内参与畸变（同方案 A，来自 camera_pinhole.yaml） --------------
  scale_ = this->declare_parameter<double>("scale", 0.5);
  intrinsics_.width = this->declare_parameter<int>("cam_width", 640);
  intrinsics_.height = this->declare_parameter<int>("cam_height", 512);
  intrinsics_.fx = this->declare_parameter<double>("cam_fx", 646.78);
  intrinsics_.fy = this->declare_parameter<double>("cam_fy", 646.66);
  intrinsics_.cx = this->declare_parameter<double>("cam_cx", 313.46);
  intrinsics_.cy = this->declare_parameter<double>("cam_cy", 261.40);
  distortion_ = this->declare_parameter<std::vector<double>>(
      "cam_d", std::vector<double>{-0.076160, 0.123001, -0.00113, 0.000251});

  // ---- 外参：Camera->LiDAR（Rcl/Pcl）与 LiDAR->IMU（可选） ----------------
  T_lc_ = makeTlcFromRclPcl();
  // T_li（LiDAR->IMU）默认单位阵：Super-LIO 把 IMU 量测旋转到 LiDAR 系，
  // 其状态本身就以 LiDAR 为 body，因此这里取单位阵即可（T_wi ≈ T_wl）。
  T_li_ = Eigen::Matrix4d::Identity();

  // LiDAR 先验强度（D2）：先验越小越"信任" lio/odom，视觉修正越弱。
  prior_pos_sigma_ = this->declare_parameter<double>("lv.prior_pos_sigma", 0.05);
  prior_rot_sigma_ = this->declare_parameter<double>("lv.prior_rot_sigma", 0.02);

  // 默认值与官方 avia.yaml 一致（0.1 s），本数据集已通过 offset 扫描验证最优。
  img_time_offset_ = this->declare_parameter<double>("img_time_offset", 0.1);

  // ---- 前端 PatchTracker --------------------------------------------------
  patch_options_.patch_size = this->declare_parameter<int>("patch.patch_size", 5);
  patch_options_.pyramid_levels = this->declare_parameter<int>("patch.pyramid_levels", 4);
  patch_options_.max_features = this->declare_parameter<int>("patch.max_features", 200);
  patch_options_.lssd_threshold = this->declare_parameter<double>("patch.lssd_threshold", 0.5);
  patch_options_.min_distance = this->declare_parameter<double>("patch.min_distance", 8.0);
  patch_options_.quality_level = this->declare_parameter<double>("patch.quality_level", 0.01);
  patch_options_.use_lssd = this->declare_parameter<bool>("patch.use_lssd", true);
  patch_options_.search_radius = this->declare_parameter<int>("patch.search_radius", 4);
  frontend_ = std::make_unique<PatchTracker>(patch_options_);

  // ---- 特征生命周期 FeatureLifecycle --------------------------------------
  lifecycle_options_.mature_observations = this->declare_parameter<int>("lifecycle.mature_observations", 4);
  lifecycle_options_.max_failures = this->declare_parameter<int>("lifecycle.max_failures", 2);
  lifecycle_options_.max_track_length = this->declare_parameter<int>("lifecycle.max_track_length", 30);
  lifecycle_ = std::make_unique<FeatureLifecycle>(lifecycle_options_);

  // ---- 关键帧管理 KeyframeManager -----------------------------------------
  keyframe_options_.max_keyframes = this->declare_parameter<int>("kf.max_keyframes", 20);
  keyframe_options_.min_translation = this->declare_parameter<double>("kf.min_translation", 0.15);
  keyframe_options_.min_rotation_deg = this->declare_parameter<double>("kf.min_rotation_deg", 5.0);
  keyframe_options_.min_tracked_features = this->declare_parameter<int>("kf.min_tracked_features", 40);
  keyframe_manager_ = std::make_unique<KeyframeManager>(keyframe_options_);

  // ---- 2D-3D 约束 LidarVisualConstraint -----------------------------------
  lidar_visual_options_.fx = intrinsics_.fx;
  lidar_visual_options_.fy = intrinsics_.fy;
  lidar_visual_options_.cx = intrinsics_.cx;
  lidar_visual_options_.cy = intrinsics_.cy;
  lidar_visual_options_.image_width = intrinsics_.width;
  lidar_visual_options_.image_height = intrinsics_.height;
  lidar_visual_options_.pixel_radius = this->declare_parameter<int>("lv.pixel_radius", 3);
  lidar_visual_options_.huber_delta = this->declare_parameter<double>("lv.huber_delta", 1.0);
  lidar_visual_options_.min_constraints = this->declare_parameter<int>("lv.min_constraints", 8);
  lidar_visual_options_.max_pixel_error = this->declare_parameter<double>("lv.max_pixel_error", 8.0);
  // 测量噪声模型：没有它等价于 σ=1 px 且地图点无误差，一帧即可压塌协方差。
  lidar_visual_options_.sigma_pix = this->declare_parameter<double>("lv.sigma_pix", 1.5);
  lidar_visual_options_.sigma_anchor = this->declare_parameter<double>("lv.sigma_anchor", 0.05);
  lidar_visual_ = std::make_unique<LidarVisualConstraint>(lidar_visual_options_);

  // ---- MSCKF 多帧约束 MsckfConstraint -------------------------------------
  msckf_options_.fx = intrinsics_.fx;
  msckf_options_.fy = intrinsics_.fy;
  msckf_options_.cx = intrinsics_.cx;
  msckf_options_.cy = intrinsics_.cy;
  msckf_options_.min_observations = this->declare_parameter<int>("msckf.min_observations", 4);
  msckf_options_.max_track_length = this->declare_parameter<int>("msckf.max_track_length", 20);
  msckf_options_.min_parallax_deg = this->declare_parameter<double>("msckf.min_parallax_deg", 1.0);
  msckf_options_.huber_delta = this->declare_parameter<double>("msckf.huber_delta", 1.0);
  msckf_options_.use_nullspace_projection = this->declare_parameter<bool>("msckf.use_nullspace_projection", true);
  msckf_options_.sigma_pix = this->declare_parameter<double>("msckf.sigma_pix", 1.5);
  msckf_options_.sigma_track_pose = this->declare_parameter<double>("msckf.sigma_track_pose", 0.05);
  msckf_ = std::make_unique<MsckfConstraint>(msckf_options_);

  // ---- 光度约束 PhotometricConstraint（可选，默认关） ----------------------
  photo_options_.fx = intrinsics_.fx;
  photo_options_.fy = intrinsics_.fy;
  photo_options_.cx = intrinsics_.cx;
  photo_options_.cy = intrinsics_.cy;
  photo_options_.enable = this->declare_parameter<bool>("photo.enable", false);
  photo_options_.patch_size = this->declare_parameter<int>("photo.patch_size", 5);
  photo_options_.pyramid_levels = this->declare_parameter<int>("photo.pyramid_levels", 3);
  photo_options_.huber_delta = this->declare_parameter<double>("photo.huber_delta", 1.0);
  photo_options_.max_intensity_error = this->declare_parameter<double>("photo.max_intensity_error", 40.0);
  photometric_ = std::make_unique<PhotometricConstraint>(photo_options_);

  // ---- 统一估计器 UnifiedEstimator ----------------------------------------
  estimator_options_.enable_visual = this->declare_parameter<bool>("est.enable_visual", true);
  estimator_options_.visual_weight = this->declare_parameter<double>("est.visual_weight", 1.0);
  estimator_options_.estimate_extrinsics = this->declare_parameter<bool>("est.estimate_extrinsics", false);
  estimator_options_.gyro_noise = this->declare_parameter<double>("est.gyro_noise", 1.0e-3);
  estimator_options_.acc_noise = this->declare_parameter<double>("est.acc_noise", 1.0e-2);
  estimator_options_.bias_gyro_noise = this->declare_parameter<double>("est.bias_gyro_noise", 1.0e-5);
  estimator_options_.bias_acc_noise = this->declare_parameter<double>("est.bias_acc_noise", 1.0e-4);
  estimator_options_.initial_covariance = this->declare_parameter<double>("est.initial_covariance", 1.0);
  // 视觉注入强度上限与信息耦合方式（详见 UnifiedEstimator::Options 注释）。
  estimator_options_.visual_info_ratio = this->declare_parameter<double>("est.visual_info_ratio", 1.0);
  estimator_options_.visual_rot_sigma_floor =
      this->declare_parameter<double>("est.visual_rot_sigma_floor", 0.005);
  estimator_options_.visual_pos_sigma_floor =
      this->declare_parameter<double>("est.visual_pos_sigma_floor", 0.02);
  estimator_options_.visual_pose_only = this->declare_parameter<bool>("est.visual_pose_only", true);
  estimator_ = std::make_unique<UnifiedEstimator>(estimator_options_);

  // ---- 延迟地图提交 MapCommitScheduler ------------------------------------
  scheduler_options_.enable = this->declare_parameter<bool>("scheduler.enable", true);
  scheduler_options_.max_pending = this->declare_parameter<int>("scheduler.max_pending", 3);
  scheduler_ = std::make_unique<MapCommitScheduler>(scheduler_options_);

  // ---- 彩色点云 ColorizerPipeline（复用方案 A 的包） ----------------------
  colorizer_options_.intrinsics = intrinsics_;
  // 着色轨迹源：默认 false=/lio/odom；true=统一估计器修正后的位姿（own_pose_cache_）。
  color_use_own_odom_ = this->declare_parameter<bool>("color.use_own_odom", false);
  // T_cl 在实现中的语义是 T_cw（作用于世界系点），逐帧变化，见 runColorizer。
  colorizer_options_.T_cl = Eigen::Matrix4d::Identity();
  colorizer_options_.sync.max_offset = this->declare_parameter<double>("color.sync_max_offset", 0.15);
  colorizer_options_.sync.use_exposure_mid = this->declare_parameter<bool>("color.use_exposure_mid", true);
  colorizer_options_.occlusion.depth_tolerance = this->declare_parameter<double>("color.depth_tolerance", 0.1);
  colorizer_options_.occlusion.check_depth_edges = this->declare_parameter<bool>("color.check_depth_edges", true);
  colorizer_options_.occlusion.edge_threshold = this->declare_parameter<double>("color.edge_threshold", 0.5);
  colorizer_options_.rgb.bilinear = this->declare_parameter<bool>("color.bilinear", true);
  colorizer_options_.rgb.keep_best_quality = this->declare_parameter<bool>("color.keep_best_quality", true);
  colorizer_options_.rgb.min_quality = this->declare_parameter<float>("color.min_quality", 0.2F);

  // ---- 彩色地图落盘（把跑完的彩色点云输出为 PCD） ------------------------
  pcd_save_dir_ = this->declare_parameter<std::string>("pcd.save_dir", "/home/ywj/slam_data/pcd");
  pcd_save_interval_sec_ = this->declare_parameter<double>("pcd.save_interval_sec", 30.0);
  max_colored_map_points_ = static_cast<std::size_t>(
      this->declare_parameter<int>("pcd.max_points", 20000000));
  params_loaded_ = true;
}

void VisualBNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  if (!params_loaded_) {
    return;
  }
  // 图像 header 时刻：与 dense 点云 header 同一时钟基准（实测差 < 7 ms），只用于配对。
  // 图像真实采集时刻 header + img_time_offset_ 只用于取相机位姿，见 runColorizer。
  const double stamp = rclcpp::Time(msg->header.stamp).seconds();

  // toCvCopy：得到拥有数据所有权的 Mat，才能安全放进 image_cache_。
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

void VisualBNode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  const auto& pose = msg->pose.pose;
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1e-9) {
    return;
  }
  q.normalize();

  Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
  T_wl.block<3, 3>(0, 0) = q.toRotationMatrix();
  T_wl(0, 3) = pose.position.x;
  T_wl(1, 3) = pose.position.y;
  T_wl(2, 3) = pose.position.z;
  T_wl_ = T_wl;

  colorizer::PoseSample sample;
  sample.stamp = rclcpp::Time(msg->header.stamp).seconds();
  sample.T_wl = T_wl;
  pose_cache_.push_back(sample);
  if (pose_cache_.size() > max_pose_samples_) {
    pose_cache_.erase(pose_cache_.begin(),
                      pose_cache_.begin() + (pose_cache_.size() - max_pose_samples_));
  }

  // 用首帧里程计初始化统一状态（否则滤波器从单位阵起步，长期追不上真实轨迹）。
  if (!estimator_initialized_) {
    State init;
    init.R = T_wl.block<3, 3>(0, 0);
    init.p = T_wl.block<3, 1>(0, 3);
    init.T_ic = T_li_ * T_lc_;       // Camera -> IMU
    init.T_il = T_li_.inverse();     // IMU -> LiDAR

    // 加计单位归一化（对齐官方 IMU_Processing::IMU_init）：
    //   静止段 mean_acc 主要由重力构成，|mean_acc|≈1g ⇒ scale=9.81/|mean_acc| 把 acc 变 m/s²；
    //   世界系重力 = -R * mean_acc（加计静止时测的是比力 -R^T g）。
    //   注意必须左乘 init.R：NTU 的 Super-LIO 初始姿态 roll≈180°，不旋会把 g 的 z 符号搞反，
    //   传播里重力补偿变成 2g，轨迹立刻自由落体（实测 180 s 漂到 200 km）。
    if (!imu_buffer_.empty()) {
      Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
      Eigen::Vector3d mean_gyro = Eigen::Vector3d::Zero();
      for (const auto& s : imu_buffer_) {
        mean_acc += s.acc;
        mean_gyro += s.gyro;
      }
      mean_acc /= static_cast<double>(imu_buffer_.size());
      mean_gyro /= static_cast<double>(imu_buffer_.size());
      const double norm = mean_acc.norm();
      if (norm > 1e-3) {
        imu_acc_scale_ = kGravityNorm / norm;
        init.g = init.R * (-mean_acc / norm * kGravityNorm);
      }
      // 陀螺零偏必须用静止段均值初始化：NTU 的 VN100 零偏约 (0.40,0.20,0.11) °/s，
      // 从 0 起估计会让姿态在 60 s 内漂 20~30°，重力泄漏成水平加速度后轨迹发散（实测 3 km）。
      init.bg = mean_gyro;
      RCLCPP_INFO(this->get_logger(),
                  "IMU 加计归一化: |mean_acc|=%.4f ⇒ acc_scale=%.4f, gravity=(%.2f,%.2f,%.2f), "
                  "gyro_bias=(%.4f,%.4f,%.4f) rad/s",
                  norm, imu_acc_scale_, init.g.x(), init.g.y(), init.g.z(),
                  init.bg.x(), init.bg.y(), init.bg.z());
    }

    estimator_->resetState(init);
    estimator_initialized_ = true;
    imu_initialized_ = !imu_buffer_.empty();
    if (imu_initialized_) {
      const auto& last = imu_buffer_.back();
      last_imu_stamp_ = last.stamp;
      last_imu_acc_ = last.acc;
      last_imu_gyro_ = last.gyro;
    } else {
      last_imu_stamp_ = sample.stamp;
    }
    RCLCPP_INFO(this->get_logger(), "统一状态已用首帧 lio/odom 初始化");
  }

  // 新采样到齐后重试待着色帧（t_img = t_cloud + img_time_offset，通常要等未来位姿）。
  if (!pending_color_.empty()) {
    drainPendingColorization();
  }
}

void VisualBNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  // lio/cloud_world：世界系点，直接作为 2D-3D 约束的地图点来源。
  pcl::PointCloud<pcl::PointXYZI> cloud;
  try {
    pcl::fromROSMsg(*msg, cloud);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "点云解析失败: %s", e.what());
    return;
  }
  for (std::size_t i = 0; i < cloud.size();
       i += static_cast<std::size_t>(std::max(1, map_append_stride_))) {
    const auto& p = cloud.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    map_points_.emplace_back(p.x, p.y, p.z);
  }
  if (map_points_.size() > max_map_points_) {
    map_points_.erase(map_points_.begin(),
                      map_points_.begin() + (map_points_.size() - max_map_points_));
  }
  last_cloud_ = msg;
}

void VisualBNode::onBodyCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
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

  // 新扫描到达即入队等图像/位姿（大图回调比云慢约 1 帧，不能等图像回调才入队）。
  runColorizer(*msg);
}

void VisualBNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  ImuSample sample;
  sample.stamp = rclcpp::Time(msg->header.stamp).seconds();
  sample.acc = Eigen::Vector3d(msg->linear_acceleration.x,
                               msg->linear_acceleration.y,
                               msg->linear_acceleration.z);
  sample.gyro = Eigen::Vector3d(msg->angular_velocity.x,
                                msg->angular_velocity.y,
                                msg->angular_velocity.z);
  imu_buffer_.push_back(sample);
  if (imu_buffer_.size() > max_imu_samples_) {
    imu_buffer_.erase(imu_buffer_.begin(),
                      imu_buffer_.begin() + (imu_buffer_.size() - max_imu_samples_));
  }
}

void VisualBNode::processVisualFrame(const cv::Mat& gray, double stamp) {
  // ==========================================================================
  // 主流程（对应 gpt §3.4）：
  //   前端 -> 生命周期 -> 关键帧/轨迹 -> LiDAR 2D-3D + MSCKF -> 统一估计器
  // ==========================================================================
  const auto t_start = std::chrono::steady_clock::now();

  // 1) Patch 前端跟踪
  const std::vector<PatchObservation> obs = frontend_->process(gray, stamp);

  // 2) 生命周期：记录观测，推进成熟；上一帧有、这一帧没跟住的记为 miss
  for (const auto& o : obs) {
    lifecycle_->onObserved(o.feature_id, stamp);
  }
  {
    std::vector<int> current_ids;
    current_ids.reserve(obs.size());
    for (const auto& o : obs) {
      current_ids.push_back(o.feature_id);
    }
    for (const int id : prev_feature_ids_) {
      if (std::find(current_ids.begin(), current_ids.end(), id) == current_ids.end()) {
        lifecycle_->onMissed(id, stamp);
      }
    }
    prev_feature_ids_ = std::move(current_ids);
  }
  lifecycle_->prune();

  // 3) 组装特征轨迹（patch 观测 + 相机位姿）
  const std::vector<FeatureTrack> tracks = assembleTracks(obs, stamp);
  const auto t_front = std::chrono::steady_clock::now();

  // 4) 关键帧判定与轨迹更新
  const Eigen::Matrix4d T_wc = T_wl_ * T_lc_;
  if (keyframe_manager_->shouldCreateKeyframe(T_wc, static_cast<int>(obs.size()), stamp)) {
    VisualKeyframe kf;
    kf.id = next_keyframe_id_++;
    kf.stamp = stamp;
    kf.T_wc = T_wc;
    kf.feature_ids.reserve(obs.size());
    for (const auto& o : obs) {
      kf.feature_ids.push_back(o.feature_id);
    }
    keyframe_manager_->addKeyframe(kf);
  }
  keyframe_manager_->updateTracks(tracks);

  // 5) LiDAR 地图辅助 2D-3D 约束
  lidar_visual_->setMapPoints(map_points_);
  const auto lv_residuals = lidar_visual_->build(tracks, T_wc);
  Eigen::MatrixXd H_lv;
  Eigen::VectorXd b_lv;
  double cost_lv = 0.0;
  lidar_visual_->accumulate(lv_residuals, H_lv, b_lv, cost_lv);

  // 6) MSCKF 多帧轨迹约束
  const auto constraint_tracks = keyframe_manager_->constraintTracks(
      msckf_options_.min_observations, msckf_options_.min_parallax_deg);
  const MultiFrameNormalEquations msckf_eq = msckf_->build(constraint_tracks);

  // 7) 合并视觉法方程（固定 6x6）
  Eigen::MatrixXd H_vis = Eigen::MatrixXd::Zero(6, 6);
  Eigen::VectorXd b_vis = Eigen::VectorXd::Zero(6);
  double cost_vis = 0.0;
  int used_features = 0;

  if (H_lv.rows() == 6 && H_lv.cols() == 6 && b_lv.size() == 6) {
    H_vis += H_lv;
    b_vis += b_lv;
    cost_vis += cost_lv;
  }
  if (msckf_eq.valid && msckf_eq.H.rows() == 6 && msckf_eq.b.size() == 6) {
    H_vis += msckf_eq.H;
    b_vis += msckf_eq.b;
    cost_vis += msckf_eq.cost;
  }
  for (const auto& r : lv_residuals) {
    if (r.valid) {
      ++used_features;
    }
  }
  used_features += msckf_eq.used_features;

  // 7.1（可选）Patch 光度残差：只有三角化成功的轨迹才有三维锚点。
  if (photo_options_.enable) {
    std::vector<FeatureTrack> photo_tracks;
    photo_tracks.reserve(constraint_tracks.size());
    for (const auto& t : constraint_tracks) {
      FeatureTrack copy = t;
      Eigen::Vector3d xyz;
      if (msckf_->triangulate(copy, xyz)) {
        copy.xyz_world = xyz;
        copy.triangulated = true;
        photo_tracks.push_back(copy);
      }
    }
    for (const auto& r : photometric_->build(gray, photo_tracks)) {
      if (!r.valid) {
        continue;
      }
      H_vis += r.weight * r.jacobian.transpose() * r.jacobian;
      b_vis += r.weight * r.jacobian.transpose() * r.residual;
      cost_vis += r.weight * r.residual * r.residual;
    }
  }

  // 7.2 视觉质量（0~1）：约束越多、残差越小，质量越高。
  double mean_pixel_error = 0.0;
  if (used_features > 0) {
    mean_pixel_error = std::sqrt(std::max(0.0, 2.0 * cost_vis / used_features));
  }
  const double quality = std::clamp(
      static_cast<double>(used_features) / (static_cast<double>(used_features) + 20.0) *
          (1.0 / (1.0 + mean_pixel_error)),
      0.0, 1.0);
  const auto t_vis = std::chrono::steady_clock::now();

  // 8) IMU 传播：把上次传播到本帧之间的 IMU 样本逐条积分
  //    acc 乘 imu_acc_scale_ 归一化到 m/s²（见初始化处说明）。
  if (imu_initialized_) {
    double t_prev = last_imu_stamp_;
    Eigen::Vector3d acc_prev = last_imu_acc_;
    Eigen::Vector3d gyro_prev = last_imu_gyro_;
    for (const auto& s : imu_buffer_) {
      if (s.stamp <= t_prev) {
        continue;
      }
      if (s.stamp > stamp) {
        break;
      }
      estimator_->propagate(acc_prev * imu_acc_scale_, gyro_prev, s.stamp - t_prev);
      t_prev = s.stamp;
      acc_prev = s.acc;
      gyro_prev = s.gyro;
    }
    if (stamp > t_prev) {
      // 长间隔（回调被阻塞/IMU 丢包）不能用陈旧 acc 外推：dt 大 1 个量级，
      // 速度误差就大 1 个量级且再也拉不回来。超过 0.2 s 视为中断，不积分。
      const double gap = stamp - t_prev;
      if (gap > 0.2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "IMU 中断 %.3f s，跳过该段传播（速度状态保持）", gap);
      } else {
        estimator_->propagate(acc_prev * imu_acc_scale_, gyro_prev, gap);
      }
      t_prev = stamp;
    }
    last_imu_stamp_ = t_prev;
    last_imu_acc_ = acc_prev;
    last_imu_gyro_ = gyro_prev;
  }

  // 9) LiDAR 更新（D2）：把 lio/odom 位姿当强先验注入
  {
    Eigen::MatrixXd H_prior;
    Eigen::VectorXd b_prior;
    buildLidarPrior(T_wl_, H_prior, b_prior);
    estimator_->lidarUpdate(H_prior, b_prior, 0.0);
  }

  // 10) 视觉更新
  if (estimator_options_.enable_visual) {
    VisualUpdateInput vin;
    vin.H = H_vis;
    vin.b = b_vis;
    vin.cost = cost_vis;
    vin.quality = quality;
    vin.used_features = used_features;
    estimator_->visualUpdate(vin);
  }

  // 11) 延迟地图提交：状态更新后再用新位姿把本帧点云写入本地地图
  Eigen::Matrix4d T_wl_updated = T_wl_;
  {
    const FilterState& fs = estimator_->filterState();
    T_wl_updated = Eigen::Matrix4d::Identity();
    T_wl_updated.block<3, 3>(0, 0) = fs.state.R;
    T_wl_updated.block<3, 1>(0, 3) = fs.state.p;

    if (scheduler_options_.enable && !body_points_.empty()) {
      PendingCloud pending;
      pending.points = body_points_;
      pending.T_wl = T_wl_updated;
      pending.stamp = body_stamp_;
      scheduler_->stage(pending);
    }
    // flush 返回的点仍以 LiDAR 系存储，这里用更新后的位姿变换到世界系后并入地图。
    for (const auto& cloud : scheduler_->flush(T_wl_updated)) {
      for (const auto& p : cloud.points) {
        const Eigen::Vector4d pw = T_wl_updated * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
        map_points_.emplace_back(pw.head<3>());
      }
    }
    if (map_points_.size() > max_map_points_) {
      map_points_.erase(map_points_.begin(),
                        map_points_.begin() + (map_points_.size() - max_map_points_));
    }
  }

  // 12) 发布：以统一估计器的位姿为准（视觉修正后的结果）
  const auto t_update = std::chrono::steady_clock::now();
  publishOdomAndPath(T_wl_updated, toRosStamp(stamp));

  // 12.5) 缓存修正位姿（按图像 header 时刻），供 color.use_own_odom=true 时着色插值。
  {
    colorizer::PoseSample own;
    own.stamp = stamp;
    own.T_wl = T_wl_updated;
    own_pose_cache_.push_back(own);
    if (own_pose_cache_.size() > max_pose_samples_) {
      own_pose_cache_.erase(own_pose_cache_.begin(),
                            own_pose_cache_.begin() +
                                (own_pose_cache_.size() - max_pose_samples_));
    }
  }

  if (!body_points_.empty() && last_cloud_) {
    runColorizer(*last_cloud_);
  }
  const auto t_end = std::chrono::steady_clock::now();
  auto ms = [](const std::chrono::steady_clock::time_point& a,
               const std::chrono::steady_clock::time_point& b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };

  const UnifiedEstimator::VisualDiagnostics& vd = estimator_->visualDiagnostics();
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "视觉帧: obs=%zu tracks=%zu lv=%zu mscfk=%d used=%d quality=%.2f | "
      "注入 λ=%.2e cap=%.2e scale=%.4f dxθ=%.4frad dxp=%.4fm σp=%.3fm σθ=%.4frad |v|=%.2fm/s | "
      "耗时 ms: 前端=%.1f 约束=%.1f 更新=%.1f 彩色=%.1f 合计=%.1f",
      obs.size(), constraint_tracks.size(), lv_residuals.size(), msckf_eq.used_features,
      used_features, quality, vd.info_lambda, vd.cap_lambda, vd.scale, vd.dx_rot, vd.dx_pos,
      vd.sigma_pos,
      vd.sigma_rot, vd.vel_norm,
      ms(t_start, t_front), ms(t_front, t_vis), ms(t_vis, t_update),
      ms(t_update, t_end), ms(t_start, t_end));
}

void VisualBNode::runColorizer(const sensor_msgs::msg::PointCloud2& cloud_msg) {
  // 与方案 A 相同：lio/body/cloud 已是 LiDAR 系的去畸变点，逐点时间都用扫描帧头。
  // 注意 colorizer 的 Options.T_cl 实现语义为 T_cw（作用于世界系点），逐帧重建 options。
  //
  // 时间基准（与 A 一致，已用离线重放实验确认）：
  //   t_cloud = body_stamp_  → 点云世界系变换
  //   t_pair  = 图像 header.stamp → 选图（实测 |t_pair-t_cloud| < 7 ms）
  //   t_img   = t_pair + img_time_offset_ → 取相机位姿（本数据集最优 0.1 s）
  (void)cloud_msg;
  if (body_points_.empty()) {
    return;
  }

  const double t_cloud = body_stamp_;

  // 把当前 dense 云放入"等图像"队列（去重）：大图回调比云慢约 1 帧，
  // 立即配对会把绝大多数帧误判"无图像"丢弃（bag2 实测 97.7 ms）。
  const bool already_queued =
      std::abs(t_cloud - last_colored_cloud_stamp_) < 1e-6 ||
      std::any_of(pending_clouds_.begin(), pending_clouds_.end(),
                  [t_cloud](const PendingCloudWait& c) {
                    return std::abs(c.t_cloud - t_cloud) < 1e-6;
                  }) ||
      std::any_of(pending_color_.begin(), pending_color_.end(),
                  [t_cloud](const PendingColorFrame& f) {
                    return std::abs(f.t_cloud - t_cloud) < 1e-6;
                  });
  if (!already_queued) {
    PendingCloudWait pc;
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

void VisualBNode::drainPendingClouds() {
  const auto now = std::chrono::steady_clock::now();
  for (auto it = pending_clouds_.begin(); it != pending_clouds_.end();) {
    // 显式配对：按 header.stamp 挑最接近 t_cloud 的一帧图像。
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

void VisualBNode::drainPendingColorization() {
  const auto now = std::chrono::steady_clock::now();
  for (auto it = pending_color_.begin(); it != pending_color_.end();) {
    if (colorizeFrame(*it)) {
      it = pending_color_.erase(it);
      continue;
    }
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

bool VisualBNode::colorizeFrame(const PendingColorFrame& frame) {
  const double t_cloud = frame.t_cloud;
  const double t_img = frame.t_img;

  Eigen::Matrix4d T_wl_img = Eigen::Matrix4d::Identity();
  if (!poseAtStamp(t_img, T_wl_img)) {
    return false;  // 未来时刻的位姿还没发布，稍后重试
  }

  colorizer::ColorizerPipeline::Options options = colorizer_options_;
  const Eigen::Matrix4d T_wc_img = T_wl_img * T_lc_;
  options.T_cl = T_wc_img.inverse();

  // 只把选中的图像交给流水线，时间戳按"配对时刻"给出（流水线只用它判断同步窗口）。
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
                       "dt_pair_cloud=%+.4f dt_img_cloud=%+.4f %zu/%zu 上色 帧#%zu",
                       t_cloud, frame.t_pair, t_img, img_time_offset_,
                       frame.t_pair - t_cloud, t_img - t_cloud, colored_count,
                       colored.size(), stat_colored_frames_);

  accumulateColoredMap(colored, t_cloud);
  publishColoredCloud(colored, toRosStamp(t_cloud));
  return true;
}

bool VisualBNode::poseAtStamp(double stamp, Eigen::Matrix4d& T_wl) const {
  // 查询时刻晚于最新采样时必须拒绝：插值器会直接用最后一个采样顶替，
  // 那等于把 t_img 的位姿退化成 t_cloud 的位姿，offset 就白加了。
  // 轨迹源由 color.use_own_odom 决定：默认 /lio/odom；true 时用统一估计器修正位姿。
  const std::vector<colorizer::PoseSample>& cache =
      color_use_own_odom_ ? own_pose_cache_ : pose_cache_;
  double newest = -std::numeric_limits<double>::max();
  for (const auto& sample : cache) {
    newest = std::max(newest, sample.stamp);
  }
  if (stamp > newest) {
    return false;
  }

  colorizer::LidarPoseInterpolator interp(
      colorizer::LidarPoseInterpolator::Options{kPoseHistoryMaxGapSec});
  for (const auto& sample : cache) {
    interp.addSample(sample);
  }
  return interp.poseAt(stamp, T_wl);
}

void VisualBNode::buildLidarPrior(const Eigen::Matrix4d& T_wl,
                                  Eigen::MatrixXd& H,
                                  Eigen::VectorXd& b) const {
  // D2：把 lio/odom 当作 LiDAR 观测，只在 30 维状态的 R/p 块上注入强先验。
  // 法方程（信息形式）：b = H·(x_cur ⊖ x_prior)，配合 dx = -P·b 会把状态拉回先验。
  H = Eigen::MatrixXd::Zero(StateLayout::kSize, StateLayout::kSize);
  b = Eigen::VectorXd::Zero(StateLayout::kSize);
  if (!estimator_initialized_) {
    return;
  }

  const double sr = std::max(1e-6, prior_rot_sigma_);
  const double sp = std::max(1e-6, prior_pos_sigma_);
  const Eigen::Matrix3d Hr = Eigen::Matrix3d::Identity() / (sr * sr);
  const Eigen::Matrix3d Hp = Eigen::Matrix3d::Identity() / (sp * sp);

  // 状态是 IMU 位姿：T_wi = T_wl · T_li（T_li 单位阵时 T_wi = T_wl）。
  const Eigen::Matrix4d T_wi_prior = T_wl * T_li_;
  const State& s = estimator_->filterState().state;

  // 旋转误差（左扰动）：δ = Log(R_cur · R_prior^T)，则 dx ≈ -δ。
  const Eigen::Vector3d rot_err =
      so3Log(s.R * T_wi_prior.block<3, 3>(0, 0).transpose());
  const Eigen::Vector3d pos_err = s.p - T_wi_prior.block<3, 1>(0, 3);

  H.block<3, 3>(StateLayout::kRot, StateLayout::kRot) = Hr;
  H.block<3, 3>(StateLayout::kPos, StateLayout::kPos) = Hp;
  b.segment<3>(StateLayout::kRot) = Hr * rot_err;
  b.segment<3>(StateLayout::kPos) = Hp * pos_err;
}

std::vector<FeatureTrack> VisualBNode::assembleTracks(
    const std::vector<PatchObservation>& obs, double stamp) const {
  // 本帧每条观测组装成"单观测轨迹"，跨帧合并交给 KeyframeManager::updateTracks。
  const Eigen::Matrix4d T_wc = T_wl_ * T_lc_;
  std::vector<FeatureTrack> tracks;
  tracks.reserve(obs.size());
  for (const auto& o : obs) {
    FeatureTrack track;
    track.id = o.feature_id;
    FeatureObservation observation;
    observation.stamp = stamp;
    observation.pixel = Eigen::Vector2d(o.u, o.v);
    observation.T_wc = T_wc;
    track.addObservation(observation);
    track.quality = std::clamp(static_cast<double>(o.score), 0.0, 1.0);
    track.valid = true;
    tracks.push_back(std::move(track));
  }
  return tracks;
}

void VisualBNode::publishColoredCloud(const std::vector<colorizer::ColoredPoint>& points,
                                      const builtin_interfaces::msg::Time& stamp) {
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
  msg.header.frame_id = "imu";  // 点仍在 LiDAR 系；Super-LIO 名义上叫 imu
  colored_cloud_pub_->publish(msg);
}

void VisualBNode::accumulateColoredMap(const std::vector<colorizer::ColoredPoint>& points,
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

  // 与彩色流水线一致：lio/body/cloud 已去畸变，逐点时间都取扫描帧头，
  // 因此在 stamp 时刻取一次 T_wl，把点从 LiDAR 系搬到世界系。
  // 禁止回退到最新位姿：那会把点云搬到错误时刻的世界系，产生跨帧错位的"重影"。
  Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
  if (!poseAtStamp(stamp, T_wl)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "彩色[丢弃] 累积位姿插值失败: %.4f 缓存=%zu", stamp,
                         pose_cache_.size());
    return;
  }

  const Eigen::Matrix3d R = T_wl.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T_wl.block<3, 1>(0, 3);

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

void VisualBNode::saveColoredMap() {
  if (colored_map_.empty() || colored_map_.size() == pcd_saved_points_) {
    return;  // 无新增点则跳过，避免重复写同一个大文件
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
      (std::filesystem::path(pcd_save_dir_) / "super_livo_visual_b_colored.pcd").string();
  colored_map_.width = static_cast<std::uint32_t>(colored_map_.size());
  colored_map_.height = 1;
  colored_map_.is_dense = false;
  if (pcl::io::savePCDFileBinary(path, colored_map_) == 0) {
    pcd_saved_points_ = colored_map_.size();
    RCLCPP_INFO(this->get_logger(), "彩色地图已保存: %s（%zu 点，世界系）", path.c_str(),
                colored_map_.size());
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "保存 PCD 失败: %s", path.c_str());
  }
}

void VisualBNode::publishOdomAndPath(const Eigen::Matrix4d& T_wl,
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

cv::Mat VisualBNode::rectifyBgr(const cv::Mat& bgr) const {
  if (bgr.empty()) {
    return {};
  }
  const cv::Size size(bgr.cols, bgr.rows);
  if (!rectify_maps_ready_) {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = intrinsics_.fx;
    K.at<double>(1, 1) = intrinsics_.fy;
    K.at<double>(0, 2) = intrinsics_.cx;
    K.at<double>(1, 2) = intrinsics_.cy;
    cv::Mat D = cv::Mat::zeros(1, static_cast<int>(distortion_.size()), CV_64F);
    for (std::size_t i = 0; i < distortion_.size(); ++i) {
      D.at<double>(0, static_cast<int>(i)) = distortion_[i];
    }
    cv::initUndistortRectifyMap(K, D, cv::Mat(), K, size, CV_32FC1,
                                rectify_map1_, rectify_map2_);
    rectify_maps_ready_ = true;
  }
  cv::Mat rectified;
  cv::remap(bgr, rectified, rectify_map1_, rectify_map2_, cv::INTER_LINEAR);
  return rectified;
}

Eigen::Matrix4d VisualBNode::makeTlcFromRclPcl() {
  // Rcl/Pcl 表示 p_cam = Rcl·p_lidar + Pcl（LiDAR -> Camera），
  // 本节点内部用 T_lc（Camera -> LiDAR），因此需求逆（同方案 A 的说明）。
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

Eigen::Matrix4d VisualBNode::applyLeftPerturbation(const Eigen::Matrix4d& T,
                                                   const Eigen::VectorXd& dx) const {
  // dx = [φ; ρ]：R' = Exp(φ)·R，p' = Exp(φ)·p + ρ。
  if (dx.size() < 6) {
    return T;
  }
  const Eigen::Matrix3d dR = so3Exp(dx.head<3>());
  Eigen::Matrix4d T2 = T;
  T2.block<3, 3>(0, 0) = dR * T.block<3, 3>(0, 0);
  T2.block<3, 1>(0, 3) = dR * T.block<3, 1>(0, 3) + dx.tail<3>();
  return T2;
}

}  // namespace superlivo_b

// ----------------------------------------------------------------------------
// 入口：标准 ROS2 节点 main。
// ----------------------------------------------------------------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<superlivo_b::VisualBNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
