#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "super_livo_visual_b/bridge/constraint_bridge.hpp"
#include "super_livo_visual_b/constraint/lidar_visual_constraint.hpp"
#include "super_livo_visual_b/constraint/msckf_constraint.hpp"
#include "super_livo_visual_b/constraint/photometric_constraint.hpp"
#include "super_livo_visual_b/estimator/map_commit_scheduler.hpp"
#include "super_livo_visual_b/estimator/rotation_estimator.hpp"
#include "super_livo_visual_b/estimator/rotation_refiner.hpp"
#include "super_livo_visual_b/estimator/unified_estimator.hpp"
#include "super_livo_visual_b/feature_map/keyframe_manager.hpp"
#include "super_livo_visual_b/frontend/feature_lifecycle.hpp"
#include "super_livo_visual_b/frontend/patch_tracker.hpp"
#include "super_livo_visual_b/state/state_layout.hpp"
#include "pointcloud_colorizer/colorizer_pipeline.hpp"
#include "pointcloud_colorizer/voxel_color_median.hpp"
#include "super_livo_common/degeneracy_signature.hpp"
#include "super_livo_common/latency_timer.hpp"

namespace superlivo_b {

// ============================================================================
// 方案 B · ROS 无关视觉流水线（单体化 S2）
// ----------------------------------------------------------------------------
// 本类是 VisualBNode 的"去 ROS 化"等价实现：
//   * 输入全部是普通类型（cv::Mat / Eigen::Matrix4d / std::vector<double>），
//     由 visual_b_app 在 RO 回调里转换后 submit*；
//   * 输出全部走 std::function 回调，由 app 负责发布；
//   * 算法模块（Patch 前端 / 生命周期 / 关键帧 / 2D-3D / MSCKF / 光度 /
//     统一估计器 / 上色 / 落盘）与参数语义一行未改。
//
// 内部仍是三条线程（与 VisualBNode 完全一致）：
//   ① submit*：只把普通类型事件放进 input_queue_，立即返回；
//   ② 工作线程：按 FIFO 串行执行全部算法（所有算法状态仅此线程访问）；
//   ③ 落盘线程：只做彩色地图增量写盘。
// 这样 SuperLIO 的 OutputThread / DenseThread 不会被视觉处理阻塞。
//
// 视觉约束写回 ESKF 仍走代码级 ConstraintBridge（publishVisualConstraint 内 push），
// 与 ROS 无关。
// ============================================================================

/// 参数读取抽象：由 app 用 ROS 参数实现，流水线本身不依赖 rclcpp。
/// 这样 loadParameters() 可以逐行保留原来的参数名与默认值。
class ParamSource {
 public:
  virtual ~ParamSource() = default;
  virtual bool getBool(const std::string& name, bool def) = 0;
  virtual int getInt(const std::string& name, int def) = 0;
  virtual double getDouble(const std::string& name, double def) = 0;
  virtual float getFloat(const std::string& name, float def) = 0;
  virtual std::string getString(const std::string& name, const std::string& def) = 0;
  virtual std::vector<double> getDoubleArray(const std::string& name,
                                             const std::vector<double>& def) = 0;
};

/// 进程内视觉流水线（方案 B）。
class VisualPipeline {
 public:
  enum class LogLevel { kInfo, kWarn, kError };

  /// 彩色点云输出（点表达在 LiDAR 系，时间为点云时刻）。
  using ColoredCloudCallback =
      std::function<void(const std::vector<colorizer::ColoredPoint>& points, double stamp)>;
  /// 位姿输出（T_wl，时间为实际取用的位姿时刻）。
  using OdomCallback = std::function<void(const Eigen::Matrix4d& T_wl, double stamp)>;
  /// 日志输出（节流在内部完成，回调只负责转发到 ROS 日志宏）。
  using LogCallback = std::function<void(LogLevel level, const std::string& msg)>;

  /// @param params 参数源（生命周期需覆盖本对象）
  /// @param bridge 视觉->ESKF 桥；为空时不做任何注入
  VisualPipeline(ParamSource& params, std::shared_ptr<ConstraintBridge> bridge);
  ~VisualPipeline();

  VisualPipeline(const VisualPipeline&) = delete;
  VisualPipeline& operator=(const VisualPipeline&) = delete;

  // ---- 输出注册（应在第一个输入之前调用） ----
  void setColoredCloudCallback(ColoredCloudCallback cb);
  void setOdomCallback(OdomCallback cb);
  /// 旋转精修通道（rot_refine.enable=false 时不会触发）。
  void setRotRefinedOdomCallback(OdomCallback cb);
  void setLogCallback(LogCallback cb);

  // ---- 输入（线程安全，入队后立即返回） ----
  /// BGR 图像；内部完成去畸变 + 灰度。
  /// keepalive：允许调用方传入浅拷贝 Mat（如 cv_bridge::toCvShare 直接引用
  /// ROS 消息内存）时保活底层缓冲；事件消费完即释放。深拷贝可不传。
  void submitImage(const cv::Mat& bgr, double stamp,
                   std::shared_ptr<const void> keepalive = nullptr);
  /// T_w = LIO 位姿（原 lio/odom 的 pose 部分；单体钩子直供 IMU 系位姿）。
  void submitOdom(const Eigen::Matrix4d& T_w, double stamp);
  /// 独立的相机位姿源（原 color.cam_odom_topic）；不配置时与 submitOdom 同源。
  void submitCamOdom(const Eigen::Matrix4d& T_w, double stamp);
  /// 世界系点云（原 lio/cloud_world）：当前扫描 + 局部地图。
  void submitWorldCloud(const std::vector<Eigen::Vector3d>& points_world);
  /// 机体系全量点云（原 lio/cloud_body_dense）：仅用于彩色点云。
  void submitBodyCloud(const std::vector<Eigen::Vector3f>& points_lidar, double stamp);
  /// IMU 样本（原 /livox/imu，LiDAR 系量测）。
  void submitImu(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double stamp);
  /// 退化签名（原 lio/degeneracy；单体内由 SuperLIO 快照直供）。
  void submitDegeneracy(const superlivo_common::DegeneracySignature& sig);

  // ---- 控制 ----
  /// 请求一次彩色地图落盘（在内部工作线程执行，不阻塞调用方）。
  void requestSaveColoredMap();
  /// 等待已入队事件全部处理完（用于退出前补完尾段）。
  void flush();
  /// 停机并**同步**写出最终彩色地图（幂等，可重复调用）。
  /// 退出前必须调用：异步 save 线程会随进程结束被直接杀掉，长序列上
  /// 大图还没写完进程就退出了，产物会整个丢失。
  void shutdownAndSave();
  /// 彩色/队列统计（退出打印）。
  std::string statsString() const;

  /// 若 odom 发布的是机体系（robot body），用本接口注入 T_imu_body。
  /// 单体钩子直供 IMU 系位姿时保持默认单位阵。
  void setOdomBodyExtrinsic(const Eigen::Matrix4d& T_imu_body) { T_imu_robo_ = T_imu_body; }

 private:
  // ---- 参数 ----
  void loadParameters();

  // ---- 输入事件处理（全部在工作线程串行执行） ----
  void onImage(const cv::Mat& bgr, double stamp);
  void onDegeneracy(const superlivo_common::DegeneracySignature& sig);
  bool degeneracySignature(double& severity, Eigen::Vector3d& dir_w) const;
  void onOdom(const Eigen::Matrix4d& T_w, double stamp);
  void onCamOdom(const Eigen::Matrix4d& T_w, double stamp);
  void onCloud(const std::vector<Eigen::Vector3d>& points_world);
  void onBodyCloud(const std::vector<Eigen::Vector3f>& points_lidar, double stamp);
  void onImu(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double stamp);

  // ---- 启动时间对齐 ----
  void feedAlignSample(bool is_image, double header,
                       std::chrono::steady_clock::time_point arrival);
  void tryAlign();

  // ---- 主流程 ----
  void processVisualFrame(const cv::Mat& gray, double stamp);
  void publishVisualConstraint(double stamp);
  void refineRotation(double stamp, double t_pose,
                      const std::vector<PatchObservation>& obs);
  Eigen::Matrix3d whiteningFromCov(const Eigen::Matrix3d& cov) const;
  void runColorizer();

  struct PendingColorFrame {
    std::vector<Eigen::Vector3f> points_lidar;
    double t_cloud{0.0};
    double t_pair{0.0};
    double t_img{0.0};
    cv::Mat image;
    std::chrono::steady_clock::time_point queued_at;
  };

  void drainPendingColorization();
  bool colorizeFrame(const PendingColorFrame& frame);
  void pushOwnPoseSample(double stamp, const Eigen::Matrix4d& T_w_lio);
  bool poseAtStamp(double stamp, Eigen::Matrix4d& T_w) const;
  bool camPoseAtStamp(double stamp, Eigen::Matrix4d& T_w) const;

  // ---- 辅助 ----
  cv::Mat rectifyBgr(const cv::Mat& bgr) const;
  Eigen::Matrix4d makeTlcFromRclPcl() const;
  Eigen::Matrix4d applyLeftPerturbation(const Eigen::Matrix4d& T,
                                        const Eigen::VectorXd& dx) const;
  void buildLidarPrior(const Eigen::Matrix4d& T_w, double t_pose,
                       Eigen::MatrixXd& H, Eigen::VectorXd& b) const;
  bool lioVelocity(double t_pose, Eigen::Vector3d& v_lio) const;
  std::vector<FeatureTrack> assembleTracks(
      const std::vector<PatchObservation>& obs, double stamp) const;
  void publishColoredCloud(const std::vector<colorizer::ColoredPoint>& points, double stamp);
  void accumulateColoredMap(const std::vector<colorizer::ColoredPoint>& points, double stamp);
  void saveColoredMap();
  void publishOdomAndPath(const Eigen::Matrix4d& T_w, double stamp);

  // ---- 日志（printf 风格，节流在内部完成） ----
  void logf(LogLevel level, const char* fmt, ...) const;
  void logfThrottled(LogLevel level, int period_ms, const char* fmt, ...) const;

  // ---- 算法模块 ----
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

  // ---- 相机/外参/时间 ----
  colorizer::CameraIntrinsics intrinsics_;
  std::vector<double> distortion_;
  double scale_{0.5};
  Eigen::Matrix4d T_lc_{Eigen::Matrix4d::Identity()};  ///< Camera->LiDAR
  Eigen::Matrix4d T_li_{Eigen::Matrix4d::Identity()};  ///< LiDAR->IMU
  Eigen::Matrix4d T_imu_robo_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_wl_{Eigen::Matrix4d::Identity()};  ///< 当前位姿
  double last_odom_stamp_{0.0};
  double img_time_offset_{0.0};

  // ---- 启动时间对齐 ----
  /// off = 雷达帧戳 − 图像 header = −(D + a)：
  ///   D = 系统钟 − 雷达钟（IMU 与雷达同钟域，由 (到达, header) 现测）
  ///   a = 相机打戳延迟（一次性标定）
  /// 10 Hz 下整数周期无法只靠时刻分辨，必须靠这个绝对锚点。
  struct AlignSample {
    double header{0.0};
    std::chrono::steady_clock::time_point arrival;
  };
  std::deque<AlignSample> align_images_;
  std::deque<AlignSample> align_lidar_;
  std::deque<double> align_d_samples_;  ///< 系统钟 − IMU header
  bool align_enable_{true};
  bool align_done_{false};
  bool align_failed_{false};
  std::size_t align_samples_{20};
  double align_window_ms_{50.0};
  double align_img_stamp_latency_s_{0.111};  ///< 相机打戳延迟标定值（秒）
  double align_clock_lag_max_s_{2.0};        ///< 系统钟与图像 header 的最大允许差（秒）
  double align_timeout_s_{10.0};
  std::chrono::steady_clock::time_point align_t0_{};

  // ---- E2 诊断落盘 ----
  bool e2_dump_enable_{false};
  std::string e2_dump_dir_;
  std::ofstream e2_pose_ofs_;
  std::ofstream e2_obs_ofs_;

  // ---- 旋转后处理 ----
  bool rot_refine_enable_{false};
  std::string rot_noise_mode_{"w3"};
  RotationEstimator::Options rot_estimator_options_;
  RotationRefiner::Options rot_refine_options_;
  std::unique_ptr<RotationEstimator> rot_estimator_;
  std::unique_ptr<RotationRefiner> rot_refiner_;
  std::ofstream rot_refine_ofs_;
  struct RotPrevFrame {
    bool valid{false};
    double t_pose{0.0};
    Eigen::Matrix4d T_wl{Eigen::Matrix4d::Identity()};
    std::unordered_map<int, Eigen::Vector2d> obs;
  };
  RotPrevFrame rot_prev_;
  Eigen::Matrix3d R_bc_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_bc_{Eigen::Vector3d::Zero()};
  std::size_t stat_rot_frames_{0};
  std::size_t stat_rot_usable_{0};

  // ---- 数据缓存 ----
  std::vector<colorizer::TimedImage> image_cache_;
  std::vector<colorizer::PoseSample> pose_cache_;
  std::vector<colorizer::PoseSample> cam_pose_cache_;
  std::vector<Eigen::Vector3d> map_points_;
  bool world_cloud_valid_{false};

  std::vector<Eigen::Vector3f> body_points_;
  double body_stamp_{0.0};

  std::vector<int> prev_feature_ids_;
  int next_keyframe_id_{0};

  mutable cv::Mat rectify_map1_;
  mutable cv::Mat rectify_map2_;
  mutable bool rectify_maps_ready_{false};

  std::size_t max_image_cache_{30};
  std::size_t max_pose_samples_{2000};
  std::size_t max_map_points_{60000};
  std::size_t max_imu_samples_{1200};
  int map_append_stride_{3};

  // ---- LiDAR 先验（D2）参数 ----
  double prior_pos_sigma_{0.05};
  double prior_rot_sigma_{0.02};
  double prior_vel_sigma_{0.25};
  double own_res_max_pos_{0.5};
  double own_res_max_rot_{0.035};
  double own_anchor_t_{-1.0};
  Eigen::Vector3d own_anchor_dp_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d own_anchor_dR_{Eigen::Matrix3d::Identity()};
  double prior_pos_sigma_healthy_{0.005};
  double prior_pos_sigma_degen_{1.0};
  bool prior_directional_en_{true};
  bool guard_enable_{true};
  double guard_lio_jump_pos_{1.0};
  double guard_lio_jump_rot_{0.2618};
  double guard_sev_threshold_{0.9};
  double guard_vel_ratio_{1.5};
  double guard_vel_fallback_{3.0};
  double guard_vel_abs_cap_{3.0};
  double vel_ref_{0.0};
  bool vel_ref_valid_{false};
  Eigen::Matrix4d last_trust_T_wl_{Eigen::Matrix4d::Identity()};
  double last_trust_stamp_{-1.0};
  std::size_t stat_prior_veto_{0};
  std::size_t stat_vel_clamp_{0};
  std::size_t stat_no_imu_{0};

  struct ImuSample {
    double stamp{0.0};
    Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  };
  std::vector<ImuSample> imu_buffer_;
  double imu_acc_scale_{1.0};

  // ---- LiDAR 退化签名 ----
  mutable std::mutex degen_mutex_;
  std::string degen_topic_;
  superlivo_common::DegeneracySignature degen_sig_;
  double degen_recv_time_{-1.0};
  bool degen_stale_warned_{false};
  double gate_cond_lo_{superlivo_common::DegeneracyScale{}.cond_lo};
  double gate_cond_hi_{superlivo_common::DegeneracyScale{}.cond_hi};
  double degen_timeout_{superlivo_common::DegeneracyScale{}.timeout_sec};
  Eigen::Matrix4d T_wl_frame_{Eigen::Matrix4d::Identity()};

  // ---- 视觉约束桥接 ----
  std::shared_ptr<ConstraintBridge> bridge_;

  // ---- 运行状态 ----
  bool imu_initialized_{false};
  bool estimator_initialized_{false};
  double last_imu_stamp_{0.0};
  Eigen::Vector3d last_imu_acc_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_imu_gyro_{Eigen::Vector3d::Zero()};

  // ---- 彩色地图落盘 ----
  pcl::PointCloud<pcl::PointXYZRGB> colored_map_;
  colorizer::VoxelColorMedian median_map_;
  std::string pcd_save_dir_{"/home/ywj/slam_data/pcd"};
  double pcd_save_interval_sec_{30.0};
  std::size_t max_colored_map_points_{20000000};
  bool pcd_capacity_warned_{false};
  std::size_t pcd_saved_points_{0};

  // ---- 彩色着色调度 ----
  std::deque<PendingColorFrame> pending_color_;
  std::chrono::steady_clock::time_point last_color_drain_{};

  struct PendingCloudWait {
    std::vector<Eigen::Vector3f> points_lidar;
    double t_cloud{0.0};
    std::chrono::steady_clock::time_point queued_at;
  };
  std::deque<PendingCloudWait> pending_clouds_;
  void drainPendingClouds();

  // ---- 视觉帧调度 ----
  struct PendingVisualFrame {
    cv::Mat gray;
    double stamp{0.0};
    std::chrono::steady_clock::time_point queued_at;
  };
  std::deque<PendingVisualFrame> pending_visual_;
  void drainPendingVisualFrames();

  bool color_use_own_odom_{false};
  std::vector<colorizer::PoseSample> own_pose_cache_;

  double last_colored_cloud_stamp_{-1.0};
  std::size_t stat_colored_frames_{0};
  std::size_t stat_drop_no_image_{0};
  std::size_t stat_drop_pose_{0};
  std::size_t stat_deferred_{0};

  // ==========================================================================
  // 线程化设施（与 VisualBNode 完全一致）
  // ==========================================================================
  struct InputEvent {
    enum class Kind : std::uint8_t {
      kImage,
      kOdom,
      kCamOdom,
      kCloud,
      kBodyCloud,
      kImu,
      kDegeneracy,
      kSaveSnapshot,
      kBarrier,
      kQuit
    };
    Kind kind{Kind::kQuit};
    cv::Mat image;                            ///< BGR（可能是引用消息内存的浅拷贝）
    std::shared_ptr<const void> image_keepalive;  ///< 浅拷贝时保活底层缓冲（如 ROS 消息）
    double stamp{0.0};
    Eigen::Matrix4d T{Eigen::Matrix4d::Identity()};
    std::vector<Eigen::Vector3d> points_world;
    std::vector<Eigen::Vector3f> points_body;
    Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
    superlivo_common::DegeneracySignature degen;
    std::shared_ptr<std::promise<void>> barrier;
    std::chrono::steady_clock::time_point enqueued{};
  };

  void startWorkerThreads();
  void stopWorkerThreads();
  bool pushInputEvent(InputEvent event);
  void workerLoop();
  void saveLoop();
  /// 每 60 s 打印一次延时窗口（timing_en_ 开启且有记录时），工作线程调用
  void maybePrintLatencyWindow();

  struct ColorSegment {
    pcl::PointCloud<pcl::PointXYZRGB> points;
    std::shared_ptr<colorizer::VoxelColorMedian> median;
  };

  void requestColoredMapSave();
  void writeColoredSnapshot(std::deque<ColorSegment> segments);

  std::deque<InputEvent> input_queue_;
  std::mutex input_mutex_;
  std::condition_variable input_cv_;
  bool input_stop_{false};
  std::atomic<std::size_t> stat_input_dropped_{0};
  std::thread worker_;

  // 延时计时（口径仿 Super-LIO lio.eva.timer）：img_enqueue_wall_ /
  // last_timing_report_ 仅工作线程读写；latency_timer_ 内部加锁。
  bool timing_en_{false};
  superlivo_common::LatencyTimer latency_timer_;
  std::chrono::steady_clock::time_point img_enqueue_wall_{};
  std::chrono::steady_clock::time_point last_timing_report_{};

  std::size_t save_handed_points_{0};
  std::deque<ColorSegment> save_segments_;
  bool save_pending_{false};
  std::mutex save_mutex_;
  std::condition_variable save_cv_;
  bool save_stop_{false};
  std::thread save_thread_;
  pcl::PointCloud<pcl::PointXYZRGB> save_buffer_;
  std::shared_ptr<colorizer::VoxelColorMedian> save_median_;
  /// shutdownAndSave() 只执行一次；并发调用者（shutdown 钩子 / 主线程 /
  /// 析构）会阻塞到那次执行完成，保证进程退出前文件已落盘。
  std::once_flag shutdown_once_;

  // ---- 探针 ----
  void probeMaybeReport();
  void probeReport();

  ParamSource& params_;
  ColoredCloudCallback colored_cb_;
  OdomCallback odom_cb_;
  OdomCallback rot_odom_cb_;
  LogCallback log_cb_;
};

}  // namespace superlivo_b
