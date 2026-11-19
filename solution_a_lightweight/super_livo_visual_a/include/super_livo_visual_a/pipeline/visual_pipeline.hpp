#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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
// 方案 A · ROS 无关视觉流水线
// ----------------------------------------------------------------------------
// 本类是 VisualANode 的"去 ROS 化"等价实现：所有输入都是普通类型（cv::Mat /
// Eigen::Matrix4d / std::vector），所有输出走 std::function 回调。
//
// 单体化后的数据流（进程内、无 DDS）：
//   SuperLIO 各输出线程 ──(submit*)──► 内部任务队列 ──► 单工作线程串行执行
//   图像（外部 ROS 输入）─(submitImage)─┘
//   工作线程 ──► ColoredCloudCallback / OdomCallback ──► ROS 发布（对外输出）
//
// 为什么用单工作线程：
//   * 原始 VisualANode 的状态（frontend_/pose_cache_/pending_*/pose_offset_ 等）
//     全部依赖回调串行访问；原实现靠 MultiThreadedExecutor + 逻辑上的先后关系，
//     实际存在数据竞争窗口。这里显式串行化，行为更严格且可复现。
//   * SuperLIO 的 OutputThread/DenseThread 只做 memcpy + 入队，不会被视觉处理阻塞。
//
// 算法模块（KLT/深度关联/重投影/门控/更新/上色）一行未改，全部沿用原参数语义。
// ============================================================================

/// 流水线配置（对应原 VisualANode::loadParameters() 的普通结构体版本）。
struct VisualPipelineOptions {
  // ---- 相机内参与畸变（scale 0.5 后） ----
  colorizer::CameraIntrinsics intrinsics;
  std::vector<double> distortion{-0.076160, 0.123001, -0.00113, 0.000251};
  /// 相机-LiDAR 外参（avia_lvi.yaml 的 Rcl/Pcl，行主序 3x3 + 3 维平移）。
  /// 语义：p_cam = Rcl · p_lidar + Pcl（即 T_cam_lidar），内部取其逆得 T_lc。
  std::vector<double> Rcl{1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::vector<double> Pcl{0, 0, 0};
  /// 图像真实采集时刻 = header.stamp + 该偏移（秒），仅用于取相机位姿。
  double img_time_offset{0.1};

  // ---- 算法模块参数 ----
  KltFrontend::Options klt;
  LidarDepthAssociation::Options depth;
  ReprojectionConstraint::Options reproj;
  DegeneracyGate::Options gate;
  VisualUpdate::Options update;
  colorizer::ColorizerPipeline::Options colorizer;

  // ---- 退化严重度映射（与 super_livo_common::DegeneracyScale 同口径） ----
  double gate_cond_lo{superlivo_common::DegeneracyScale{}.cond_lo};
  double gate_cond_hi{superlivo_common::DegeneracyScale{}.cond_hi};
  double degen_timeout{superlivo_common::DegeneracyScale{}.timeout_sec};

  // ---- 运行模式 ----
  /// 着色用哪条轨迹：false=LIO 原始位姿，true=本流水线视觉修正后的位姿。
  bool color_use_own_odom{false};
  /// 视觉链路是否用"图像曝光时刻"的插值位姿（关掉即回到旧行为，用于判别实验）。
  bool use_exposure_pose{true};
  /// 持久偏移的泄漏衰减（健康方向 τ≈1/rate 秒）。
  bool offset_decay_en{true};
  double offset_decay_rate{1.0};
  /// 状态回灌：视觉 dx 写回 own_pose_cache_ 尾部采样，替代持久偏移模式。
  bool reinject{true};

  // ---- 缓存规模 ----
  std::size_t max_image_cache{30};
  std::size_t max_pose_samples{300};
  std::size_t max_map_points{60000};
  int map_append_stride{3};
  std::size_t max_colored_map_points{20000000};

  // ---- 彩色地图落盘 ----
  std::string pcd_save_dir{"/home/ywj/slam_data/pcd"};
  /// 时间中位数体素边长（米），<=0 关闭。
  float median_voxel_size{0.01F};

  // ---- 轨迹 / 图像落盘（参照 FAST-LIVO2 的 evo / image_save 开关） ----
  /// 轨迹输出开关：TUM 格式（timestamp tx ty tz qx qy qz qw），
  /// 位姿为 camera_init（Super-LIO world）系下的 LiDAR 位姿 T_wl，每个 LiDAR 帧一行。
  bool pose_output_en{false};
  /// 轨迹文件名（不含 .txt），最终文件 = <pose_save_dir>/<pose_seq_name>.txt。
  std::string pose_seq_name{"scene_move"};
  /// 轨迹输出目录；空 = pcd_save_dir。
  std::string pose_save_dir;
  /// 每 N 个 LiDAR 帧写一行（<=0 关闭）。
  int pose_save_interval{1};

  /// 图像输出开关：PNG，文件名 = LIO 时间域时间戳（header + img_time_offset，
  /// 与轨迹时间戳同域），固定 6 位小数。
  bool img_save_en{false};
  /// 图像输出目录；空 = <pcd_save_dir>/image。
  std::string img_save_dir;
  /// 每 N 张图像写一张（<=0 关闭）。
  int img_save_interval{1};
};

/// 进程内视觉流水线（方案 A）。
class VisualPipeline {
 public:
  enum class LogLevel { kInfo, kWarn, kError };

  /// 彩色点云输出（点仍表达在 LiDAR 系，时间为点云时刻）。
  using ColoredCloudCallback =
      std::function<void(const std::vector<colorizer::ColoredPoint>& points, double stamp)>;
  /// 修正后位姿输出（T_wl，时间为实际取用的位姿时刻）。
  using OdomCallback = std::function<void(const Eigen::Matrix4d& T_wl, double stamp)>;
  /// 日志输出（节流在内部完成，回调只负责转发到 ROS 日志宏）。
  using LogCallback = std::function<void(LogLevel level, const std::string& msg)>;

  explicit VisualPipeline(const VisualPipelineOptions& options);
  ~VisualPipeline();

  VisualPipeline(const VisualPipeline&) = delete;
  VisualPipeline& operator=(const VisualPipeline&) = delete;

  // ---- 输出注册（应在第一个输入之前调用） ----
  void setColoredCloudCallback(ColoredCloudCallback cb);
  void setOdomCallback(OdomCallback cb);
  void setLogCallback(LogCallback cb);

  // ---- 输入（线程安全，入队后立即返回；BGR 图像内部完成去畸变+灰度） ----
  void submitImage(const cv::Mat& bgr, double stamp);
  /// T_wl = LIO 位姿（原 lio/odom；child_frame_id 标 imu 但实为 LiDAR 位姿）。
  void submitOdom(const Eigen::Matrix4d& T_wl, double stamp);
  /// 独立的相机位姿源（原 color.cam_odom_topic）；不配置时与 submitOdom 同源。
  void submitCamOdom(const Eigen::Matrix4d& T_wl, double stamp);
  /// 世界系点云（原 lio/cloud_world）：当前扫描 + 局部地图（供深度关联）。
  void submitWorldCloud(const std::vector<Eigen::Vector3d>& points_world);
  /// 机体系全量点云（原 lio/cloud_body_dense）：仅用于彩色点云。
  void submitBodyCloud(const std::vector<Eigen::Vector3f>& points_lidar, double stamp);
  /// 退化签名（原 lio/degeneracy；单体内由 SuperLIO 快照直供）。
  void submitDegeneracy(const superlivo_common::DegeneracySignature& sig);

  // ---- 控制 ----
  /// 请求一次彩色地图落盘（在内部工作线程执行，不阻塞调用方）。
  void requestSaveColoredMap();
  /// 阻塞直到已入队任务全部处理完（退出前调用，保证队列里的帧不被丢弃）。
  void flush();
  /// 统计摘要（与原节点的退出日志同口径）。
  std::string statsString() const;

 private:
  /// 一帧待着色数据：点云 + 配对图像 + 两个时刻。
  struct PendingColorFrame {
    std::vector<Eigen::Vector3f> points_lidar;  ///< LiDAR 系点（已去畸变到 t_cloud）
    double t_cloud{0.0};                        ///< 点云时刻（扫描结束）
    double t_pair{0.0};                         ///< 图像 header 时刻（配对用）
    double t_img{0.0};                          ///< 图像真实采集时刻（取位姿用）
    cv::Mat image;                              ///< 去畸变后的 BGR 图像
    std::chrono::steady_clock::time_point queued_at;  ///< 入队时刻，用于超时丢弃
  };
  /// 一帧等待配图的点云。
  struct PendingCloud {
    std::vector<Eigen::Vector3f> points_lidar;
    double t_cloud{0.0};
    std::chrono::steady_clock::time_point queued_at;
  };

  // ---- 工作线程 ----
  void workerLoop();
  void enqueue(std::function<void()> task, std::size_t cost = 1);

  // ---- 输入处理（仅工作线程执行） ----
  void handleImage(const cv::Mat& bgr, double stamp);
  void handleOdom(const Eigen::Matrix4d& T_wl, double stamp);
  void handleCamOdom(const Eigen::Matrix4d& T_wl, double stamp);
  void handleWorldCloud(const std::vector<Eigen::Vector3d>& points_world);
  void handleBodyCloud(const std::vector<Eigen::Vector3f>& points_lidar, double stamp);

  // ---- 主流程 ----
  void processVisualFrame(const cv::Mat& gray, double stamp);
  void runColorizer();
  void drainPendingClouds();
  void drainPendingColorization();
  bool colorizeFrame(const PendingColorFrame& frame);

  // ---- 位姿查询 ----
  /// clamp_future：视觉定位路径专用。相机与雷达硬件级触发同步，按图像时刻查询时
  /// 最新采样通常只早几毫秒，直接取最新采样而不是等下一帧 odom。
  bool poseAtStamp(double stamp, Eigen::Matrix4d& T_wl, bool clamp_future = false) const;
  bool poseAtStampCorrected(double stamp, Eigen::Matrix4d& T_wl,
                            bool clamp_future = false) const;
  bool camPoseAtStamp(double stamp, Eigen::Matrix4d& T_wl) const;

  // ---- 辅助 ----
  cv::Mat rectifyBgr(const cv::Mat& bgr) const;
  Eigen::Matrix4d makeTlcFromRclPcl(const std::vector<double>& rcl,
                                    const std::vector<double>& pcl) const;
  Eigen::Matrix4d applyLeftPerturbation(const Eigen::Matrix4d& T,
                                        const Eigen::VectorXd& dx) const;
  bool degeneracySignature(double& severity, Eigen::Vector3d& dir_w) const;
  void decayPoseOffset(double dt, double severity, const Eigen::Vector3d& dir_w);
  void accumulateColoredMap(const std::vector<colorizer::ColoredPoint>& points, double stamp);
  void saveColoredMap();
  void savePose(const Eigen::Matrix4d& T_wl, double stamp);
  void saveImage(const cv::Mat& bgr, double stamp);
  void publishColoredCloud(const std::vector<colorizer::ColoredPoint>& points, double stamp);
  void publishOdomAndPath(const Eigen::Matrix4d& T_wl, double stamp);

  void log(LogLevel level, const std::string& msg) const;
  void logThrottled(const std::string& key, int period_ms, LogLevel level,
                    const std::string& msg) const;
  static double steadyNowSec();

  // ---- 工作线程状态 ----
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::condition_variable idle_cv_;
  std::deque<std::function<void()>> tasks_;
  std::size_t pending_cost_{0};
  bool stop_{false};
  std::thread worker_;
  /// 「待处理工作量」上限：图像/点云帧按 1 计。超过时丢弃最老任务会破坏状态机，
  /// 因此这里只做告警不丢帧（QoS 端已经限深）。
  std::size_t queue_high_water_{64};

  // ---- 输出回调 ----
  mutable std::mutex cb_mutex_;
  ColoredCloudCallback colored_cb_;
  OdomCallback odom_cb_;
  LogCallback log_cb_;

  // ---- 日志节流（仅工作线程写） ----
  mutable std::map<std::string, std::chrono::steady_clock::time_point> log_last_;

  // ---- 配置与算法模块（构造后不变） ----
  VisualPipelineOptions opt_;
  std::unique_ptr<KltFrontend> frontend_;
  std::unique_ptr<LidarDepthAssociation> depth_assoc_;
  std::unique_ptr<ReprojectionConstraint> reproj_;
  std::unique_ptr<DegeneracyGate> gate_;
  std::unique_ptr<VisualUpdate> visual_update_;
  colorizer::ColorizerPipeline::Options colorizer_options_;

  // ---- 相机/外参 ----
  colorizer::CameraIntrinsics intrinsics_;
  std::vector<double> distortion_;
  Eigen::Matrix4d T_lc_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_wl_{Eigen::Matrix4d::Identity()};
  double img_time_offset_{0.0};
  mutable bool rectify_maps_ready_{false};
  mutable cv::Mat rectify_map1_;
  mutable cv::Mat rectify_map2_;

  // ---- 数据缓存 ----
  std::vector<colorizer::TimedImage> image_cache_;
  std::vector<colorizer::PoseSample> pose_cache_;
  std::vector<colorizer::PoseSample> cam_pose_cache_;
  std::vector<Eigen::Vector3d> local_map_;
  std::vector<Eigen::Vector3d> sweep_;
  std::vector<Eigen::Vector3f> body_points_;
  double body_stamp_{0.0};
  bool world_cloud_valid_{false};

  // ---- 视觉修正状态 ----
  Eigen::Matrix4d pose_offset_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_lio_prev_raw_{Eigen::Matrix4d::Identity()};
  bool lio_prev_raw_valid_{false};
  std::vector<colorizer::PoseSample> own_pose_cache_;
  double last_visual_stamp_{-1.0};
  double last_odom_stamp_{-1.0};

  // ---- 退化签名 ----
  mutable std::mutex degen_mutex_;
  superlivo_common::DegeneracySignature degen_sig_;
  double degen_recv_time_{-1.0};
  bool degen_stale_warned_{false};
  bool degen_seen_{false};

  // ---- 彩色地图 ----
  pcl::PointCloud<pcl::PointXYZRGB> colored_map_;
  colorizer::VoxelColorMedian median_map_;
  bool pcd_capacity_warned_{false};
  std::size_t pcd_saved_points_{0};
  std::deque<PendingColorFrame> pending_color_;
  std::deque<PendingCloud> pending_clouds_;
  double last_colored_cloud_stamp_{-1.0};

  // ---- 轨迹 / 图像落盘（仅工作线程写） ----
  std::ofstream pose_file_;
  std::string img_save_dir_resolved_;
  std::size_t pose_frame_count_{0};
  std::size_t img_frame_count_{0};

  // ---- 统计 ----
  std::size_t stat_colorizer_calls_{0};
  std::size_t stat_colored_frames_{0};
  std::size_t stat_drop_no_image_{0};
  std::size_t stat_drop_pose_{0};
  std::size_t stat_deferred_{0};
  std::size_t stat_skip_duplicate_{0};
  std::size_t stat_drop_accum_{0};
};

}  // namespace superlivo_a
