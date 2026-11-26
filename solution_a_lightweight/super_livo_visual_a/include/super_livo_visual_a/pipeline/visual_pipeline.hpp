#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <future>
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
#include "super_livo_common/latency_timer.hpp"

namespace superlivo_a {

// ============================================================================
// 方案 A · ROS 无关视觉流水线
// ----------------------------------------------------------------------------
// 本类是 VisualANode 的"去 ROS 化"等价实现：所有输入都是普通类型（cv::Mat /
// Eigen::Matrix4d / std::vector），所有输出走 std::function 回调。
//
// 单体化后的数据流（进程内、无 DDS），内部四条线程：
//   ① submit*（ROS 回调 / SuperLIO 输出线程）：只把带类型的事件放进对应队列
//      （机体系点云进 color_queue_，其余进 input_queue_），零算法、立即返回；
//   ② 工作线程：按 FIFO 串行执行视觉定位算法（去畸变 + KLT/深度/重投影/门控/
//      更新），并把去畸变图像与位姿采样镜像进着色线程的缓存；
//   ③ 着色线程：消费点云队列，配对图像 → 投影上色 → 累积彩色地图，按 ColorSegment
//      增量移交落盘线程（与 VIO 并行，互不阻塞）；
//   ④ 落盘线程：只做彩色地图增量段合并 + PCD 写盘，落盘不再阻塞计算。
//   工作线程 ──► OdomCallback；着色线程 ──► ColoredCloudCallback ──► ROS 发布
//
// 为什么这样拆线程：
//   * 实测 RK3588 上单 worker 串行跑 VIO（~130ms/帧）+ 着色（10Hz）吞吐饱和，
//     排队超时丢弃上万事件、着色帧覆盖率仅 30%。着色链路与 VIO 只共享"图像 +
//     位姿缓存"这类只读输入，天然可并行，拆到独立线程后两者互不拖累。
//   * 算法状态按线程归属严格拆分：VIO 状态（frontend_/pose_cache_/pending_* 等）
//     仍只被工作线程无锁访问；彩色状态（pending_clouds_/colored_map_/median_map_）
//     只被着色线程访问。跨线程交接只经由加锁的镜像缓存（color_*_cache_）与
//     原子计数器，不给 worker 的 pose_cache_ 加锁共用。
//   * 整图 PCD 重写从计算线程挪到独立落盘线程，计算与 IO 互不阻塞。
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

  // ---- 实时性控制 ----
  /// 视觉支路抽帧系数：1=每帧图像都跑前端/深度/更新，2=隔一帧（10 Hz -> 5 Hz）。
  /// 只影响视觉定位支路；图像仍全部进入缓存供彩色点云使用。
  int visual_decimate{1};
  /// 输入事件排队时延上限（秒）：超过就丢弃该事件并计数，避免单线程 worker
  /// 在计算跟不上时把队列越积越长（实测会出现端到端 30 s 级的排队延迟）。
  /// <=0 表示关闭该保护。
  double max_queue_latency{0.5};

  // ---- 缓存规模 ----
  std::size_t max_image_cache{30};
  std::size_t max_pose_samples{300};
  std::size_t max_map_points{60000};
  int map_append_stride{3};
  /// 输出点云上限：0=无限（默认）。voxmap 上限是 Super-LIO 自己的参数，与此无关。
  std::size_t max_colored_map_points{0};

  // ---- 彩色地图落盘 ----
  std::string pcd_save_dir{"/home/ywj/slam_data/pcd"};
  /// 时间中位数体素边长（米），<=0 关闭。
  float median_voxel_size{0.01F};
  /// 分片落盘：每 N 个彩色帧异步写一个增量 PCD（0 = 关闭，只在退出时写整图）。
  /// 参照 Super-LIO 的 lio.map.save_interval：每片只含自上一片以来的新增点，
  /// 由落盘线程写出、不阻塞工作线程；分片天然按时间有序，便于离线做时序滤波。
  std::size_t pcd_save_interval_frames{0};
  /// 按时间分片落盘（秒）：与 pcd_save_interval_frames 二选一/可叠加，任一满足
  /// 即切一片。参照 Super-LIO 的 lio.map.save_interval（秒级增量落盘）。
  double pcd_shard_interval_sec{0.0};
  /// 未染色点云落盘：把进入着色链路但没拿到颜色的点（视场外、遮挡、等图/等位
  /// 姿超时帧）按同一分片节奏写 uncolored_%05zu.pcd（灰色），退出时写
  /// super_livo_visual_a_uncolored.pcd。与彩色分片并集≈全部进入管线的点云，
  /// 便于离线拼接查看完整几何。
  bool save_uncolored{true};
  /// 每个彩色分片同时落盘移交时刻的最新去畸变帧：frame_%05zu.jpg（与
  /// colored_%05zu.pcd 同编号），便于核对色温/曝光/视场覆盖。
  bool save_frame_jpg{true};

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

  // ---- 延时计时（口径仿 Super-LIO lio.eva.timer，见 LatencyTimer）----
  /// 开启后按 60 s 窗口打印视觉支路端到端墙钟延时：
  /// [Img->VisOdom] 图像入队 → 修正 odom 发布；[Img->VisCloud] 图像入队 → 彩色云发布。
  bool timing_en{false};
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

  /// 运行时更新图像-雷达时钟偏移（t_lidar = t_img + offset），线程安全。
  /// 雷达/相机各自使用独立时钟，偏移会随运行漂移（实测约 1.3 ms/s），
  /// 因此启动后应用实测值覆盖参数初值，避免长跑后配对失败。
  void setImageTimeOffset(double offset);
  double imageTimeOffset() const;

  // ---- 输入（线程安全，入队后立即返回；BGR 图像内部完成去畸变+灰度） ----
  /// keepalive：允许调用方传入浅拷贝 Mat（如直接引用 ROS 消息内存）时保活
  /// 底层缓冲；事件消费完即释放。深拷贝可不传（与方案 B 同口径）。
  void submitImage(const cv::Mat& bgr, double stamp,
                   std::shared_ptr<const void> keepalive = nullptr);
  /// T_wl = LIO 位姿（原 lio/odom；child_frame_id 标 imu 但实为 LiDAR 位姿）。
  void submitOdom(const Eigen::Matrix4d& T_wl, double stamp);
  /// 独立的相机位姿源（原 color.cam_odom_topic）；不配置时与 submitOdom 同源。
  void submitCamOdom(const Eigen::Matrix4d& T_wl, double stamp);
  /// 世界系点云（原 lio/cloud_world）：当前扫描 + 局部地图（供深度关联）。
  void submitWorldCloud(const std::vector<Eigen::Vector3d>& points_world);
  /// 机体系全量点云（原 lio/cloud_body_dense）：仅用于彩色点云。
  /// 直接进着色线程的 color_queue_（带排队时延保护），不占 worker 输入队列。
  void submitBodyCloud(const std::vector<Eigen::Vector3f>& points_lidar, double stamp);
  /// 退化签名（原 lio/degeneracy；单体内由 SuperLIO 快照直供）。
  void submitDegeneracy(const superlivo_common::DegeneracySignature& sig);

  // ---- 控制 ----
  /// 请求一次彩色地图落盘（投 kSaveSnapshot 事件到着色队列：着色线程切增量段
  /// 移交落盘线程写盘，不阻塞调用方）。
  void requestSaveColoredMap();
  /// 等待已入队事件全部处理完（先 worker 后着色线程各投一个 barrier 事件，
  /// 30 s 超时打 WARN；用于退出前补完尾段）。
  void flush();
  /// 停机并**同步**写出最终彩色地图（幂等，可重复调用，std::call_once）。
  /// 退出前必须调用：异步 save 线程会随进程结束被直接杀掉，长序列上
  /// 大图还没写完进程就退出了，产物会整个丢失。
  void shutdownAndSave();
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
  /// 着色线程每轮处理前对镜像缓存取的一致性快照：持锁短暂拷贝后整轮无锁使用，
  /// 避免在 colorizeFrame/accumulateColoredMap 的重计算期间阻塞工作线程推送。
  struct ColorSnapshot {
    std::vector<colorizer::TimedImage> images;
    std::vector<colorizer::PoseSample> poses;      ///< 镜像自 worker pose_cache_
    std::vector<colorizer::PoseSample> cam_poses;  ///< 镜像自 worker cam_pose_cache_
    std::vector<colorizer::PoseSample> own_poses;  ///< 镜像自 worker own_pose_cache_
  };

  // ---- 输入处理（仅工作线程执行） ----
  void handleImage(const cv::Mat& bgr, double stamp);
  void handleOdom(const Eigen::Matrix4d& T_wl, double stamp);
  void handleCamOdom(const Eigen::Matrix4d& T_wl, double stamp);
  void handleWorldCloud(const std::vector<Eigen::Vector3d>& points_world);
  void handleDegeneracy(const superlivo_common::DegeneracySignature& sig);

  // ---- 主流程 ----
  void processVisualFrame(const cv::Mat& gray, double stamp);

  // ---- 着色链路（仅着色线程执行；shutdown join 后由收尾线程调用 drain*） ----
  /// 一帧点云入待配对队列（含非有限点过滤与重复帧去重）。
  void handleBodyCloudColor(const std::vector<Eigen::Vector3f>& points_lidar, double stamp);
  void drainPendingClouds(const ColorSnapshot& snap);
  void drainPendingColorization(const ColorSnapshot& snap);
  bool colorizeFrame(const PendingColorFrame& frame, const ColorSnapshot& snap);

  // ---- 位姿查询（工作线程：VIO 用 worker 私有缓存；着色线程：查快照） ----
  /// 通用版：在给定采样缓存上插值/clamp。
  bool poseAtFromCache(const std::vector<colorizer::PoseSample>& cache, double stamp,
                       Eigen::Matrix4d& T_wl, bool clamp_future) const;
  /// clamp_future：视觉定位路径专用。相机与雷达硬件级触发同步，按图像时刻查询时
  /// 最新采样通常只早几毫秒，直接取最新采样而不是等下一帧 odom。
  bool poseAtStamp(double stamp, Eigen::Matrix4d& T_wl, bool clamp_future = false) const;
  bool poseAtStampCorrected(double stamp, Eigen::Matrix4d& T_wl,
                            bool clamp_future = false) const;
  bool camPoseAtStamp(double stamp, Eigen::Matrix4d& T_wl) const;
  /// 着色线程版本：语义与 poseAtStamp/camPoseAtStamp 完全一致，只是查快照。
  bool poseAtStampColor(const ColorSnapshot& snap, double stamp, Eigen::Matrix4d& T_wl,
                        bool clamp_future = false) const;
  bool camPoseAtStampColor(const ColorSnapshot& snap, double stamp,
                           Eigen::Matrix4d& T_wl) const;

  // ---- 辅助 ----
  cv::Mat rectifyBgr(const cv::Mat& bgr) const;
  Eigen::Matrix4d makeTlcFromRclPcl(const std::vector<double>& rcl,
                                    const std::vector<double>& pcl) const;
  Eigen::Matrix4d applyLeftPerturbation(const Eigen::Matrix4d& T,
                                        const Eigen::VectorXd& dx) const;
  bool degeneracySignature(double& severity, Eigen::Vector3d& dir_w) const;
  void decayPoseOffset(double dt, double severity, const Eigen::Vector3d& dir_w);
  /// 累积彩色地图（着色线程；位姿取快照的镜像 pose 缓存）。
  void accumulateColoredMap(const std::vector<colorizer::ColoredPoint>& points, double stamp,
                            const ColorSnapshot& snap);
  /// 累积未染色点（着色线程）：机体系点用 stamp 时刻位姿变换到世界系后追加，
  /// 灰色 (128,128,128)。位姿插值失败或达到点数上限则跳过。
  void appendUncolored(const std::vector<Eigen::Vector3f>& points_lidar, double stamp,
                       const ColorSnapshot& snap);
  void saveColoredMap();
  void saveUncoloredMap();
  void savePose(const Eigen::Matrix4d& T_wl, double stamp);
  void saveImage(const cv::Mat& bgr, double stamp);
  void publishColoredCloud(const std::vector<colorizer::ColoredPoint>& points, double stamp);
  void publishOdomAndPath(const Eigen::Matrix4d& T_wl, double stamp);

  void log(LogLevel level, const std::string& msg) const;
  void logThrottled(const std::string& key, int period_ms, LogLevel level,
                    const std::string& msg) const;
  static double steadyNowSec();

  // ==========================================================================
  // 线程化设施（与方案 B 的 VisualPipeline 同构）
  // ==========================================================================
  struct InputEvent {
    enum class Kind : std::uint8_t {
      kImage,
      kOdom,
      kCamOdom,
      kCloud,
      kDegeneracy,
      kBarrier,
      kQuit
    };
    Kind kind{Kind::kQuit};
    cv::Mat image;                            ///< BGR（可能是引用消息内存的浅拷贝）
    std::shared_ptr<const void> image_keepalive;  ///< 浅拷贝时保活底层缓冲（如 ROS 消息）
    double stamp{0.0};
    Eigen::Matrix4d T{Eigen::Matrix4d::Identity()};
    std::vector<Eigen::Vector3d> points_world;
    superlivo_common::DegeneracySignature degen;
    std::shared_ptr<std::promise<void>> barrier;
    std::chrono::steady_clock::time_point enqueued{};
  };

  /// 着色线程事件：点云直接进 color_queue_，落盘快照/barrier 也走这条队列，
  /// 保证「快照切段」发生在其之前的所有点云着色完成之后。
  struct ColorEvent {
    enum class Kind : std::uint8_t { kBodyCloud, kSaveSnapshot, kBarrier, kQuit };
    Kind kind{Kind::kQuit};
    std::vector<Eigen::Vector3f> points_body;
    double stamp{0.0};
    std::shared_ptr<std::promise<void>> barrier;
    std::chrono::steady_clock::time_point enqueued{};
  };

  void startWorkerThreads();
  void stopWorkerThreads(bool keep_save_thread = false);
  /// 停止着色线程并等它排空 color_queue_（幂等）。
  void stopColorThread();
  /// 停止落盘线程并等它写完队列里剩下的段（幂等）。
  void stopSaveThread();
  bool pushInputEvent(InputEvent event);
  bool pushColorEvent(ColorEvent event);
  void workerLoop();
  void colorLoop();
  void saveLoop();
  /// 对镜像缓存取一致性快照（着色线程每轮开头调用）。
  ColorSnapshot takeColorSnapshot() const;
  // ---- 工作线程 → 着色线程镜像推送（短临界区 + 唤醒） ----
  void pushColorImage(const cv::Mat& rectified, double stamp);
  void pushColorPose(const colorizer::PoseSample& sample);
  void pushColorCamPose(const colorizer::PoseSample& sample);
  void pushColorOwnPose(const colorizer::PoseSample& sample);
  /// 回灌模式下 processVisualFrame 会改写 own_pose_cache_ 尾采样，同步镜像。
  void updateColorOwnPoseBack(const Eigen::Matrix4d& T_wl, double stamp);
  /// 每 60 s 打印一次延时窗口（timing_en 开启且有记录时），工作线程调用
  void maybePrintLatencyWindow();

  struct ColorSegment {
    pcl::PointCloud<pcl::PointXYZRGB> points;
    std::shared_ptr<colorizer::VoxelColorMedian> median;
    /// 分片模式下颜色已在移交时刻用实时 median_map_ 回填完毕（与"拷贝快照后
    /// 回填"同一时点、同一 colorAt，结果逐点一致），落盘线程不再二次回填。
    bool color_resolved{false};
    /// 未染色段：写 uncolored_%05zu.pcd，不做中位数回填。
    bool is_uncolored{false};
    /// 移交时刻的最新去畸变帧（引用计数浅拷贝，仅彩色段携带）。
    cv::Mat frame;
  };

  void requestColoredMapSave();
  void requestUncoloredMapSave();
  void writeColoredSnapshot(std::deque<ColorSegment> segments);

  std::deque<InputEvent> input_queue_;
  std::mutex input_mutex_;
  std::condition_variable input_cv_;
  bool input_stop_{false};
  std::atomic<std::size_t> stat_input_dropped_{0};
  std::thread worker_;

  // ---- 着色线程：独立事件队列 + 镜像缓存 ----
  std::deque<ColorEvent> color_queue_;
  std::mutex color_mutex_;
  std::condition_variable color_cv_;
  bool color_stop_{false};
  /// worker 推入新图像/位姿后置真并 notify，唤醒着色线程重试待配对帧。
  std::atomic<bool> color_wake_{false};
  std::thread color_thread_;
  /// 保护 color_*_cache_（worker 写、着色线程读快照）。
  mutable std::mutex color_cache_mutex_;
  std::vector<colorizer::TimedImage> color_image_cache_;
  std::vector<colorizer::PoseSample> color_pose_cache_;
  std::vector<colorizer::PoseSample> color_cam_pose_cache_;
  std::vector<colorizer::PoseSample> color_own_pose_cache_;

  // 延时计时：img_enqueue_wall_ 由 worker 写、着色线程读（原子交接）；
  // last_timing_report_ 仅工作线程读写；
  // latency_timer_ 内部加锁（退出总报告可能来自 shutdown 线程）。
  superlivo_common::LatencyTimer latency_timer_;
  std::atomic<std::chrono::steady_clock::time_point> img_enqueue_wall_{};
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
  /// 分片落盘（pcd_save_interval_frames > 0）：每片一个 colored_%05zu.pcd。
  std::size_t save_interval_frames_{0};
  double shard_interval_sec_{0.0};
  bool shard_mode_{false};  ///< 帧数或秒数任一开启即进入分片模式
  std::chrono::steady_clock::time_point last_shard_time_{};
  std::size_t shard_index_{0};
  std::size_t frames_since_shard_{0};
  // ---- 未染色点云（着色线程独占，与 colored_map_ 同生命周期）----
  bool save_uncolored_{true};
  bool save_frame_{true};
  cv::Mat latest_color_frame_;  ///< 着色线程独占：每轮快照更新，移交时挂到段上
  pcl::PointCloud<pcl::PointXYZRGB> uncolored_map_;
  std::size_t uncolored_handed_points_{0};
  std::size_t uncolored_shard_index_{0};
  std::chrono::steady_clock::time_point last_uncolored_shard_time_{};
  bool uncolored_capacity_warned_{false};
  /// shutdownAndSave() 只执行一次；并发调用者（shutdown 钩子 / 主线程 /
  /// 析构）会阻塞到那次执行完成，保证进程退出前文件已落盘。
  std::once_flag shutdown_once_;

  // ---- 输出回调 ----
  mutable std::mutex cb_mutex_;
  ColoredCloudCallback colored_cb_;
  OdomCallback odom_cb_;
  LogCallback log_cb_;

  // ---- 日志节流（工作线程与落盘线程都会写，需加锁） ----
  mutable std::mutex log_last_mutex_;
  mutable std::map<std::string, std::chrono::steady_clock::time_point> log_last_;

  // ---- 配置与算法模块（构造后不变） ----
  VisualPipelineOptions opt_;
  /// 视觉抽帧系数（>=1）与输入排队时延上限（秒，<=0 关闭）。
  int visual_decimate_{1};
  double max_queue_latency_{0.5};
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
  std::atomic<double> img_time_offset_{0.0};
  mutable bool rectify_maps_ready_{false};
  mutable cv::Mat rectify_map1_;
  mutable cv::Mat rectify_map2_;

  // ---- 数据缓存（仅工作线程访问；彩色所需副本经 color_*_cache_ 镜像交接） ----
  std::vector<colorizer::PoseSample> pose_cache_;
  std::vector<colorizer::PoseSample> cam_pose_cache_;
  std::vector<Eigen::Vector3d> local_map_;
  std::vector<Eigen::Vector3d> sweep_;

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

  // ---- 彩色地图（着色线程独占；shutdown join 后由收尾线程补完尾段） ----
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

  // ---- 统计（worker / 着色线程 / statsString 读取线程三方触碰，全部原子化） ----
  std::atomic<std::size_t> stat_colorizer_calls_{0};
  std::atomic<std::size_t> stat_colored_frames_{0};
  std::atomic<std::size_t> stat_drop_no_image_{0};
  std::atomic<std::size_t> stat_drop_pose_{0};
  std::atomic<std::size_t> stat_deferred_{0};
  std::atomic<std::size_t> stat_skip_duplicate_{0};
  std::atomic<std::size_t> stat_drop_accum_{0};
  /// 视觉抽帧跳过的图像数（visual_decimate > 1 时）。
  std::atomic<std::size_t> stat_visual_skipped_{0};
  /// 延时保护丢弃的输入事件数（排队超过 max_queue_latency；worker 与着色线程共用）。
  std::atomic<std::size_t> stat_drop_stale_{0};
  /// 已入队的图像计数（抽帧用）。
  std::atomic<std::size_t> img_seen_{0};
};

}  // namespace superlivo_a
