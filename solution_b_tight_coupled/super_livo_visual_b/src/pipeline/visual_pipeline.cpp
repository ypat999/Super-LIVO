#include "super_livo_visual_b/pipeline/visual_pipeline.hpp"

#include <cstdarg>
#include <cstdio>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>

#include <opencv2/calib3d.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "super_livo_common/anisotropic_prior.hpp"
#include <Eigen/LU>

namespace superlivo_b {

namespace {
/// 位姿历史缓冲允许的最大相邻采样间隔（秒）。
/// 与 color.sync_max_offset（图像-点云同步窗口）是两件事，不能混用。
/// pose_cache_ 来自 odom_topic（默认 lio/odom，10 Hz 激光帧后验）；0.5 s 已能容忍
/// 连续 5 帧丢失，只用于拒绝真空洞。
constexpr double kPoseHistoryMaxGapSec = 0.5;

/// 待着色帧最长等待时间（秒）：t_img 比 t_cloud 晚 img_time_offset，等 1 s 还拿不到
/// 位姿说明确实没有后续 odom（例如 bag 结束）。
constexpr double kPendingColorTimeoutSec = 1.0;

/// 待着色队列上限，防止 odom 长时间缺失时内存膨胀。
constexpr std::size_t kMaxPendingColorFrames = 8;

/// 彩色排空最小间隔（秒）：位姿缓存只随 odom_topic（默认 lio/odom，10 Hz）推进，
/// 图像也是 10 Hz，drain 由 odom/云/图像回调共同触发，按最小间隔限流可避免无解重试
/// 把工作线程吃满（见 [PROBE-D]）。
constexpr double kColorDrainMinIntervalSec = 0.1;

/// 待处理视觉帧：odom_topic（默认 lio/odom）要等下一帧激光位姿才覆盖图像时刻
/// （实测取用时差 avg≈0.32–0.35 s），留 1 s 余量后丢弃。
constexpr double kPendingVisualTimeoutSec = 1.0;
constexpr std::size_t kMaxPendingVisualFrames = 8;

/// 等图像队列：上限与超时（秒）。
constexpr std::size_t kMaxPendingClouds = 6;
constexpr double kPendingCloudTimeoutSec = 1.0;

/// 标准重力幅值（m/s²），用于把加计归一化到 m/s²（对齐官方 G_m_s2）。
constexpr double kGravityNorm = 9.81;

/// 单调时钟秒（退化签名的到达时刻；与 ROS 时间无关）。
double steadyNowSec() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

/// W3 视觉本征噪声（相机系，rad²）：§36.12 的实测协方差（350–500 s 段，跨段复用）。
/// 主轴 σ = [0.0235, 0.0278, 0.0567]°，floor 0.02°；与 §36.14 离线回放的
/// Cw_body = A_cam·R_CL 逐项同源，保证接线后可与离线 0.0540° 对齐。
Eigen::Matrix3d visualNoiseW3() {
  Eigen::Matrix3d S;
  S << 2.230418678636757e-07, -1.510679266130551e-07, 6.753803354649112e-08,
      -1.510679266130551e-07, 9.07086570961247e-07, -1.6356947135164733e-07,
      6.753803354649112e-08, -1.6356947135164733e-07, 2.5483994908395086e-07;
  return S;
}

/// 在给定（按 stamp 升序）位姿缓存上按时刻插值；查询时刻晚于最新采样时返回
/// false（拿最后一个采样顶替等于把不同时刻的位姿当成同一时刻）。
///
/// 这里直接在缓存上二分 + 自行插值，不再把整个缓存重建成 LidarPoseInterpolator：
///   * 插值器内部只保留最近 200 条采样，而缓存有 2000 条（10 Hz ≈ 200 s），
///     逐条 addSample 后前面 1800 条会被立刻裁掉 —— 既白做 O(N) 工作，又让
///     落后较多的查询必然查不到位姿；
///   * 彩色排空路径上每次对积压的每一帧重试，单次约 1 ms，会把工作线程直接吃满
///     （见 [PROBE-D]）。
bool poseFromSamples(const std::vector<colorizer::PoseSample>& cache, double stamp,
                     Eigen::Matrix4d& T_wl) {
  if (cache.size() < 2) {
    return false;
  }
  if (stamp > cache.back().stamp) {
    return false;
  }

  const auto it = std::lower_bound(
      cache.begin(), cache.end(), stamp,
      [](const colorizer::PoseSample& s, double value) { return s.stamp < value; });

  if (it == cache.begin()) {
    // 查询时刻早于最早采样：时间差足够小时直接用首个采样。
    if (it->stamp - stamp > kPoseHistoryMaxGapSec) {
      return false;
    }
    T_wl = it->T_wl;
    return true;
  }
  if (it == cache.end()) {
    const auto& last = cache.back();
    if (stamp - last.stamp > kPoseHistoryMaxGapSec) {
      return false;
    }
    T_wl = last.T_wl;
    return true;
  }

  const auto& s1 = *it;
  const auto& s0 = *(it - 1);
  const double dt_total = s1.stamp - s0.stamp;
  if (dt_total <= 0.0 || dt_total > kPoseHistoryMaxGapSec) {
    return false;
  }
  const double alpha = (stamp - s0.stamp) / dt_total;

  const Eigen::Vector3d t = (1.0 - alpha) * s0.T_wl.block<3, 1>(0, 3) +
                            alpha * s1.T_wl.block<3, 1>(0, 3);
  Eigen::Quaterniond q0(s0.T_wl.block<3, 3>(0, 0));
  Eigen::Quaterniond q1(s1.T_wl.block<3, 3>(0, 0));
  q0.normalize();
  q1.normalize();
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() = -q1.coeffs();
  }
  const Eigen::Quaterniond q = q0.slerp(alpha, q1);

  T_wl = Eigen::Matrix4d::Identity();
  T_wl.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T_wl.block<3, 1>(0, 3) = t;
  return true;
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

// ===========================================================================
// 临时探针（只为回答"B 为什么只出 52 帧"，不参与任何估计，可整块删除）。
// 单线程 executor ⇒ 全部用文件级静态变量，无需加锁。
// 计三类证据：
//   ① 订阅侧：每个 topic 的到达次数与相邻 header 间隔（帧到底有没有到）
//   ② 处理侧：processVisualFrame 次数、相邻处理间隔、各分段耗时的窗口均值/峰值
//      （现有 "耗时 ms" 行是 INFO_THROTTLE(1000)，一秒只打印最后一帧，不能用来算总量）
//   ③ 丢弃侧：每条丢弃/延迟路径分开计数，外加周期落盘对 executor 的阻塞时长
// ===========================================================================
struct ProbeAcc {
  double sum{0.0};
  double max{0.0};
  std::size_t n{0};
  void add(double v) {
    sum += v;
    max = std::max(max, v);
    ++n;
  }
  double avg() const { return n > 0 ? sum / static_cast<double>(n) : 0.0; }
  void reset() { *this = ProbeAcc{}; }
};

struct BranchProbe {
  // ① 订阅侧
  std::size_t n_image{0}, n_cloud{0}, n_body{0}, n_odom{0}, n_imu{0};
  ProbeAcc image_gap, cloud_gap, body_gap, odom_gap, imu_gap;  ///< wall 到达间隔 ms
  ProbeAcc image_hgap, odom_hgap, body_hgap;  ///< header 相邻增量 ms（找时间域空洞）
  double first_image{-1.0}, last_image{0.0};
  double first_odom{-1.0}, last_odom{0.0};
  double first_body{-1.0}, last_body{0.0};
  std::chrono::steady_clock::time_point t_image{}, t_cloud{}, t_body{}, t_odom{}, t_imu{};

  // ② 处理侧
  std::size_t n_process{0};
  std::chrono::steady_clock::time_point t_process{};
  ProbeAcc process_gap;   ///< 相邻两次 processVisualFrame 的 wall 间隔 ms
  ProbeAcc wait_visual;   ///< 视觉帧在 pending_visual_ 中等待 t_pose 的时长 ms
  ProbeAcc ms_front, ms_constraint, ms_update, ms_color, ms_total;
  ProbeAcc ms_copy_map, ms_project, ms_normal_eq, ms_select, ms_mscfk;
  ProbeAcc ms_colorize, ms_accum_map, ms_pub_cloud;
  ProbeAcc ms_imu_prop, ms_prior, ms_vis_upd, ms_map_commit, ms_pub;
  ProbeAcc acc_obs, acc_tracks, acc_lv, acc_mscfk_used, acc_used, acc_map_pts;

  // ③ 丢弃 / 延迟侧（每条路径独立计数，累计值不清零）
  std::size_t drop_pose_hole{0};       ///< 位姿空洞，processVisualFrame 直接返回
  std::size_t drop_pose_timeout{0};    ///< 视觉帧等位姿超时
  std::size_t drop_visual_full{0};     ///< 待定视觉队列超限丢弃
  std::size_t drop_no_image{0};        ///< 云等图像超时
  std::size_t drop_cloud_full{0};      ///< 等图像队列溢出
  std::size_t drop_color_timeout{0};   ///< 着色等位姿超时
  std::size_t drop_color_full{0};      ///< 待着色队列溢出
  std::size_t color_defer{0};          ///< 本帧位姿未到，留到下次重试
  std::size_t color_ok{0};             ///< 成功着色帧
  std::size_t queue_push{0};           ///< runColorizer 入队次数
  std::size_t n_save{0};
  ProbeAcc ms_save;                    ///< 周期落盘阻塞时长（单线程 executor 期间全丢）
  ProbeAcc ms_colored_pts;             ///< 每帧上色点数

  std::chrono::steady_clock::time_point t_report{};
  std::chrono::steady_clock::time_point t0{};
  double t_last_report{-1.0};
  // 窗口增量用：上次上报时的累计计数。
  std::size_t prev_image{0}, prev_cloud{0}, prev_body{0}, prev_odom{0}, prev_imu{0};
  std::size_t prev_process{0}, prev_color_ok{0}, prev_queue_push{0}, prev_save{0};

  // ④ 阻塞定位：输入队列排队延迟与深度（窗口内统计）
  ProbeAcc q_wait_image, q_wait_odom, q_wait_cloud, q_wait_body, q_wait_imu;
  ProbeAcc queue_depth;      ///< 每次出队后队列剩余深度
  ProbeAcc ms_handler;       ///< 工作线程单事件处理时长（分派+算法）
  std::size_t n_dequeue{0}, prev_dequeue{0};

  // ⑤ 阻塞定位：彩色排空路径。drainPendingColorization() 挂在 odom_topic（默认
  // lio/odom，10 Hz）以及云/图像回调上，每次对 pending_color_ 里每一帧重试一次
  // colorizeFrame()；失败重试本身要重建位姿插值器，因此"重试次数 × 单次耗时"
  // 可能整段吃掉工作线程。
  ProbeAcc ms_drain_color;   ///< 单次 drainPendingColorization() 时长
  std::size_t n_retry_color{0}, prev_retry_color{0};  ///< colorizeFrame 调用次数
  std::size_t n_pose_fail{0};                         ///< 其中位姿查询失败次数
};

BranchProbe g_probe;

/// 相邻消息到达间隔（首次调用只记时刻）。
void probeGap(ProbeAcc& acc, std::chrono::steady_clock::time_point& last,
              const std::chrono::steady_clock::time_point& now) {
  if (last.time_since_epoch().count() != 0) {
    acc.add(std::chrono::duration<double, std::milli>(now - last).count());
  }
  last = now;
}

/// 从给定时刻到现在的毫秒数（探针专用）。
double probeMs(const std::chrono::steady_clock::time_point& from) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - from).count();
}

/// header 跨度（秒）转 ms 记录，用于发现时间域空洞。
void probeHeaderSpan(ProbeAcc& acc, double& first, double& last, double stamp) {
  if (first < 0.0) {
    first = stamp;
  } else if (stamp > last) {
    acc.add((stamp - last) * 1000.0);
  }
  last = std::max(last, stamp);
}
}  // namespace

VisualPipeline::VisualPipeline(ParamSource& params,
                               std::shared_ptr<ConstraintBridge> bridge)
    : params_(params), bridge_(std::move(bridge)) {
  loadParameters();
  startWorkerThreads();
  // 启动自检：确认参数确实注入（分支式配置最容易静默失效）。
  logf(LogLevel::kInfo,
       "VisualPipeline 初始化完成：相机 %dx%d fx=%.2f fy=%.2f cx=%.2f cy=%.2f "
       "img_time_offset=%.3f pcd=%s",
       intrinsics_.width, intrinsics_.height, intrinsics_.fx, intrinsics_.fy,
       intrinsics_.cx, intrinsics_.cy, img_time_offset_, pcd_save_dir_.c_str());
}

// ===========================================================================
// 对外接口：输入 -> 队列，输出 -> 回调
// ===========================================================================

void VisualPipeline::setColoredCloudCallback(ColoredCloudCallback cb) {
  colored_cb_ = std::move(cb);
}

void VisualPipeline::setOdomCallback(OdomCallback cb) {
  odom_cb_ = std::move(cb);
}

void VisualPipeline::setRotRefinedOdomCallback(OdomCallback cb) {
  rot_odom_cb_ = std::move(cb);
}

void VisualPipeline::setLogCallback(LogCallback cb) {
  log_cb_ = std::move(cb);
}

void VisualPipeline::submitImage(const cv::Mat& bgr, double stamp,
                                 std::shared_ptr<const void> keepalive) {
  InputEvent ev;
  ev.kind = InputEvent::Kind::kImage;
  ev.image = bgr;
  ev.image_keepalive = std::move(keepalive);
  ev.stamp = stamp;
  pushInputEvent(std::move(ev));
}

void VisualPipeline::submitOdom(const Eigen::Matrix4d& T_w, double stamp) {
  InputEvent ev;
  ev.kind = InputEvent::Kind::kOdom;
  ev.T = T_w;
  ev.stamp = stamp;
  pushInputEvent(std::move(ev));
}

void VisualPipeline::submitCamOdom(const Eigen::Matrix4d& T_w, double stamp) {
  InputEvent ev;
  ev.kind = InputEvent::Kind::kCamOdom;
  ev.T = T_w;
  ev.stamp = stamp;
  pushInputEvent(std::move(ev));
}

void VisualPipeline::submitWorldCloud(const std::vector<Eigen::Vector3d>& points_world) {
  InputEvent ev;
  ev.kind = InputEvent::Kind::kCloud;
  ev.points_world = points_world;
  pushInputEvent(std::move(ev));
}

void VisualPipeline::submitBodyCloud(const std::vector<Eigen::Vector3f>& points_lidar,
                                     double stamp) {
  InputEvent ev;
  ev.kind = InputEvent::Kind::kBodyCloud;
  ev.points_body = points_lidar;
  ev.stamp = stamp;
  pushInputEvent(std::move(ev));
}

void VisualPipeline::submitImu(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,
                               double stamp) {
  InputEvent ev;
  ev.kind = InputEvent::Kind::kImu;
  ev.acc = acc;
  ev.gyro = gyro;
  ev.stamp = stamp;
  pushInputEvent(std::move(ev));
}

void VisualPipeline::submitDegeneracy(
    const superlivo_common::DegeneracySignature& sig) {
  InputEvent ev;
  ev.kind = InputEvent::Kind::kDegeneracy;
  ev.degen = sig;
  pushInputEvent(std::move(ev));
}

void VisualPipeline::requestSaveColoredMap() {
  InputEvent ev;
  ev.kind = InputEvent::Kind::kSaveSnapshot;
  pushInputEvent(std::move(ev));
}

void VisualPipeline::flush() {
  auto promise = std::make_shared<std::promise<void>>();
  auto future = promise->get_future();
  InputEvent ev;
  ev.kind = InputEvent::Kind::kBarrier;
  ev.barrier = promise;
  // 停机后 pushInputEvent 会拒收：此时不再等待（否则会白等 30 s）。
  if (!pushInputEvent(std::move(ev))) {
    return;
  }
  if (future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
    logf(LogLevel::kWarn, "flush 超时：工作线程可能已停止");
  }
}

std::string VisualPipeline::statsString() const {
  char buf[512];
  std::snprintf(
      buf, sizeof(buf),
      "彩色帧=%zu 丢(无图)=%zu 丢(无位姿)=%zu 延后=%zu 输入丢弃=%zu "
      "地图点=%zu 姿态帧=%zu 可用=%zu",
      stat_colored_frames_, stat_drop_no_image_, stat_drop_pose_,
      stat_deferred_, stat_input_dropped_.load(std::memory_order_relaxed),
      map_points_.size(), stat_rot_frames_, stat_rot_usable_);
  return std::string(buf);
}

// ---- 日志 ----
void VisualPipeline::logf(LogLevel level, const char* fmt, ...) const {
  char buf[2048];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (log_cb_) {
    log_cb_(level, std::string(buf));
  }
}

void VisualPipeline::logfThrottled(LogLevel level, int period_ms, const char* fmt, ...) const {
  if (period_ms > 0) {
    // 以格式串地址作为调用点标识（与 RCLCPP_*_THROTTLE 的按调用点节流等价）。
    static std::mutex throttle_mutex;
    static std::unordered_map<std::string, std::chrono::steady_clock::time_point> last;
    const std::string key = fmt;
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(throttle_mutex);
    auto it = last.find(key);
    if (it != last.end() &&
        std::chrono::duration<double, std::milli>(now - it->second).count() <
            static_cast<double>(period_ms)) {
      return;
    }
    last[key] = now;
  }
  char buf[2048];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (log_cb_) {
    log_cb_(level, std::string(buf));
  }
}

// ===========================================================================
// 线程化设施（分工与理由见 hpp "线程化：IO 与计算互不阻塞"段）
// ===========================================================================

void VisualPipeline::startWorkerThreads() {
  save_thread_ = std::thread(&VisualPipeline::saveLoop, this);
  worker_ = std::thread(&VisualPipeline::workerLoop, this);
}

void VisualPipeline::stopWorkerThreads(bool keep_save_thread) {
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (input_stop_) {
      return;
    }
    input_stop_ = true;
  }
  input_cv_.notify_one();
  if (worker_.joinable()) {
    worker_.join();  // 先排空输入队列，剩余事件照常处理完（不丢尾段）
  }
  if (!keep_save_thread) {
    stopSaveThread();
  }
}

void VisualPipeline::stopSaveThread() {
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    save_stop_ = true;
  }
  save_cv_.notify_one();
  if (save_thread_.joinable()) {
    save_thread_.join();
  }
}

bool VisualPipeline::pushInputEvent(InputEvent event) {
  event.enqueued = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (input_stop_) {
      return false;  // 停机后不再接收，避免向已退出的 worker 投递
    }
    // 上限兜底：正常情况 worker 单帧约 30 ms、输入约 213 条/秒，不会积压。
    // 真积压说明 CPU 被抢或算法退化，此时丢最旧事件以守住内存上界
    // （一帧原图 4 MB，不设限会直接吃掉几十 GB）。
    constexpr std::size_t kMaxInputEvents = 2048;
    while (input_queue_.size() >= kMaxInputEvents) {
      input_queue_.pop_front();
      stat_input_dropped_.fetch_add(1, std::memory_order_relaxed);
    }
    input_queue_.push_back(std::move(event));
  }
  input_cv_.notify_one();
  return true;
}

void VisualPipeline::workerLoop() {
  for (;;) {
    InputEvent ev;
    {
      std::unique_lock<std::mutex> lock(input_mutex_);
      input_cv_.wait(lock, [this] { return input_stop_ || !input_queue_.empty(); });
      if (input_queue_.empty()) {
        if (input_stop_) {
          break;
        }
        continue;
      }
      ev = std::move(input_queue_.front());
      input_queue_.pop_front();
      // 探针：出队时刻的队列深度与"入队→出队"排队延迟，按事件类型分开统计。
      // 若某一类的延迟持续抬升，说明工作线程被该类型/更早的事件阻塞。
      {
        const auto now = std::chrono::steady_clock::now();
        g_probe.queue_depth.add(static_cast<double>(input_queue_.size()));
        const double waited_ms =
            std::chrono::duration<double, std::milli>(now - ev.enqueued).count();
        switch (ev.kind) {
          case InputEvent::Kind::kImage:
            g_probe.q_wait_image.add(waited_ms);
            break;
          case InputEvent::Kind::kOdom:
          case InputEvent::Kind::kCamOdom:
            g_probe.q_wait_odom.add(waited_ms);
            break;
          case InputEvent::Kind::kCloud:
            g_probe.q_wait_cloud.add(waited_ms);
            break;
          case InputEvent::Kind::kBodyCloud:
            g_probe.q_wait_body.add(waited_ms);
            break;
          case InputEvent::Kind::kImu:
            g_probe.q_wait_imu.add(waited_ms);
            break;
          default:
            break;
        }
        ++g_probe.n_dequeue;
      }
    }
    // 全部算法状态只在这条线程上访问，因此 onXxx 内部无需任何锁。
    const auto handler_begin = std::chrono::steady_clock::now();
    switch (ev.kind) {
      case InputEvent::Kind::kImage:
        img_enqueue_wall_ = ev.enqueued;  // 延时计时锚点：图像入队墙钟时刻
        onImage(ev.image, ev.stamp);
        break;
      case InputEvent::Kind::kOdom:
        onOdom(ev.T, ev.stamp);
        break;
      case InputEvent::Kind::kCamOdom:
        onCamOdom(ev.T, ev.stamp);
        break;
      case InputEvent::Kind::kCloud:
        onCloud(ev.points_world);
        break;
      case InputEvent::Kind::kBodyCloud:
        onBodyCloud(ev.points_body, ev.stamp);
        break;
      case InputEvent::Kind::kImu:
        onImu(ev.acc, ev.gyro, ev.stamp);
        break;
      case InputEvent::Kind::kDegeneracy:
        onDegeneracy(ev.degen);
        break;
      case InputEvent::Kind::kSaveSnapshot:
        requestColoredMapSave();
        break;
      case InputEvent::Kind::kBarrier:
        if (ev.barrier) {
          ev.barrier->set_value();
        }
        break;
      case InputEvent::Kind::kQuit:
        return;
    }
    // 工作线程占用时长（分派+处理），配合报告窗口可算线程占空比。
    g_probe.ms_handler.add(probeMs(handler_begin));
    maybePrintLatencyWindow();
  }
}

void VisualPipeline::maybePrintLatencyWindow() {
  if (!timing_en_) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (last_timing_report_.time_since_epoch().count() == 0) {
    last_timing_report_ = now;  // 首个事件确立窗口起点
    return;
  }
  if (now - last_timing_report_ < std::chrono::seconds(60) || latency_timer_.empty()) {
    return;
  }
  latency_timer_.PrintAndClear();
  last_timing_report_ = now;
}

void VisualPipeline::saveLoop() {
  for (;;) {
    std::deque<ColorSegment> segments;
    {
      std::unique_lock<std::mutex> lock(save_mutex_);
      save_cv_.wait(lock, [this] { return save_stop_ || save_pending_; });
      if (!save_pending_) {
        if (save_stop_) {
          break;
        }
        continue;
      }
      save_pending_ = false;
      segments.swap(save_segments_);  // 整队取走，工作线程随后照常往空队列追加
    }
    writeColoredSnapshot(std::move(segments));
  }
}

void VisualPipeline::requestColoredMapSave() {
  const std::size_t n = colored_map_.size();
  if (n <= save_handed_points_) {
    return;  // 无新增
  }
  ++g_probe.n_save;
  const auto t0 = std::chrono::steady_clock::now();
  // 只切"上次移交以来新增的那一段"：彩色地图是 append-only（accumulateColoredMap
  // 只有 push_back，达到上限后直接 return），所以按下标切段等价于整图快照，
  // 但工作线程的代价从 O(全图) 降到 O(15 s 新增)。
  ColorSegment seg;
  seg.points.assign(colored_map_.points.begin() +
                        static_cast<std::ptrdiff_t>(save_handed_points_),
                    colored_map_.points.end());
  seg.points.width = static_cast<std::uint32_t>(seg.points.size());
  seg.points.height = 1;
  seg.points.is_dense = false;
  if (median_map_.enabled()) {
    seg.median = std::make_shared<colorizer::VoxelColorMedian>(median_map_);
  }
  save_handed_points_ = n;
  g_probe.ms_save.add(probeMs(t0));
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    save_segments_.push_back(std::move(seg));
    save_pending_ = true;
  }
  save_cv_.notify_one();
  probeMaybeReport();
}

void VisualPipeline::writeColoredSnapshot(std::deque<ColorSegment> segments) {
  std::error_code ec;
  std::filesystem::create_directories(pcd_save_dir_, ec);
  if (ec) {
    logfThrottled(LogLevel::kWarn, 5000,
                         "无法创建 PCD 输出目录 %s: %s", pcd_save_dir_.c_str(),
                         ec.message().c_str());
    return;
  }
  const std::string path =
      (std::filesystem::path(pcd_save_dir_) / "super_livo_visual_b_colored.pcd").string();

  // ---- 分片模式（参照 Super-LIO scans_*.pcd）：一段写一个文件，纯增量 ----
  // 每片只含本时间窗新增点，落盘线程异步写；分片按时间有序，可直接做时序滤波。
  if (save_interval_frames_ > 0) {
    const auto c8 = [](double v) {
      return static_cast<std::uint8_t>(std::clamp(v, 0.0, 255.0));
    };
    for (auto& seg : segments) {
      if (seg.points.empty()) {
        continue;
      }
      if (seg.median) {
        save_median_ = seg.median;
      }
      pcl::PointCloud<pcl::PointXYZRGB> cloud = std::move(seg.points);
      if (save_median_ && save_median_->enabled()) {
        for (auto& p : cloud.points) {
          Eigen::Vector3d rgb;
          if (save_median_->colorAt(Eigen::Vector3f(p.x, p.y, p.z), rgb)) {
            p.r = c8(rgb.x());
            p.g = c8(rgb.y());
            p.b = c8(rgb.z());
          }
        }
      }
      cloud.width = static_cast<std::uint32_t>(cloud.size());
      cloud.height = 1;
      cloud.is_dense = false;
      char name[64];
      std::snprintf(name, sizeof(name), "colored_%05zu.pcd", shard_index_++);
      const std::string shard_path =
          (std::filesystem::path(pcd_save_dir_) / name).string();
      if (pcl::io::savePCDFileBinary(shard_path, cloud) == 0) {
        logf(LogLevel::kInfo, "彩色分片已保存: %s（%zu 点）", shard_path.c_str(),
             cloud.size());
      } else {
        logfThrottled(LogLevel::kWarn, 5000, "保存 PCD 分片失败: %s",
                      shard_path.c_str());
      }
    }
    return;
  }

  // 合并增量段到常驻缓冲。正常每 15 s 只有一段；落盘慢于移交时会有多段。
  for (auto& seg : segments) {
    save_buffer_.points.insert(save_buffer_.points.end(), seg.points.points.begin(),
                               seg.points.points.end());
    if (seg.median) {
      save_median_ = seg.median;
    }
  }
  if (save_buffer_.empty()) {
    return;
  }

  // 时间中位数：只回填颜色，几何/点数完全不动（避免体素化把点云变稀）。
  if (save_median_ && save_median_->enabled()) {
    const auto c8 = [](double v) {
      return static_cast<std::uint8_t>(std::clamp(v, 0.0, 255.0));
    };
    for (auto& p : save_buffer_.points) {
      Eigen::Vector3d rgb;
      if (save_median_->colorAt(Eigen::Vector3f(p.x, p.y, p.z), rgb)) {
        p.r = c8(rgb.x());
        p.g = c8(rgb.y());
        p.b = c8(rgb.z());
      }
    }
  }
  save_buffer_.width = static_cast<std::uint32_t>(save_buffer_.size());
  save_buffer_.height = 1;
  save_buffer_.is_dense = false;
  if (pcl::io::savePCDFileBinary(path, save_buffer_) == 0) {
    logf(LogLevel::kInfo, "彩色地图已保存: %s（%zu 点，世界系%s）", path.c_str(),
                save_buffer_.size(),
                (save_median_ && save_median_->enabled()) ? "，时间中位数回填色" : "");
  } else {
    logfThrottled(LogLevel::kWarn, 5000,
                         "保存 PCD 失败: %s", path.c_str());
  }
}

void VisualPipeline::shutdownAndSave() {
  // 用 call_once 保证只执行一次：shutdown 钩子跑在信号线程，主线程在
  // executor.spin() 返回后也会调一次，两者可能并发；后到者阻塞到这次执行
  // 结束，从而保证进程退出前 PCD 已经写完。
  std::call_once(shutdown_once_, [this]() {
    // 先停 worker（保留落盘线程）：worker 排空输入队列后退出。
    stopWorkerThreads(/*keep_save_thread=*/true);
    // 收尾：bag 结束时队列里可能还有等图像/等位姿的帧，尽量补完再落盘。
    // 待处理视觉帧不会再等到位姿了，直接按最新位姿算完，保证轨迹不缺尾段。
    for (const auto& frame : pending_visual_) {
      processVisualFrame(frame.gray, frame.stamp);
    }
    pending_visual_.clear();
    drainPendingClouds();
    drainPendingColorization();
    // 分片模式：把最后不足一个间隔的尾段也交给落盘线程写一片，
    // 保证「分片并集 == 合并图」，时序滤波不会漏掉结尾。
    if (save_interval_frames_ > 0) {
      requestColoredMapSave();
    }
    stopSaveThread();
    // 同步写出整张彩色地图。长序列（几百万点 + 中位数回填）要几秒，
    // 这一步必须在本进程内完成，不能交给 save 线程（进程退出时线程被杀）。
    saveColoredMap();
    // 延时总报告（worker 已 join，无并发 Record；60 s 窗口报告之外的尾量在这）
    if (timing_en_) {
      latency_timer_.PrintAll();
    }
  });
}

VisualPipeline::~VisualPipeline() {
  shutdownAndSave();
  probeReport();  // 收尾打一份完整探针汇总
}

void VisualPipeline::loadParameters() {
  // ---- 延时计时开关（仿 lio.eva.timer 口径；app 的 ParamSource 已带
  //      super_livo_visual. 前缀，YAML/-p 用全名 super_livo_visual.timing）----
  timing_en_ = params_.getBool("timing", false);
  // ---- 相机内参与畸变（同方案 A，来自 camera_pinhole.yaml） --------------
  scale_ = params_.getDouble("scale", 0.5);
  intrinsics_.width = params_.getInt("cam_width", 640);
  intrinsics_.height = params_.getInt("cam_height", 512);
  intrinsics_.fx = params_.getDouble("cam_fx", 646.78);
  intrinsics_.fy = params_.getDouble("cam_fy", 646.66);
  intrinsics_.cx = params_.getDouble("cam_cx", 313.46);
  intrinsics_.cy = params_.getDouble("cam_cy", 261.40);
  distortion_ = params_.getDoubleArray(
      "cam_d", std::vector<double>{-0.076160, 0.123001, -0.00113, 0.000251});

  // ---- 外参：Camera->LiDAR（Rcl/Pcl）与 LiDAR->IMU（可选） ----------------
  T_lc_ = makeTlcFromRclPcl();
  // T_li（LiDAR->IMU）默认单位阵：Super-LIO 把 IMU 量测旋转到 LiDAR 系，
  // 其状态本身就以 LiDAR 为 body，因此这里取单位阵即可（T_wi ≈ T_wl）。
  T_li_ = Eigen::Matrix4d::Identity();

  // LiDAR 先验强度（D2）：先验越小越"信任" lio/odom，视觉修正越弱。
  prior_pos_sigma_ = params_.getDouble("update.prior_pos_sigma", 0.05);
  prior_rot_sigma_ = params_.getDouble("update.prior_rot_sigma", 0.02);
  prior_vel_sigma_ = params_.getDouble("update.prior_vel_sigma", 0.25);
  // 退化感知先验：退化方向松开、垂直方向收紧（详见头文件注释）
  prior_directional_en_ =
      params_.getBool("update.directional_prior_en", true);
  prior_pos_sigma_healthy_ =
      params_.getDouble("update.prior_pos_sigma_healthy", 0.005);
  prior_pos_sigma_degen_ =
      params_.getDouble("update.prior_pos_sigma_degen", 1.0);
  // sev 对数映射标尺（本数据集实测）：健康段 cond_t≈25-40，必须映射为 sev≈0，
  // 否则先验在健康段就被无端松开一半（旧默认 10/100 使 cond_t=33 映射成 0.5）。
  // 完全退化（n_effect=0，cond=1e12）由 axes/active 硬触发直接置 1，不走该映射。
  gate_cond_lo_ = params_.getDouble(
      "gate.cond_lo", superlivo_common::DegeneracyScale{}.cond_lo);
  gate_cond_hi_ = params_.getDouble(
      "gate.cond_hi", superlivo_common::DegeneracyScale{}.cond_hi);
  degen_timeout_ = params_.getDouble(
      "gate.degen_timeout", superlivo_common::DegeneracyScale{}.timeout_sec);

  // 退化段守卫（Trust + 速度限幅）
  guard_enable_ = params_.getBool("guard.enable", true);
  guard_lio_jump_pos_ = params_.getDouble("guard.lio_jump_pos", 1.0);
  guard_lio_jump_rot_ = params_.getDouble("guard.lio_jump_rot", 0.2618);
  guard_sev_threshold_ = params_.getDouble("guard.sev_threshold", 0.9);
  guard_vel_ratio_ = params_.getDouble("guard.vel_ratio", 1.5);
  guard_vel_fallback_ = params_.getDouble("guard.vel_fallback", 3.0);
  guard_vel_abs_cap_ = params_.getDouble("guard.vel_abs_cap", 3.0);

  // 默认值与官方 avia.yaml 一致（0.1 s），本数据集已通过 offset 扫描验证最优。
  img_time_offset_ = params_.getDouble("img_time_offset", 0.1);

  // 启动时间对齐：off = −(D + a)，D 由 IMU 桥每次会话现测，a 为相机打戳延迟标定值。
  // align.enable=false 时退回上面手填的 img_time_offset。
  align_enable_ = params_.getBool("align.enable", true);
  align_samples_ = static_cast<std::size_t>(
      params_.getInt("align.samples", 20));
  align_window_ms_ = params_.getDouble("align.window_ms", 50.0);
  align_img_stamp_latency_s_ =
      params_.getDouble("align.img_stamp_latency_s", 0.111);
  align_clock_lag_max_s_ = params_.getDouble("align.clock_lag_max_s", 2.0);
  align_timeout_s_ = params_.getDouble("align.timeout_s", 10.0);

  // ---- 前端 PatchTracker --------------------------------------------------
  patch_options_.patch_size = params_.getInt("patch.patch_size", 5);
  patch_options_.pyramid_levels = params_.getInt("patch.pyramid_levels", 4);
  patch_options_.max_features = params_.getInt("patch.max_features", 200);
  patch_options_.lssd_threshold = params_.getDouble("patch.lssd_threshold", 0.5);
  patch_options_.min_distance = params_.getDouble("patch.min_distance", 8.0);
  patch_options_.quality_level = params_.getDouble("patch.quality_level", 0.01);
  patch_options_.use_lssd = params_.getBool("patch.use_lssd", true);
  patch_options_.search_radius = params_.getInt("patch.search_radius", 4);
  frontend_ = std::make_unique<PatchTracker>(patch_options_);

  // ---- 特征生命周期 FeatureLifecycle --------------------------------------
  lifecycle_options_.mature_observations = params_.getInt("lifecycle.mature_observations", 4);
  lifecycle_options_.max_failures = params_.getInt("lifecycle.max_failures", 2);
  lifecycle_options_.max_track_length = params_.getInt("lifecycle.max_track_length", 30);
  lifecycle_ = std::make_unique<FeatureLifecycle>(lifecycle_options_);

  // ---- 关键帧管理 KeyframeManager -----------------------------------------
  keyframe_options_.max_keyframes = params_.getInt("kf.max_keyframes", 20);
  keyframe_options_.min_translation = params_.getDouble("kf.min_translation", 0.15);
  keyframe_options_.min_rotation_deg = params_.getDouble("kf.min_rotation_deg", 5.0);
  keyframe_options_.min_tracked_features = params_.getInt("kf.min_tracked_features", 40);
  keyframe_manager_ = std::make_unique<KeyframeManager>(keyframe_options_);

  // ---- 2D-3D 约束 LidarVisualConstraint -----------------------------------
  lidar_visual_options_.fx = intrinsics_.fx;
  lidar_visual_options_.fy = intrinsics_.fy;
  lidar_visual_options_.cx = intrinsics_.cx;
  lidar_visual_options_.cy = intrinsics_.cy;
  lidar_visual_options_.image_width = intrinsics_.width;
  lidar_visual_options_.image_height = intrinsics_.height;
  lidar_visual_options_.pixel_radius = params_.getInt("depth.pixel_radius", 3);
  lidar_visual_options_.huber_delta = params_.getDouble("reproj.huber_delta", 1.0);
  lidar_visual_options_.min_constraints = params_.getInt("reproj.min_constraints", 8);
  lidar_visual_options_.max_pixel_error = params_.getDouble("reproj.max_pixel_error", 8.0);
  lidar_visual_options_.max_pixel_error_anchored =
      params_.getDouble("reproj.max_pixel_error_anchored", 60.0);
  // 测量噪声模型：没有它等价于 σ=1 px 且地图点无误差，一帧即可压塌协方差。
  lidar_visual_options_.sigma_pix = params_.getDouble("reproj.sigma_pix", 1.5);
  lidar_visual_options_.sigma_anchor = params_.getDouble("reproj.sigma_anchor", 0.05);
  lidar_visual_options_.anchor_keep_sec =
      params_.getDouble("depth.anchor_keep_sec", 20.0);
  lidar_visual_ = std::make_unique<LidarVisualConstraint>(lidar_visual_options_);

  // ---- MSCKF 多帧约束 MsckfConstraint -------------------------------------
  msckf_options_.fx = intrinsics_.fx;
  msckf_options_.fy = intrinsics_.fy;
  msckf_options_.cx = intrinsics_.cx;
  msckf_options_.cy = intrinsics_.cy;
  msckf_options_.min_observations = params_.getInt("msckf.min_observations", 4);
  msckf_options_.max_track_length = params_.getInt("msckf.max_track_length", 20);
  msckf_options_.min_parallax_deg = params_.getDouble("msckf.min_parallax_deg", 1.0);
  msckf_options_.huber_delta = params_.getDouble("msckf.huber_delta", 1.0);
  msckf_options_.use_nullspace_projection = params_.getBool("msckf.use_nullspace_projection", true);
  msckf_options_.sigma_pix = params_.getDouble("msckf.sigma_pix", 1.5);
  msckf_options_.sigma_track_pose = params_.getDouble("msckf.sigma_track_pose", 0.05);
  msckf_ = std::make_unique<MsckfConstraint>(msckf_options_);

  // ---- 光度约束 PhotometricConstraint（可选，默认关） ----------------------
  photo_options_.fx = intrinsics_.fx;
  photo_options_.fy = intrinsics_.fy;
  photo_options_.cx = intrinsics_.cx;
  photo_options_.cy = intrinsics_.cy;
  photo_options_.enable = params_.getBool("photo.enable", false);
  photo_options_.patch_size = params_.getInt("photo.patch_size", 5);
  photo_options_.pyramid_levels = params_.getInt("photo.pyramid_levels", 3);
  photo_options_.huber_delta = params_.getDouble("photo.huber_delta", 1.0);
  photo_options_.max_intensity_error = params_.getDouble("photo.max_intensity_error", 40.0);
  photometric_ = std::make_unique<PhotometricConstraint>(photo_options_);

  // ---- 统一估计器 UnifiedEstimator ----------------------------------------
  estimator_options_.enable_visual = params_.getBool("update.enable", true);
  estimator_options_.visual_weight = params_.getDouble("est.visual_weight", 1.0);
  estimator_options_.estimate_extrinsics = params_.getBool("est.estimate_extrinsics", false);
  estimator_options_.gyro_noise = params_.getDouble("est.gyro_noise", 1.0e-3);
  estimator_options_.acc_noise = params_.getDouble("est.acc_noise", 1.0e-2);
  estimator_options_.bias_gyro_noise = params_.getDouble("est.bias_gyro_noise", 1.0e-5);
  estimator_options_.bias_acc_noise = params_.getDouble("est.bias_acc_noise", 1.0e-4);
  estimator_options_.initial_covariance = params_.getDouble("est.initial_covariance", 1.0);
  // 视觉注入强度上限与信息耦合方式（详见 UnifiedEstimator::Options 注释）。
  estimator_options_.visual_info_ratio = params_.getDouble("est.visual_info_ratio", 1.0);
  estimator_options_.visual_rot_sigma_floor =
      params_.getDouble("est.visual_rot_sigma_floor", 0.005);
  estimator_options_.visual_pos_sigma_floor =
      params_.getDouble("est.visual_pos_sigma_floor", 0.02);
  // 退化时的视觉上限（LiDAR 已不可信，需让视觉真正接管）
  estimator_options_.visual_info_ratio_degen =
      params_.getDouble("est.visual_info_ratio_degen", 5.0);
  estimator_options_.visual_pos_sigma_floor_degen =
      params_.getDouble("est.visual_pos_sigma_floor_degen", 0.01);
  estimator_options_.visual_pose_only = params_.getBool("est.visual_pose_only", true);
  estimator_ = std::make_unique<UnifiedEstimator>(estimator_options_);

  // ---- 延迟地图提交 MapCommitScheduler ------------------------------------
  scheduler_options_.enable = params_.getBool("scheduler.enable", true);
  scheduler_options_.max_pending = params_.getInt("scheduler.max_pending", 3);
  scheduler_ = std::make_unique<MapCommitScheduler>(scheduler_options_);

  // ---- 彩色点云 ColorizerPipeline（复用方案 A 的包） ----------------------
  colorizer_options_.intrinsics = intrinsics_;
  // 着色轨迹源：默认 false=/lio/odom；true=统一估计器修正后的位姿（own_pose_cache_）。
  color_use_own_odom_ = params_.getBool("color.use_own_odom", false);
  // own 轨迹信任域（见 pushOwnPoseSample / 12.5）：残差超过上限按上限截断。
  own_res_max_pos_ = params_.getDouble("color.own_res_max_pos", 0.5);
  own_res_max_rot_ = params_.getDouble("color.own_res_max_rot", 0.035);
  // T_cl 在实现中的语义是 T_cw（作用于世界系点），逐帧变化，见 runColorizer。
  colorizer_options_.T_cl = Eigen::Matrix4d::Identity();
  colorizer_options_.sync.max_offset = params_.getDouble("color.sync_max_offset", 0.15);
  colorizer_options_.sync.use_exposure_mid = params_.getBool("color.use_exposure_mid", true);
  colorizer_options_.occlusion.depth_tolerance = params_.getDouble("color.depth_tolerance", 0.1);
  colorizer_options_.occlusion.check_depth_edges = params_.getBool("color.check_depth_edges", true);
  colorizer_options_.occlusion.edge_threshold = params_.getDouble("color.edge_threshold", 0.5);
  colorizer_options_.rgb.bilinear = params_.getBool("color.bilinear", true);
  colorizer_options_.rgb.keep_best_quality = params_.getBool("color.keep_best_quality", true);
  colorizer_options_.rgb.min_quality = params_.getFloat("color.min_quality", 0.2F);

  // 相机位姿源（官方口径）：点云世界变换用 LIO 位姿，相机位姿用 VIO 光度修正后的位姿。
  // 留空表示与 odom_topic 同源（旧行为）。订阅由 app 负责，这里只做提示。
  const std::string cam_odom_topic =
      params_.getString("color.cam_odom_topic", "");
  if (!cam_odom_topic.empty()) {
    logf(LogLevel::kInfo,
         "相机位姿源: %s（点云世界变换仍用 odom_topic；订阅由 app 提供）",
         cam_odom_topic.c_str());
  }

  // ---- 彩色地图落盘（把跑完的彩色点云输出为 PCD） ------------------------
  pcd_save_dir_ = params_.getString("pcd.save_dir", "/home/ywj/slam_data/pcd");
  pcd_save_interval_sec_ = params_.getDouble("pcd.save_interval_sec", 30.0);
  max_colored_map_points_ = static_cast<std::size_t>(
      params_.getInt("pcd.max_points", 20000000));
  median_map_.set_voxel_size(
      static_cast<float>(params_.getDouble("pcd.median_voxel_size", 0.01)));
  // 分片落盘：每 N 个彩色帧异步写一个增量 PCD（0 = 只在退出时写整图）。
  save_interval_frames_ = static_cast<std::size_t>(
      params_.getInt("pcd.save_interval_frames", 0));

  // ---- E2 诊断落盘 --------------------------------------------------------
  // 默认关闭；打开后只多写两个 CSV，不改变估计器输入输出。
  e2_dump_enable_ = params_.getBool("e2.dump_enable", false);
  e2_dump_dir_ = params_.getString("e2.dump_dir", "");
  if (e2_dump_enable_ && !e2_dump_dir_.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(e2_dump_dir_, ec);
    if (ec) {
      logf(LogLevel::kError, "无法创建 E2 输出目录 %s: %s",
                   e2_dump_dir_.c_str(), ec.message().c_str());
    } else {
      const std::filesystem::path dir(e2_dump_dir_);
      e2_pose_ofs_.open((dir / "e2_pose.csv").string());
      e2_obs_ofs_.open((dir / "e2_obs.csv").string());
      logf(LogLevel::kInfo, "E2 落盘开启: %s", e2_dump_dir_.c_str());
    }
  }

  // ---- 旋转后处理（P0-F RotationRefiner 移植，默认关）---------------------
  // 只做"旋转后处理"：R' = Exp(u)·R_lio，位置原样透传；不触碰 ESKF 状态与地图。
  rot_refine_enable_ = params_.getBool("rot_refine.enable", false);
  // 像素对门限默认 40：与 §36.14 离线回放一致（估计器内部另有 min_inl=25 的内点门）。
  rot_estimator_options_.min_inl =
      params_.getInt("rot_refine.min_pairs", 40);
  rot_noise_mode_ = params_.getString("rot_refine.noise_mode", "w3");
  rot_estimator_options_.seed_px =
      params_.getDouble("rot_refine.seed_px", rot_estimator_options_.seed_px);
  rot_estimator_options_.f_scale =
      params_.getDouble("rot_refine.f_scale", rot_estimator_options_.f_scale);
  rot_estimator_options_.rounds =
      params_.getInt("rot_refine.rounds", rot_estimator_options_.rounds);
  rot_refine_options_.window =
      params_.getInt("rot_refine.window", rot_refine_options_.window);
  rot_refine_options_.sigma_prior_deg = params_.getDouble(
      "rot_refine.sigma_prior_deg", rot_refine_options_.sigma_prior_deg);
  rot_refine_options_.sigma_accel_deg = params_.getDouble(
      "rot_refine.sigma_accel_deg", rot_refine_options_.sigma_accel_deg);
  rot_refine_options_.clip_deg =
      params_.getDouble("rot_refine.clip_deg", rot_refine_options_.clip_deg);
  rot_refine_options_.slew_factor = params_.getDouble(
      "rot_refine.slew_factor", rot_refine_options_.slew_factor);
  rot_refine_options_.huber_c =
      params_.getDouble("rot_refine.huber_c", rot_refine_options_.huber_c);
  rot_refine_options_.irls_rounds =
      params_.getInt("rot_refine.irls_rounds", rot_refine_options_.irls_rounds);
  rot_refine_options_.max_iters =
      params_.getInt("rot_refine.max_iters", rot_refine_options_.max_iters);
  rot_refine_options_.enable_bounds =
      params_.getBool("rot_refine.enable_bounds", rot_refine_options_.enable_bounds);
  const std::string rot_dump_csv =
      params_.getString("rot_refine.dump_csv", "");

  // 相机外参（Camera->LiDAR）：旋转必须投影到 SO(3) 后再用于 log/exp。
  R_bc_ = Eigen::Quaterniond(T_lc_.block<3, 3>(0, 0)).normalized().toRotationMatrix();
  t_bc_ = T_lc_.block<3, 1>(0, 3);
  // 估计器内参必须与 PatchTracker 完全同口径（NTU 752x480，非默认值）。
  rot_estimator_options_.fx = intrinsics_.fx;
  rot_estimator_options_.fy = intrinsics_.fy;
  rot_estimator_options_.cx = intrinsics_.cx;
  rot_estimator_options_.cy = intrinsics_.cy;

  if (rot_refine_enable_) {
    rot_estimator_ = std::make_unique<RotationEstimator>(rot_estimator_options_);
    rot_refiner_ = std::make_unique<RotationRefiner>(rot_refine_options_);
    if (!rot_dump_csv.empty()) {
      const std::filesystem::path csv(rot_dump_csv);
      std::error_code ec;
      if (!csv.parent_path().empty()) {
        std::filesystem::create_directories(csv.parent_path(), ec);
      }
      rot_refine_ofs_.open(csv.string());
      if (!rot_refine_ofs_.is_open()) {
        logf(LogLevel::kError, "无法写入旋转精修诊断 CSV: %s",
                     rot_dump_csv.c_str());
      } else {
        rot_refine_ofs_ << "stamp,t_pose,n_pairs,n_inl,t_norm,th_deg,"
                           "rvx,rvy,rvz,ux,uy,uz,sgx,sgy,sgz,px_med,px_p90,d_med,usable\n";
      }
    }
    logf(LogLevel::kInfo,
                "旋转精修开启: window=%d σp=%.2f° σa=%.2f° clip=%.2f° bounds=%d",
                rot_refine_options_.window, rot_refine_options_.sigma_prior_deg,
                rot_refine_options_.sigma_accel_deg, rot_refine_options_.clip_deg,
                static_cast<int>(rot_refine_options_.enable_bounds));
  }

}

void VisualPipeline::onImage(const cv::Mat& bgr_in, double stamp) {
  // 图像 header 时刻：与 dense 点云 header 同一时钟基准（实测差 < 7 ms），只用于配对。
  // 图像真实采集时刻 header + img_time_offset_ 只用于取相机位姿，见 runColorizer。
  const auto t_arrive = std::chrono::steady_clock::now();
  ++g_probe.n_image;
  probeGap(g_probe.image_gap, g_probe.t_image, t_arrive);
  probeHeaderSpan(g_probe.image_hgap, g_probe.first_image, g_probe.last_image, stamp);

  // 启动时间对齐期间该帧只当样本、不进视觉流程（对齐完成前一律丢弃）。
  if (align_enable_ && !align_done_ && !align_failed_) {
    feedAlignSample(true, stamp, t_arrive);
    if (!align_done_) {
      return;
    }
  }

  // 传入的 Mat 可能是引用消息内存的浅拷贝（事件 keepalive 保活到本函数返回）；
  // 下面 rectifyBgr 恒走 remap，进 image_cache_ 的是自有缓冲。
  cv::Mat bgr = bgr_in;
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

  // 等 t_pose 时刻的位姿到齐再处理，避免用 0.2 s 前的位姿算视觉残差。
  const double t_pose = stamp + img_time_offset_;
  const double newest_pose_stamp = pose_cache_.empty() ? -1e18 : pose_cache_.back().stamp;
  if (newest_pose_stamp >= t_pose) {
    processVisualFrame(gray, stamp);
    return;
  }
  pending_visual_.push_back({gray, stamp, std::chrono::steady_clock::now()});
  while (pending_visual_.size() > kMaxPendingVisualFrames) {
    ++stat_drop_pose_;
    ++g_probe.drop_visual_full;
    logfThrottled(LogLevel::kWarn, 5000,
                         "视觉[丢弃] 待定队列超限（%zu 帧），丢弃最旧帧", pending_visual_.size());
    pending_visual_.pop_front();
  }
  probeMaybeReport();
}

void VisualPipeline::onDegeneracy(
    const superlivo_common::DegeneracySignature& sig) {
  std::lock_guard<std::mutex> lock(degen_mutex_);
  degen_sig_ = sig;
  degen_recv_time_ = steadyNowSec();
}

bool VisualPipeline::degeneracySignature(double& severity,
                                      Eigen::Vector3d& dir_w) const {
  severity = 0.0;
  dir_w.setZero();
  std::lock_guard<std::mutex> lock(degen_mutex_);
  if (degen_recv_time_ < 0.0 ||
      steadyNowSec() - degen_recv_time_ > degen_timeout_) {
    return false;
  }
  // cond=0 说明 Super-LIO 侧没跑检测或不是 trans_only 模式，不能当"健康"用。
  if (!degen_sig_.valid()) {
    return false;
  }
  dir_w = degen_sig_.dir;
  severity = degen_sig_.severity(gate_cond_lo_, gate_cond_hi_);
  return true;
}

void VisualPipeline::onOdom(const Eigen::Matrix4d& T_w_in, double stamp) {
  const auto t_arrive = std::chrono::steady_clock::now();
  Eigen::Matrix4d T_wl = T_w_in;

  // 若输入是 base_link（robot body）系：T_w_body = T_w_imu · T_imu_body。
  // 本流水线的 ESKF、IMU 量测、T_li=I 的点云全在 IMU 系，先把外参除掉。
  // 单体钩子直供 IMU 系位姿时 T_imu_robo_ 保持单位阵，此处无操作。
  if (!T_imu_robo_.isApprox(Eigen::Matrix4d::Identity(), 1e-12)) {
    T_wl = T_wl * T_imu_robo_.inverse();
  }
  T_wl_ = T_wl;

  colorizer::PoseSample sample;
  sample.stamp = stamp;
  // 启动时间对齐：这条 /lio/odom 的（帧戳, 系统到达时刻）作为一个雷达样本。
  if (align_enable_ && !align_done_ && !align_failed_) {
    feedAlignSample(false, sample.stamp, t_arrive);
  }
  sample.T_wl = T_wl;
  pose_cache_.push_back(sample);
  if (pose_cache_.size() > max_pose_samples_) {
    pose_cache_.erase(pose_cache_.begin(),
                      pose_cache_.begin() + (pose_cache_.size() - max_pose_samples_));
  }
  ++g_probe.n_odom;
  probeGap(g_probe.odom_gap, g_probe.t_odom, std::chrono::steady_clock::now());
  probeHeaderSpan(g_probe.odom_hgap, g_probe.first_odom, g_probe.last_odom, sample.stamp);

  // own 着色轨迹与 /lio/odom 同频落缓存（残差零阶保持）。
  // 只在视觉锚点写 own_pose_cache_ 是不够的：本数据集相机 1 Hz ⇒ 锚点间隔 1 s，
  // 而 poseFromSamples 会拒绝相邻采样间隔 >0.5 s 的查询（kPoseHistoryMaxGapSec），
  // 10 Hz 的点云因此基本查不到 own 位姿 —— 要么"位姿空洞"、要么等下一个锚点时
  // 已超时/挤爆待着色队列，实测每 10 帧云只有约 1 帧能上色。
  if (color_use_own_odom_ && own_anchor_t_ >= 0.0) {
    pushOwnPoseSample(sample.stamp, sample.T_wl);
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
      logf(LogLevel::kInfo,
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
    logf(LogLevel::kInfo, "统一状态已用首帧 lio/odom 初始化");
  }

  // 新采样到齐后重试待处理视觉帧（t_pose 通常比 /lio/odom 的最新时刻还晚）。
  if (!pending_visual_.empty()) {
    drainPendingVisualFrames();
  }

  // 新采样到齐后重试待着色帧（t_img = t_cloud + img_time_offset，通常要等未来位姿）。
  if (!pending_color_.empty()) {
    drainPendingColorization();
  }
  probeMaybeReport();
}

// ============================================================================
// 启动时间对齐
// ----------------------------------------------------------------------------
// 相机与雷达各自打戳，两者可能相差一整个时钟域（livo_bag 相差 2×10^8 s），
// 且 10 Hz 下"取哪一帧"的整数周期无法只靠时刻分辨（相位每 100 ms 重复一次，
// 差一帧的两种配对残差一样小）。因此启动对齐 = 一个绝对锚点 + 一个标定常量：
//   D   = 系统钟 − 雷达钟。IMU 与雷达同一钟域、报文小、几乎不积压，用它的
//         (到达, header) 逐样本测 D，取最近窗口的中位数。
//   a   = 相机打戳延迟（一次性标定，见 align.img_stamp_latency_s）。
//   off = −(D + a)，冻结后严格按它取时刻。
// 图像/雷达到达时刻只用于校验：按 D 配对后残差应≈0。
// 若系统钟与图像 header 相差过大，说明节点跑在离线回放上（now 不是传感器主机钟），
// D 不可用，直接退回配置 img_time_offset 并告警。
// ============================================================================
void VisualPipeline::feedAlignSample(bool is_image, double header,
                                  std::chrono::steady_clock::time_point arrival) {
  auto& buf = is_image ? align_images_ : align_lidar_;
  buf.push_back(AlignSample{header, arrival});
  // 只留最近若干条：对齐窗口不需要历史，长期对不上时也避免无限增长。
  const std::size_t cap = std::max<std::size_t>(2 * align_samples_, 40);
  while (buf.size() > cap) {
    buf.pop_front();
  }
  if (align_t0_ == std::chrono::steady_clock::time_point{}) {
    align_t0_ = arrival;
  }
  tryAlign();
}

void VisualPipeline::tryAlign() {
  if (align_done_ || align_failed_) {
    return;
  }
  auto median_of = [](std::vector<double> v) {
    const std::size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    return v[mid];
  };

  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - align_t0_).count();
  if (elapsed > align_timeout_s_) {
    align_failed_ = true;
    logf(LogLevel::kWarn,
         "启动时间对齐失败（%.1f s 内样本不足：图像 %zu 帧、IMU %zu 条），继续使用配置 "
         "img_time_offset=%+.6f s",
         elapsed, align_images_.size(), align_d_samples_.size(), img_time_offset_);
    return;
  }
  // 等够样本：align.samples 张图像（默认 20 帧 ≈2 s）+ 0.25 s 的 IMU。
  if (align_images_.size() < align_samples_ || align_lidar_.empty() ||
      align_d_samples_.size() < 50) {
    return;
  }

  // ---- 1) 系统钟是否仍是传感器主机钟 --------------------------------------
  // 离线回放时系统钟是"今天"的墙钟，而图像 header 是录制时的墙钟（可差几天），
  // 由此算出的 D 完全失真；且差值通常是 0.1 s 的整数倍，相位校验也看不出来。
  const double now_s =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
          .count();
  std::vector<double> lags;
  lags.reserve(align_images_.size());
  for (const auto& im : align_images_) {
    lags.push_back(now_s - im.header);
  }
  const double lag = median_of(lags);
  if (std::abs(lag) > align_clock_lag_max_s_) {
    align_failed_ = true;
    logf(LogLevel::kWarn,
         "启动时间对齐放弃：系统钟与图像 header 相差 %.1f s（离线回放，系统钟不是"
         "传感器主机钟），无法恢复雷达钟域；沿用配置 img_time_offset=%+.6f s",
         lag, img_time_offset_);
    align_images_.clear();
    align_lidar_.clear();
    align_d_samples_.clear();
    return;
  }

  // ---- 2) 冻结 off = −(D + a) ---------------------------------------------
  const std::vector<double> dv(align_d_samples_.begin(), align_d_samples_.end());
  const double D = median_of(dv);
  const double off = -(D + align_img_stamp_latency_s_);

  // ---- 3) 校验：按到达时刻配对后，图像应几乎正好落在雷达帧上 ---------------
  const double win_ns = align_window_ms_ * 1e6;
  std::vector<double> cand;
  cand.reserve(align_images_.size());
  for (const auto& im : align_images_) {
    const AlignSample* best = nullptr;
    double best_ns = 0.0;
    for (const auto& ld : align_lidar_) {
      const double d =
          std::chrono::duration<double, std::nano>(im.arrival - ld.arrival).count();
      if (best == nullptr || std::abs(d) < std::abs(best_ns)) {
        best = &ld;
        best_ns = d;
      }
    }
    if (best == nullptr || std::abs(best_ns) > win_ns) {
      continue;  // 该图不落在任何雷达帧附近（多速率相机 / 启动错位）
    }
    cand.push_back(best->header - im.header);
  }
  double resid_ms = std::numeric_limits<double>::quiet_NaN();
  if (!cand.empty()) {
    resid_ms = (off - median_of(cand)) * 1e3;
  }

  const double old = img_time_offset_;
  img_time_offset_ = off;
  align_done_ = true;
  logf(LogLevel::kInfo,
       "启动时间对齐完成：D(系统钟−雷达钟)=%+.6f s，相机打戳延迟=%.1f ms "
       "⇒ img_time_offset=%+.6f s（配置值 %+.6f；%zu 帧校验残差中位 %+.1f ms）",
       D, align_img_stamp_latency_s_ * 1e3, off, old, cand.size(), resid_ms);
  align_images_.clear();
  align_lidar_.clear();
  align_d_samples_.clear();
}

void VisualPipeline::onCamOdom(const Eigen::Matrix4d& T_w, double stamp) {
  colorizer::PoseSample sample;
  sample.stamp = stamp;
  sample.T_wl = T_w;
  cam_pose_cache_.push_back(sample);
  if (cam_pose_cache_.size() > max_pose_samples_) {
    cam_pose_cache_.erase(cam_pose_cache_.begin(),
                          cam_pose_cache_.begin() +
                              (cam_pose_cache_.size() - max_pose_samples_));
  }
  if (!pending_color_.empty()) {
    drainPendingColorization();
  }
}

void VisualPipeline::onCloud(const std::vector<Eigen::Vector3d>& points_world) {
  ++g_probe.n_cloud;
  probeGap(g_probe.cloud_gap, g_probe.t_cloud, std::chrono::steady_clock::now());
  // 世界系点，直接作为 2D-3D 约束的地图点来源。
  for (std::size_t i = 0; i < points_world.size();
       i += static_cast<std::size_t>(std::max(1, map_append_stride_))) {
    const auto& p = points_world[i];
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
      continue;
    }
    map_points_.emplace_back(p);
  }
  if (map_points_.size() > max_map_points_) {
    map_points_.erase(map_points_.begin(),
                      map_points_.begin() + (map_points_.size() - max_map_points_));
  }
  world_cloud_valid_ = true;
}

void VisualPipeline::onBodyCloud(const std::vector<Eigen::Vector3f>& points_lidar,
                                 double stamp) {
  body_points_.clear();
  body_points_.reserve(points_lidar.size());
  for (const auto& p : points_lidar) {
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
      continue;
    }
    body_points_.push_back(p);
  }
  body_stamp_ = stamp;
  ++g_probe.n_body;
  probeGap(g_probe.body_gap, g_probe.t_body, std::chrono::steady_clock::now());
  probeHeaderSpan(g_probe.body_hgap, g_probe.first_body, g_probe.last_body, body_stamp_);

  // 新扫描到达即入队等图像/位姿（大图回调比云慢约 1 帧，不能等图像回调才入队）。
  runColorizer();
}

void VisualPipeline::onImu(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,
                           double stamp) {
  ++g_probe.n_imu;
  probeGap(g_probe.imu_gap, g_probe.t_imu, std::chrono::steady_clock::now());
  if (align_enable_ && !align_done_ && !align_failed_) {
    // D = 系统钟 − 雷达钟。IMU 报文小、几乎不积压，单样本偏差只有 ms 量级，
    // 留最近 2 s（200 Hz）样本，取中位数压掉抖动。
    const double now_s =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    align_d_samples_.push_back(now_s - stamp);
    if (align_d_samples_.size() > 400) {
      align_d_samples_.pop_front();
    }
    if (align_t0_ == std::chrono::steady_clock::time_point{}) {
      align_t0_ = std::chrono::steady_clock::now();
    }
    tryAlign();
  }
  ImuSample sample;
  sample.stamp = stamp;
  sample.acc = acc;
  sample.gyro = gyro;
  imu_buffer_.push_back(sample);
  if (imu_buffer_.size() > max_imu_samples_) {
    imu_buffer_.erase(imu_buffer_.begin(),
                      imu_buffer_.begin() + (imu_buffer_.size() - max_imu_samples_));
  }
}

void VisualPipeline::probeMaybeReport() {
  const auto now = std::chrono::steady_clock::now();
  if (g_probe.t0.time_since_epoch().count() == 0) {
    g_probe.t0 = now;
    g_probe.t_report = now;
    return;
  }
  if (std::chrono::duration<double>(now - g_probe.t_report).count() < 2.0) {
    return;
  }
  g_probe.t_report = now;
  probeReport();
}

void VisualPipeline::probeReport() {
  BranchProbe& p = g_probe;
  const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - p.t0).count();
  const double win = t - std::max(0.0, p.t_last_report);
  p.t_last_report = t;
  auto dt = [](std::size_t cur, std::size_t& prev) {
    const std::size_t d = cur - prev;
    prev = cur;
    return d;
  };

  logf(LogLevel::kInfo,
              "[PROBE-IO] t=%.1f/%.1fs 到达(img|odom|body|cloud|imu)="
              "%zu/%zu/%zu/%zu/%zu Δ%zu/%zu/%zu/%zu/%zu | 间隔ms avg|max img=%.1f|%.1f "
              "odom=%.1f|%.1f body=%.1f|%.1f | headerΔms max img=%.1f odom=%.1f body=%.1f",
              win, t, p.n_image, p.n_odom, p.n_body, p.n_cloud, p.n_imu,
              dt(p.n_image, p.prev_image), dt(p.n_odom, p.prev_odom), dt(p.n_body, p.prev_body),
              dt(p.n_cloud, p.prev_cloud), dt(p.n_imu, p.prev_imu),
              p.image_gap.avg(), p.image_gap.max, p.odom_gap.avg(), p.odom_gap.max,
              p.body_gap.avg(), p.body_gap.max, p.image_hgap.max, p.odom_hgap.max,
              p.body_hgap.max);
  logf(LogLevel::kInfo,
              "[PROBE-T] 时刻 img[%.2f→%.2f] odom[%.2f→%.2f] body[%.2f→%.2f] | "
              "处理 Δ=%zu/%zu gap=%.1f|%.1f 等位姿=%.1f|%.1f | "
              "耗时ms avg|max 前端=%.1f|%.1f 约束=%.1f|%.1f 更新=%.1f|%.1f 彩色=%.1f|%.1f "
              "合计=%.1f|%.1f",
              p.first_image, p.last_image, p.first_odom, p.last_odom, p.first_body, p.last_body,
              dt(p.n_process, p.prev_process), p.n_process, p.process_gap.avg(), p.process_gap.max,
              p.wait_visual.avg(), p.wait_visual.max, p.ms_front.avg(), p.ms_front.max,
              p.ms_constraint.avg(), p.ms_constraint.max, p.ms_update.avg(), p.ms_update.max,
              p.ms_color.avg(), p.ms_color.max, p.ms_total.avg(), p.ms_total.max);
  logf(LogLevel::kInfo,
              "[PROBE-S] 约束细分ms avg|max 拷贝地图=%.1f|%.1f 投影建栅格=%.1f|%.1f "
              "法方程=%.1f|%.1f 选轨=%.1f|%.1f mscfk=%.1f|%.1f | "
              "上色内部ms 流水线=%.1f|%.1f 并入地图=%.1f|%.1f 发布=%.1f|%.1f",
              p.ms_copy_map.avg(), p.ms_copy_map.max, p.ms_project.avg(), p.ms_project.max,
              p.ms_normal_eq.avg(), p.ms_normal_eq.max, p.ms_select.avg(), p.ms_select.max,
              p.ms_mscfk.avg(), p.ms_mscfk.max, p.ms_colorize.avg(), p.ms_colorize.max,
              p.ms_accum_map.avg(), p.ms_accum_map.max, p.ms_pub_cloud.avg(), p.ms_pub_cloud.max);
  logf(LogLevel::kInfo,
              "[PROBE-U] 更新细分ms avg|max IMU传播=%.1f|%.1f 激光先验=%.1f|%.1f "
              "视觉更新+发布约束=%.1f|%.1f 地图提交=%.1f|%.1f 位姿发布=%.1f|%.1f",
              p.ms_imu_prop.avg(), p.ms_imu_prop.max, p.ms_prior.avg(), p.ms_prior.max,
              p.ms_vis_upd.avg(), p.ms_vis_upd.max, p.ms_map_commit.avg(), p.ms_map_commit.max,
              p.ms_pub.avg(), p.ms_pub.max);
  logf(LogLevel::kInfo,
              "[PROBE-X] 规模 avg obs=%.1f tracks=%.1f lv=%.1f mscfk用=%.1f used=%.1f "
              "地图点=%.1f 上色点=%.1f | 丢弃累计 位姿空洞=%zu 等位姿超时=%zu 待定满=%zu "
              "无图像=%zu 图像队列满=%zu 着色等位姿超时=%zu 着色队列满=%zu | "
              "着色 Δ=%zu/%zu 入队Δ=%zu 延迟重试=%zu",
              p.acc_obs.avg(), p.acc_tracks.avg(), p.acc_lv.avg(), p.acc_mscfk_used.avg(),
              p.acc_used.avg(), p.acc_map_pts.avg(), p.ms_colored_pts.avg(),
              p.drop_pose_hole, p.drop_pose_timeout, p.drop_visual_full, p.drop_no_image,
              p.drop_cloud_full, p.drop_color_timeout, p.drop_color_full,
              dt(p.color_ok, p.prev_color_ok), p.color_ok, dt(p.queue_push, p.prev_queue_push),
              p.color_defer);
  logf(LogLevel::kInfo,
              "[PROBE-Q] 队列 img=%zu pend_vis=%zu pend_cloud=%zu pend_color=%zu pose=%zu "
              "imu=%zu 地图=%zu/%zu | 落盘 Δ=%zu n=%zu 快照ms=%.1f|%.1f | "
              "守卫 先验否决=%zu 速度限幅=%zu | 输入队列溢出丢弃=%zu",
              image_cache_.size(), pending_visual_.size(), pending_clouds_.size(),
              pending_color_.size(), pose_cache_.size(), imu_buffer_.size(),
              map_points_.size(), max_map_points_,
              dt(p.n_save, p.prev_save), p.n_save, p.ms_save.avg(), p.ms_save.max,
              stat_prior_veto_, stat_vel_clamp_,
              stat_input_dropped_.load(std::memory_order_relaxed));
  // 阻塞定位：排队延迟（入队→出队）按事件类型拆开，队列深度与工作线程占用时长。
  // 判读：某类延迟随窗口持续抬升 = 工作线程被前面的事件拖住；深度长期 >10 =
  // 消费速度跟不上；处理耗时接近窗口时长 = 线程满负荷。
  logf(LogLevel::kInfo,
              "[PROBE-L] 排队延迟ms img=%.1f|%.1f odom=%.1f|%.1f body=%.1f|%.1f "
              "cloud=%.1f|%.1f imu=%.1f|%.1f | 队列深度 avg=%.1f max=%.0f 出队Δ=%zu "
              "| 处理耗时ms avg=%.1f max=%.1f",
              p.q_wait_image.avg(), p.q_wait_image.max, p.q_wait_odom.avg(),
              p.q_wait_odom.max, p.q_wait_body.avg(), p.q_wait_body.max,
              p.q_wait_cloud.avg(), p.q_wait_cloud.max, p.q_wait_imu.avg(),
              p.q_wait_imu.max, p.queue_depth.avg(), p.queue_depth.max,
              dt(p.n_dequeue, p.prev_dequeue), p.ms_handler.avg(), p.ms_handler.max);
  // 彩色排空路径占用：窗口内累计耗时（占窗口的百分比）与重试规模。
  // 判读：占比 >50% 说明工作线程主要被"查不到位姿的彩色重试"吃掉，
  // 此时输入队列必然堆积，图像事件排队 => 视觉约束时差变大 => 注入过期。
  logf(LogLevel::kInfo,
              "[PROBE-D] 彩色排空 重试Δ=%zu/%zu 位姿失败=%zu | 窗口累计=%.0fms "
              "单次ms avg=%.2f max=%.1f",
              dt(p.n_retry_color, p.prev_retry_color), p.n_retry_color,
              p.n_pose_fail, p.ms_drain_color.sum, p.ms_drain_color.avg(),
              p.ms_drain_color.max);

  // 窗口量清零（累计计数与 first/last 时刻保留）
  p.image_gap.reset();
  p.odom_gap.reset();
  p.body_gap.reset();
  p.cloud_gap.reset();
  p.imu_gap.reset();
  p.image_hgap.reset();
  p.odom_hgap.reset();
  p.body_hgap.reset();
  p.process_gap.reset();
  p.wait_visual.reset();
  p.ms_front.reset();
  p.ms_constraint.reset();
  p.ms_update.reset();
  p.ms_color.reset();
  p.ms_total.reset();
  p.ms_copy_map.reset();
  p.ms_project.reset();
  p.ms_normal_eq.reset();
  p.ms_select.reset();
  p.ms_mscfk.reset();
  p.ms_colorize.reset();
  p.ms_accum_map.reset();
  p.ms_pub_cloud.reset();
  p.ms_imu_prop.reset();
  p.ms_prior.reset();
  p.ms_vis_upd.reset();
  p.ms_map_commit.reset();
  p.ms_pub.reset();
  p.ms_save.reset();
  p.acc_obs.reset();
  p.acc_tracks.reset();
  p.acc_lv.reset();
  p.acc_mscfk_used.reset();
  p.acc_used.reset();
  p.acc_map_pts.reset();
  p.ms_colored_pts.reset();
  p.q_wait_image.reset();
  p.q_wait_odom.reset();
  p.q_wait_cloud.reset();
  p.q_wait_body.reset();
  p.q_wait_imu.reset();
  p.queue_depth.reset();
  p.ms_handler.reset();
  p.ms_drain_color.reset();
}

void VisualPipeline::processVisualFrame(const cv::Mat& gray, double stamp) {
  // ==========================================================================
  // 主流程（对应 gpt §3.4）：
  //   前端 -> 生命周期 -> 关键帧/轨迹 -> LiDAR 2D-3D + MSCKF -> 统一估计器
  // ==========================================================================
  const auto t_start = std::chrono::steady_clock::now();
  ++g_probe.n_process;
  probeGap(g_probe.process_gap, g_probe.t_process, t_start);

  // 0) 位姿时基对齐：T_wl_ 是"最新一条" lio/odom，未必是本图像时刻的位姿。
  //    相机 header 戳比 LiDAR/IMU 钟晚 img_time_offset_（实测 0.1 s），
  //    真正的采集时刻在 LiDAR 钟上是 stamp + img_time_offset_。
  //    取位姿、发布 odom 都必须用这个时刻，否则整条轨迹恒落后 LIO 0.1 s。
  const double t_pose = stamp + img_time_offset_;
  // 这里必须固定用 /lio/odom（pose_cache_），不能用 poseAtStamp：后者在
  // color.use_own_odom=true 时查 own_pose_cache_，而 own_pose_cache_ 只在本函数
  // 末尾（估计器更新后）才写入，两者构成循环依赖 —— own 模式下首帧永远等不到位姿，
  // 视觉帧 0 条、own 缓存恒空，着色全部"等待位姿超时"，最终 0 点产物。
  if (!poseFromSamples(pose_cache_, t_pose, T_wl_frame_)) {
    // 插值失败 = 位姿历史里有 >0.5 s 的空洞（Super-LIO 偶发丢帧，实测 20 处）。
    // 此时退回"最新位姿"会把视觉注入锚到最多 0.7 s 前的位姿上，实测造成 0.75 m
    // 瞬跳；宁可丢这一帧（少一个采样），也不要发布错误时间的位姿。
    ++stat_drop_pose_;
    ++g_probe.drop_pose_hole;
    // 丢帧后上一帧不再是"相邻帧"，必须切断配对，避免跨位姿空洞算区间旋转。
    rot_prev_.valid = false;
    logfThrottled(LogLevel::kWarn, 2000,
                         "视觉[丢弃] 无 t_pose=%.3f 的可靠位姿（位姿空洞），跳过该帧", t_pose);
    return;
  }
  double degen_sev = 0.0;
  Eigen::Vector3d degen_u = Eigen::Vector3d::Zero();
  if (!degeneracySignature(degen_sev, degen_u) && !degen_stale_warned_) {
    degen_stale_warned_ = true;
    logf(LogLevel::kWarn,
                "未收到 %s 退化签名（超时 %.1fs）：请给 Super-LIO 开启 "
                "lio.observe.degeneracy_detect_en 且 degen_trans_only=1，"
                "否则退化感知恒不触发。",
                degen_topic_.c_str(), degen_timeout_);
  }

  // 1) Patch 前端跟踪
  const std::vector<PatchObservation> obs = frontend_->process(gray, stamp);

  // 1.1) E2 诊断落盘（默认关闭）：只记录，不参与任何估计。
  //      T_wl_frame_ 是**该图像时刻**的 LiDAR 位姿（已按 t_pose 插值），
  //      与视觉观测同一时刻，离线才能无偏地重算 r_vis 与帧间平移 tc。
  if (e2_pose_ofs_.is_open()) {
    const Eigen::Matrix3d R = T_wl_frame_.block<3, 3>(0, 0);
    const Eigen::Vector3d p = T_wl_frame_.block<3, 1>(0, 3);
    // 时间戳必须用 17 位有效数字：ROS 时间是 1.7e9 量级，setprecision(9) 只有
    // 9 位有效数字 => 小数部分全丢，时间被量化到整秒（实测 946685438 这种值），
    // 离线插值 IMU 参照会错到 0.5 s，指标完全失真。
    e2_pose_ofs_ << std::setprecision(17) << stamp << ',' << t_pose << std::setprecision(9);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        e2_pose_ofs_ << ',' << R(r, c);
      }
    }
    e2_pose_ofs_ << ',' << p.x() << ',' << p.y() << ',' << p.z() << ',' << obs.size()
                 << '\n';
    e2_pose_ofs_.flush();
  }
  if (e2_obs_ofs_.is_open()) {
    e2_obs_ofs_ << std::setprecision(17) << stamp << std::setprecision(9) << ',' << obs.size();
    for (const auto& o : obs) {
      e2_obs_ofs_ << ',' << o.feature_id << ',' << o.u << ',' << o.v;
    }
    e2_obs_ofs_ << '\n';
    e2_obs_ofs_.flush();
  }

  // 1.2) 旋转后处理（默认关）：只用本帧像素观测与 T_wl_frame_，不写任何状态。
  if (rot_refine_enable_ && rot_refiner_) {
    refineRotation(stamp, t_pose, obs);
  }

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
  const Eigen::Matrix4d T_wc = T_wl_frame_ * T_lc_;
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
  const auto t_lv0 = std::chrono::steady_clock::now();
  lidar_visual_->setMapPoints(map_points_);
  const auto t_lv1 = std::chrono::steady_clock::now();
  const auto lv_residuals = lidar_visual_->build(tracks, T_wc);
  const auto t_lv2 = std::chrono::steady_clock::now();
  Eigen::MatrixXd H_lv;
  Eigen::VectorXd b_lv;
  double cost_lv = 0.0;
  lidar_visual_->accumulate(lv_residuals, H_lv, b_lv, cost_lv);
  const auto t_lv3 = std::chrono::steady_clock::now();

  // 6) MSCKF 多帧轨迹约束
  const auto t_ct0 = std::chrono::steady_clock::now();
  const auto constraint_tracks = keyframe_manager_->constraintTracks(
      msckf_options_.min_observations, msckf_options_.min_parallax_deg);
  const auto t_mk0 = std::chrono::steady_clock::now();
  const MultiFrameNormalEquations msckf_eq = msckf_->build(constraint_tracks);
  const auto t_mk1 = std::chrono::steady_clock::now();

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
  const auto t_prop0 = std::chrono::steady_clock::now();
  if (imu_initialized_) {
    double t_prev = last_imu_stamp_;
    Eigen::Vector3d acc_prev = last_imu_acc_;
    Eigen::Vector3d gyro_prev = last_imu_gyro_;
    for (const auto& s : imu_buffer_) {
      if (s.stamp <= t_prev) {
        continue;
      }
      if (s.stamp > t_pose) {
        break;
      }
      const double imu_dt = s.stamp - t_prev;
      if (imu_dt > 0.2) {
        // 区间内出现 IMU 空洞（订阅丢包/回调被阻塞，实测 move 有一次 0.92 s）。
        // 空洞**之后的**样本不能用旧姿态积分：空洞期间真实转过的角度已经丢失，
        // 用陈旧 R 继续积分等价于凭空产生 |g|·sin(Δθ) 的水平伪加速度
        // （实测 |v| 一帧内从 0.10 泵到 1.69 m/s、路径多出 1.2 m）。
        // 因此整段剩余区间一律不积分，交给紧随其后的激光先验按该间隔重新锚定；
        // 间隔由下面的 noteSkippedInterval(gap) 登记，先验才能放宽修正上限。
        break;
      }
      estimator_->propagate(acc_prev * imu_acc_scale_, gyro_prev, imu_dt);
      t_prev = s.stamp;
      acc_prev = s.acc;
      gyro_prev = s.gyro;
    }
    if (t_pose > t_prev) {
      // 长间隔（回调被阻塞/IMU 丢包）不能用陈旧 acc 外推：dt 大 1 个量级，
      // 速度误差就大 1 个量级且再也拉不回来。超过 0.2 s 视为中断，不积分。
      const double gap = t_pose - t_prev;
      if (gap > 0.2) {
        // 不积分，但这段真实时间必须登记给先验的间隔换算：先验的修正上限按
        // 0.1 s 标定，若仍按单帧上限逐帧追，姿态会持续落后 lio/odom，残差在
        // 下一帧 propagate 里变成 |g|·sinΔθ ≈ 2.25 m/s² 的水平伪加速度。
        estimator_->noteSkippedInterval(gap);
        logfThrottled(LogLevel::kWarn, 2000,
      "IMU 中断 %.3f s（%.3f→%.3f），该段不积分，交由激光先验按该间隔锚定",
            gap, t_prev, t_pose);
      } else {
        estimator_->propagate(acc_prev * imu_acc_scale_, gyro_prev, gap);
      }
      t_prev = t_pose;
    }
    last_imu_stamp_ = t_prev;
    last_imu_acc_ = acc_prev;
    last_imu_gyro_ = gyro_prev;

    static int acc_dbg_n = 0;
    if (++acc_dbg_n % 10 == 1) {
      const auto& st = estimator_->filterState().state;
      const Eigen::Vector3d aw =
          st.R * (acc_prev * imu_acc_scale_ - st.ba) + st.g;
      logf(LogLevel::kInfo,
                  "[传播] a_w=(%.2f,%.2f,%.2f) |a_w|=%.2f ba=(%.3f,%.3f,%.3f) "
                  "bg=(%.4f,%.4f,%.4f) g=(%.2f,%.2f,%.2f) |v|=%.3f",
                  aw.x(), aw.y(), aw.z(), aw.norm(), st.ba.x(), st.ba.y(), st.ba.z(),
                  st.bg.x(), st.bg.y(), st.bg.z(), st.g.x(), st.g.y(), st.g.z(),
                  st.v.norm());
    }
  } else {
    // 一条 IMU 样本都没有（订阅失效/回调被阻塞）：没有任何惯性传播可用。
    // 若只是跳过本段，滤波器既没有名义积分也没有过程噪声，P 被先验/视觉逐帧
    // 收缩（σp→0.002、dxp=0.0000），先验增益→0 ⇒ 状态冻结在初值：
    // 批量跑批实测 move/b path 1.33 m（SL 10.44 m），终点 (−0.07,−0.15,0.40)。
    // 退化策略：把名义状态直接重置到本帧 lio/odom 的 IMU 位姿（速度取 lio 差分），
    // 视觉更新仍在其后叠加 ⇒ 输出退化为"LIO + 有界视觉修正"，而不是停住不动。
    State deg = estimator_->filterState().state;  // 保留重力/零偏/外参
    const Eigen::Matrix4d T_wi = T_wl_frame_ * T_li_;  // T_wi = T_wl · T_li
    deg.R = T_wi.block<3, 3>(0, 0);
    deg.p = T_wi.block<3, 1>(0, 3);
    Eigen::Vector3d v_lio;
    if (lioVelocity(t_pose, v_lio)) {
      deg.v = v_lio;
    }
    estimator_->resetState(deg);
    ++stat_no_imu_;
    logfThrottled(LogLevel::kWarn, 5000,
      "未收到任何 IMU 样本：按 lio/odom 位姿重置状态（退化模式，累计 %zu 帧）",
        stat_no_imu_);
  }
  g_probe.ms_imu_prop.add(probeMs(t_prop0));

  // 9) LiDAR 更新（D2）：把 lio/odom 位姿当强先验注入
  //    Trust 守卫：退化段上游解算会被带飞，lio/odom 若出现单帧大跳变，
  //    本帧撤销激光先验（判据与 Super-LIO 一致），只保留 IMU 传播结果，
  //    避免把跳变当作观测吃进滤波器。
  bool lio_trusted = true;
  if (guard_enable_ && last_trust_stamp_ > 0.0) {
    const Eigen::Vector3d dp = T_wl_frame_.block<3, 1>(0, 3) -
                               last_trust_T_wl_.block<3, 1>(0, 3);
    const Eigen::Matrix3d dR = T_wl_frame_.block<3, 3>(0, 0) *
                               last_trust_T_wl_.block<3, 3>(0, 0).transpose();
    const double dtheta = so3Log(dR).norm();
    // 阈值必须按两次视觉帧的间隔缩放：图像空洞（move 实测 2.62 s）后正常运动
    // 就能累积到 0.37 m / 29.6°，用"单帧"阈值会把最需要先验的那一帧直接否决
    // （实测 |v| 恰在该帧冲到 2.2 m/s，随后被 IMU 残差持续泵到 3 m/s 上限）。
    const double scale_dt = std::max(1.0, (t_pose - last_trust_stamp_) / 0.1);
    if (dp.norm() > guard_lio_jump_pos_ * scale_dt ||
        dtheta > guard_lio_jump_rot_ * scale_dt) {
      lio_trusted = false;
      ++stat_prior_veto_;
      logfThrottled(LogLevel::kWarn, 2000,
      "[Guard] lio/odom 单帧跳变 %.2f m / %.2f deg 超阈值（间隔 %.2f s），本帧撤销"
          "激光先验（累计 %zu 次）",
          dp.norm(), dtheta * 57.29577951308232, t_pose - last_trust_stamp_, stat_prior_veto_);
    }
  }
  if (guard_enable_) {
    last_trust_T_wl_ = T_wl_frame_;
    last_trust_stamp_ = t_pose;
  }
  const auto t_prior0 = std::chrono::steady_clock::now();
  if (lio_trusted) {
    Eigen::MatrixXd H_prior;
    Eigen::VectorXd b_prior;
    buildLidarPrior(T_wl_frame_, t_pose, H_prior, b_prior);
    estimator_->lidarUpdate(H_prior, b_prior, 0.0);
  }
  g_probe.ms_prior.add(probeMs(t_prior0));

  // 10) 视觉更新
  const auto t_vupd0 = std::chrono::steady_clock::now();
  if (estimator_options_.enable_visual) {
    VisualUpdateInput vin;
    vin.H = H_vis;
    vin.b = b_vis;
    vin.cost = cost_vis;
    vin.quality = quality;
    vin.used_features = used_features;
    vin.degen_severity = degen_sev;
    vin.degen_dir_w = degen_u;
    estimator_->visualUpdate(vin);
    publishVisualConstraint(t_pose);
  }
  g_probe.ms_vis_upd.add(probeMs(t_vupd0));

  // 10.5) 退化段速度守卫
  //  完全退化时（Super-LIO n_effect=0 → cond=1e12）激光先验在退化方向被放松，
  //  速度只剩 IMU 积分，实测 |v| 从 2.4 涨到 60 m/s。退化严重度高时给 |v| 设上限：
  //  上限 = 健康段观测到的最大速度 × guard.vel_ratio（无健康样本时用 fallback）。
  if (guard_enable_ && estimator_initialized_) {
    const double vn = estimator_->filterState().state.v.norm();
    double cap = 0.0;
    if (std::isfinite(vn) && vn > guard_vel_abs_cap_) {
      // 绝对物理上限无条件生效。旧写法把它挂在 degen_sev >= sev_threshold 分支里，
      // 而 Super-LIO 默认不发布 /lio/degeneracy ⇒ sev 恒为 0 ⇒ 整条守卫是死代码
      // （§34：|v| 一路涨到 125 m/s、"速度限幅=0"，own 位姿飘出图像，上色率归零）。
      cap = guard_vel_abs_cap_;
    } else if (degen_sev >= guard_sev_threshold_) {
      // 退化段收紧：上限 = 健康段观测到的最大速度 × guard.vel_ratio（无样本用 fallback）。
      // 视觉 spike 一旦混进 vel_ref，ratio 上限会跟着失守，故再压到绝对上限之下。
      cap = vel_ref_valid_ ? vel_ref_ * guard_vel_ratio_ : guard_vel_fallback_;
      cap = std::min(cap, guard_vel_abs_cap_);
    }
    if (cap > 0.0) {
      if (estimator_->clampSpeed(cap) > 0.0) {
        ++stat_vel_clamp_;
        logfThrottled(LogLevel::kWarn, 2000,
      "[Guard] 速度 %.2f m/s 超上限 %.2f m/s（sev=%.2f），限幅（累计 %zu 次）",
            vn, cap, degen_sev, stat_vel_clamp_);
      }
    } else {
      // 只从物理可信的速度学习参考值，超过绝对上限的样本一律不学。
      if (degen_sev < guard_sev_threshold_ && std::isfinite(vn)) {
        vel_ref_ = vel_ref_valid_ ? std::max(vel_ref_, vn) : vn;
        vel_ref_valid_ = true;
      }
    }
  }

  // 11) 延迟地图提交：状态更新后再用新位姿把本帧点云写入本地地图
  const auto t_commit0 = std::chrono::steady_clock::now();
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
  g_probe.ms_map_commit.add(probeMs(t_commit0));
  publishOdomAndPath(T_wl_updated, t_pose);
  g_probe.ms_pub.add(probeMs(t_update));

  // 12.5) 记录本锚点的融合残差（own ⊖ lio，世界系），并按信任域截断。
  // own 轨迹 = lio 轨迹 ⊕ 该残差（见 onLivOdom 里的同频落缓存）：滤波器给的是
  // 相对 lio 的修正量，残差小且慢变，比直接插值 1 Hz 的锚点位姿稳得多；
  // 残差越界（滤波器发散）时截断，own 退化为 lio，地图不会整段作废。
  if (color_use_own_odom_) {
    Eigen::Vector3d dp =
        T_wl_updated.block<3, 1>(0, 3) - T_wl_frame_.block<3, 1>(0, 3);
    const Eigen::Matrix3d dR_full = T_wl_updated.block<3, 3>(0, 0) *
                                    T_wl_frame_.block<3, 3>(0, 0).transpose();
    const double dp_raw = dp.norm();
    bool clip_p = false;
    if (own_res_max_pos_ > 0.0 && dp.norm() > own_res_max_pos_) {
      dp *= own_res_max_pos_ / dp.norm();
      clip_p = true;
    }
    Eigen::Matrix3d dR = dR_full;
    const double ang_raw = Eigen::AngleAxisd(dR_full).angle();
    bool clip_r = false;
    if (own_res_max_rot_ > 0.0) {
      Eigen::AngleAxisd aa(dR_full);
      if (std::abs(aa.angle()) > own_res_max_rot_) {
        aa.angle() = std::copysign(own_res_max_rot_, aa.angle());
        dR = aa.toRotationMatrix();
        clip_r = true;
      }
    }
    // 诊断：锚点残差 = 融合位姿相对 lio/odom 的偏移，own 口径整段地图按它平移/旋转。
    // 若 |dp| 达到分米级，1 Hz 的分段刚体位移就是"分层"的直接来源。
    Eigen::Vector3d v_lio = Eigen::Vector3d::Zero();
    const bool has_v = lioVelocity(t_pose, v_lio);
    logfThrottled(LogLevel::kInfo, 0,
                         "[own残差] |dp|=%.3f(截=%d) ang=%.2fdeg(截=%d) "
                         "|v|=%.2f v_lio=%.2f(%d)",
                         dp_raw, clip_p ? 1 : 0, ang_raw * 180.0 / M_PI,
                         clip_r ? 1 : 0, estimator_->filterState().state.v.norm(),
                         v_lio.norm(), has_v ? 1 : 0);
    own_anchor_dp_ = dp;
    own_anchor_dR_ = dR;
    own_anchor_t_ = t_pose;
    pushOwnPoseSample(t_pose, T_wl_frame_);
  }

  if (!body_points_.empty() && world_cloud_valid_) {
    runColorizer();
  }
  const auto t_end = std::chrono::steady_clock::now();
  auto ms = [](const std::chrono::steady_clock::time_point& a,
               const std::chrono::steady_clock::time_point& b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };

  const UnifiedEstimator::VisualDiagnostics& vd = estimator_->visualDiagnostics();
  const auto tp = keyframe_manager_->trackPoolStats();
  logfThrottled(
      LogLevel::kInfo, 1000,
      "视觉帧: obs=%zu tracks=%zu lv=%zu mscfk=%d used=%d quality=%.2f | "
      "退化 sev=%.2f dir=(%+.2f %+.2f %+.2f) | "
      "注入 λ=%.2e cap=%.2e scale=%.4f dxθ=%.4frad dxp=%.4fm σp=%.3fm σθ=%.4frad |v|=%.2fm/s | "
      "耗时 ms: 前端=%.1f 约束=%.1f 更新=%.1f 彩色=%.1f 合计=%.1f | "
      "约束细分 ms: 拷贝地图=%.1f 投影建栅格=%.1f 法方程=%.1f 选轨=%.1f mscfk=%.1f | "
      "轨迹池=%zu 观测=%zu 最长=%zu 配对=%zu | "
      "地图点数=%zu | 守卫: 先验否决=%zu 速度限幅=%zu",
      obs.size(), constraint_tracks.size(), lv_residuals.size(), msckf_eq.used_features,
      used_features, quality, degen_sev, degen_u.x(), degen_u.y(), degen_u.z(),
      vd.info_lambda, vd.cap_lambda, vd.scale, vd.dx_rot, vd.dx_pos,
      vd.sigma_pos,
      vd.sigma_rot, vd.vel_norm,
      ms(t_start, t_front), ms(t_front, t_vis), ms(t_vis, t_update),
      ms(t_update, t_end), ms(t_start, t_end),
      ms(t_lv0, t_lv1), ms(t_lv1, t_lv2), ms(t_lv2, t_lv3), ms(t_ct0, t_mk0),
      ms(t_mk0, t_mk1), tp.num_tracks, tp.total_obs, tp.max_obs, tp.pair_work,
      map_points_.size(), stat_prior_veto_, stat_vel_clamp_);

  // ---- 探针：把本帧分段/规模计入窗口（2 s 汇总打印一次）----
  g_probe.ms_front.add(ms(t_start, t_front));
  g_probe.ms_constraint.add(ms(t_front, t_vis));
  g_probe.ms_update.add(ms(t_vis, t_update));
  g_probe.ms_color.add(ms(t_update, t_end));
  g_probe.ms_total.add(ms(t_start, t_end));
  g_probe.ms_copy_map.add(ms(t_lv0, t_lv1));
  g_probe.ms_project.add(ms(t_lv1, t_lv2));
  g_probe.ms_normal_eq.add(ms(t_lv2, t_lv3));
  g_probe.ms_select.add(ms(t_ct0, t_mk0));
  g_probe.ms_mscfk.add(ms(t_mk0, t_mk1));
  g_probe.acc_obs.add(static_cast<double>(obs.size()));
  g_probe.acc_tracks.add(static_cast<double>(constraint_tracks.size()));
  g_probe.acc_lv.add(static_cast<double>(lv_residuals.size()));
  g_probe.acc_mscfk_used.add(static_cast<double>(msckf_eq.used_features));
  g_probe.acc_used.add(static_cast<double>(used_features));
  g_probe.acc_map_pts.add(static_cast<double>(map_points_.size()));
  probeMaybeReport();
}

void VisualPipeline::publishVisualConstraint(double stamp) {
  if (!bridge_) {
    return;
  }
  const auto& applied = estimator_->appliedVisual();
  if (!applied.valid || applied.H.rows() != 6 || applied.H.cols() != 6 ||
      applied.b.size() != 6) {
    return;
  }
  if (!applied.H.allFinite() || !applied.b.allFinite()) {
    return;
  }

  // 白化：H = Σ w Jᵀ R⁻¹ J 为信息矩阵，令 H = L Lᵀ，则 J_w = Lᵀ、观测噪声 R_w = I。
  Eigen::LLT<Eigen::MatrixXd> llt(applied.H);
  if (llt.info() != Eigen::Success) {
    return;
  }
  const Eigen::MatrixXd L = llt.matrixL();  // 下三角
  const Eigen::MatrixXd JwT = L;            // J_wᵀ
  // B 的 b = JᵀR⁻¹(预测-观测)，本滤波器 Update 的 r 约定是 (观测-预测)，故取负。
  const Eigen::VectorXd r6 = -(L.triangularView<Eigen::Lower>().solve(applied.b));
  if (!r6.allFinite()) {
    return;
  }

  // 世界系左扰动 ξ -> ESKF 机体系右扰动 δ：ξ = R_wi · δ，旋转块右乘当前姿态。
  const Eigen::Matrix3d R_wi = estimator_->filterState().state.R;
  Eigen::MatrixXd H18 = Eigen::MatrixXd::Zero(6, 18);
  H18.block<3, 3>(0, 0) = JwT.block<3, 3>(0, 0) * R_wi;
  H18.block<3, 3>(0, 3) = JwT.block<3, 3>(0, 3);
  H18.block<3, 3>(3, 0) = JwT.block<3, 3>(3, 0) * R_wi;
  H18.block<3, 3>(3, 3) = JwT.block<3, 3>(3, 3);

  VisualObs obs;
  obs.stamp = stamp;
  obs.H = std::move(H18);
  obs.r = r6;
  obs.R = Eigen::MatrixXd::Identity(6, 6);
  bridge_->push(std::move(obs));
}

void VisualPipeline::runColorizer() {
  // 与方案 A 相同：机体系点云已是 LiDAR 系的去畸变点，逐点时间都用扫描帧头。
  // 注意 colorizer 的 Options.T_cl 实现语义为 T_cw（作用于世界系点），逐帧重建 options。
  //
  // 时间基准（与 A 一致，已用离线重放实验确认）：
  //   t_cloud = body_stamp_  → 点云世界系变换
  //   t_pair  = 图像 header.stamp → 选图（实测 |t_pair-t_cloud| < 7 ms）
  //   t_img   = t_pair + img_time_offset_ → 取相机位姿（本数据集最优 0.1 s）
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
    ++g_probe.queue_push;
    while (pending_clouds_.size() > kMaxPendingClouds) {
      pending_clouds_.pop_front();
      ++stat_drop_no_image_;
      ++g_probe.drop_cloud_full;
      logf(LogLevel::kWarn, "彩色[丢弃] 等图像队列溢出，丢弃最早一帧");
    }
  }

  drainPendingClouds();
  drainPendingColorization();
}

void VisualPipeline::drainPendingClouds() {
  const auto now = std::chrono::steady_clock::now();
  for (auto it = pending_clouds_.begin(); it != pending_clouds_.end();) {
    // 显式配对：按 header.stamp 挑最接近 t_cloud 的一帧图像。
    const colorizer::TimedImage* chosen = nullptr;
    double best_dt = std::numeric_limits<double>::max();
    for (const auto& image : image_cache_) {
      if (!image.valid || image.image.empty()) {
        continue;
      }
      // 官方口径配对：img_time_buffer 存 header + img_time_offset，官方按"该值最接近
      // LIO 时刻（= t_cloud）"取图，即图像真实采集时刻与点云同刻。
      // 所以比较 |header + img_time_offset - t_cloud|，而不是 |header - t_cloud|。
      const double dt = std::abs(image.arrival_stamp + img_time_offset_ - it->t_cloud);
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
        ++g_probe.drop_color_full;
        logf(LogLevel::kWarn, "彩色[丢弃] 待处理队列溢出，丢弃最早一帧");
      }
      it = pending_clouds_.erase(it);
      continue;
    }
    const double waited = std::chrono::duration<double>(now - it->queued_at).count();
    if (waited > kPendingCloudTimeoutSec) {
      ++stat_drop_no_image_;
      ++g_probe.drop_no_image;
      logfThrottled(LogLevel::kWarn, 5000,
                           "彩色[丢弃] 等待图像超时: t_cloud=%.4f 最近差=%.4f 阈值=%.4f 已等 %.2fs",
                           it->t_cloud, best_dt, colorizer_options_.sync.max_offset, waited);
      it = pending_clouds_.erase(it);
      continue;
    }
    ++it;
  }
}

void VisualPipeline::drainPendingVisualFrames() {
  if (pending_visual_.empty()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  // 本次调用内不会有新 odom 入队，最新位姿时刻取一次即可。
  const double newest_pose_stamp = pose_cache_.empty() ? -1e18 : pose_cache_.back().stamp;
  for (auto it = pending_visual_.begin(); it != pending_visual_.end();) {
    const double t_pose = it->stamp + img_time_offset_;
    if (newest_pose_stamp >= t_pose) {
      // 先取出副本再 erase：processVisualFrame 可能改动容器，迭代器会失效。
      const cv::Mat gray = it->gray;
      const double stamp = it->stamp;
      g_probe.wait_visual.add(std::chrono::duration<double, std::milli>(now - it->queued_at).count());
      it = pending_visual_.erase(it);
      processVisualFrame(gray, stamp);
      continue;
    }
    const double waited = std::chrono::duration<double>(now - it->queued_at).count();
    if (waited > kPendingVisualTimeoutSec) {
      ++stat_drop_pose_;
      ++g_probe.drop_pose_timeout;
      logfThrottled(LogLevel::kWarn, 5000,
                           "视觉[丢弃] 等待位姿超时: stamp=%.4f t_pose=%.4f 已等 %.2fs",
                           it->stamp, t_pose, waited);
      it = pending_visual_.erase(it);
      continue;
    }
    ++it;
  }
}

void VisualPipeline::drainPendingColorization() {
  if (pending_color_.empty()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  // 节流：调用方是 odom_topic（默认 lio/odom，10 Hz）以及云/图像回调，位姿缓存只在
  // odom 上推进，图像也只有 10 Hz —— 没必要每个事件都重试一遍（旧实现在 200 Hz odom
  // 下积压时会以 ≈740 次/s 的无解重试吃满工作线程，见 [PROBE-D]）。
  if (last_color_drain_.time_since_epoch().count() != 0 &&
      now - last_color_drain_ <
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(kColorDrainMinIntervalSec))) {
    return;
  }
  last_color_drain_ = now;

  const auto t_begin = std::chrono::steady_clock::now();
  // 与 camPoseAtStamp 取同一个缓存，判断"位姿是否已覆盖 t_img"才准。
  const auto& cache = cam_pose_cache_.empty()
                          ? (color_use_own_odom_ ? own_pose_cache_ : pose_cache_)
                          : cam_pose_cache_;
  const double newest = cache.empty() ? -std::numeric_limits<double>::max()
                                      : cache.back().stamp;
  for (auto it = pending_color_.begin(); it != pending_color_.end();) {
    // 位姿还没覆盖 t_img：此刻重试必然失败，只做超时清理，不做无解重试。
    if (it->t_img > newest) {
      const double waited = std::chrono::duration<double>(now - it->queued_at).count();
      if (waited > kPendingColorTimeoutSec) {
        ++stat_drop_pose_;
        ++g_probe.drop_color_timeout;
        // 探针：把"缺位姿"的判定依据一并打出来——t_img 比最新位姿落后多少、缓存里
        // 还有多少采样。
        logfThrottled(LogLevel::kWarn, 5000,
                             "彩色[丢弃] 等待位姿超时: t_cloud=%.4f t_img=%.4f 已等 %.2fs "
                             "最新位姿=%.4f 落后=%.3fs cache=%zu",
                             it->t_cloud, it->t_img, waited, newest, newest - it->t_img,
                             cache.size());
        it = pending_color_.erase(it);
        continue;
      }
      ++stat_deferred_;
      ++g_probe.color_defer;
      ++it;
      continue;
    }

    ++g_probe.n_retry_color;
    if (colorizeFrame(*it)) {
      ++g_probe.color_ok;
      it = pending_color_.erase(it);
      continue;
    }
    ++g_probe.color_defer;
    ++g_probe.n_pose_fail;
    const double waited = std::chrono::duration<double>(now - it->queued_at).count();
    if (waited > kPendingColorTimeoutSec) {
      ++stat_drop_pose_;
      ++g_probe.drop_color_timeout;
      logfThrottled(LogLevel::kWarn, 5000,
                           "彩色[丢弃] 着色失败超时: t_cloud=%.4f t_img=%.4f 已等 %.2fs "
                           "最新位姿=%.4f cache=%zu",
                           it->t_cloud, it->t_img, waited, newest, cache.size());
      it = pending_color_.erase(it);
      continue;
    }
    ++stat_deferred_;
    ++it;
  }
  g_probe.ms_drain_color.add(probeMs(t_begin));
}

bool VisualPipeline::colorizeFrame(const PendingColorFrame& frame) {
  const double t_cloud = frame.t_cloud;
  const double t_img = frame.t_img;

  Eigen::Matrix4d T_wl_img = Eigen::Matrix4d::Identity();
  if (!camPoseAtStamp(t_img, T_wl_img)) {
    return false;  // 未来时刻的位姿还没发布，稍后重试
  }

  colorizer::ColorizerPipeline::Options options = colorizer_options_;
  const Eigen::Matrix4d T_wc_img = T_wl_img * T_lc_;
  options.T_cl = T_wc_img.inverse();

  // 只把选中的图像交给流水线，时间戳按"配对时刻"给出（流水线只用它判断同步窗口）。
  colorizer::TimedImage image;
  image.image = frame.image;
  // 必须填"图像真实采集时刻"t_img（≈ t_cloud），而不是 header（t_pair = t_img - offset）：
  // 流水线用它和 point_stamps（全部 = t_cloud）做同步窗口检查（0.05 s），
  // 填 header 会差 0.1 s 被判不同步，整帧返回空（"0/0 点上色"）。
  image.exposure_mid_stamp = frame.t_img;
  image.arrival_stamp = frame.t_img;
  image.valid = true;
  std::vector<colorizer::TimedImage> images_for_frame{std::move(image)};
  std::vector<double> point_stamps(frame.points_lidar.size(), t_cloud);

  const colorizer::ColorizerPipeline pipeline(options);
  const auto t_pipe0 = std::chrono::steady_clock::now();
  const std::vector<colorizer::ColoredPoint> colored =
      pipeline.process(frame.points_lidar, point_stamps, images_for_frame, pose_cache_);
  g_probe.ms_colorize.add(probeMs(t_pipe0));
  if (colored.empty()) {
    logfThrottled(LogLevel::kWarn, 5000,
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
  logfThrottled(LogLevel::kInfo, 1000,
                       "彩色: t_cloud=%.4f t_pair=%.4f t_img=%.4f(offset=%+.3f) "
                       "dt_pair_cloud=%+.4f dt_img_cloud=%+.4f %zu/%zu 上色 帧#%zu",
                       t_cloud, frame.t_pair, t_img, img_time_offset_,
                       frame.t_pair - t_cloud, t_img - t_cloud, colored_count,
                       colored.size(), stat_colored_frames_);

  g_probe.ms_colored_pts.add(static_cast<double>(colored_count));
  const auto t_acc0 = std::chrono::steady_clock::now();
  accumulateColoredMap(colored, t_cloud);
  g_probe.ms_accum_map.add(probeMs(t_acc0));
  const auto t_pub0 = std::chrono::steady_clock::now();
  publishColoredCloud(colored, t_cloud);
  g_probe.ms_pub_cloud.add(probeMs(t_pub0));
  return true;
}

void VisualPipeline::pushOwnPoseSample(double stamp, const Eigen::Matrix4d& T_wl_lio) {
  colorizer::PoseSample own;
  own.stamp = stamp;
  own.T_wl = Eigen::Matrix4d::Identity();
  own.T_wl.block<3, 3>(0, 0) = own_anchor_dR_ * T_wl_lio.block<3, 3>(0, 0);
  own.T_wl.block<3, 1>(0, 3) = T_wl_lio.block<3, 1>(0, 3) + own_anchor_dp_;
  // 锚点采样可能落在两次 odom 采样之间，按时刻有序插入，别破坏插值前提。
  auto pos = std::lower_bound(
      own_pose_cache_.begin(), own_pose_cache_.end(), stamp,
      [](const colorizer::PoseSample& s, double t) { return s.stamp < t; });
  if (pos != own_pose_cache_.end() && std::abs(pos->stamp - stamp) < 1e-6) {
    pos->T_wl = own.T_wl;  // 同一时刻：锚点的精修结果覆盖零阶保持值
  } else {
    own_pose_cache_.insert(pos, own);
  }
  if (own_pose_cache_.size() > max_pose_samples_) {
    own_pose_cache_.erase(own_pose_cache_.begin(),
                          own_pose_cache_.begin() +
                              (own_pose_cache_.size() - max_pose_samples_));
  }
}

bool VisualPipeline::poseAtStamp(double stamp, Eigen::Matrix4d& T_wl) const {
  // 查询时刻晚于最新采样时必须拒绝：插值器会直接用最后一个采样顶替，
  // 那等于把 t_img 的位姿退化成 t_cloud 的位姿，offset 就白加了。
  // 轨迹源由 color.use_own_odom 决定：默认 /lio/odom；true 时用统一估计器修正位姿。
  return poseFromSamples(color_use_own_odom_ ? own_pose_cache_ : pose_cache_, stamp, T_wl);
}

bool VisualPipeline::camPoseAtStamp(double stamp, Eigen::Matrix4d& T_wl) const {
  if (cam_pose_cache_.empty()) {
    return poseAtStamp(stamp, T_wl);  // 未配置独立相机位姿源：沿用旧行为
  }
  return poseFromSamples(cam_pose_cache_, stamp, T_wl);
}

void VisualPipeline::buildLidarPrior(const Eigen::Matrix4d& T_wl,
                                  double t_pose,
                                  Eigen::MatrixXd& H,
                                  Eigen::VectorXd& b) const {
  // D2：把 lio/odom 当作 LiDAR 观测，在 30 维状态的 R/p（以及 v）块上注入强先验。
  // 法方程（信息形式）：b = H·(x_cur ⊖ x_prior)，配合 dx = -P·b 会把状态拉回先验。
  H = Eigen::MatrixXd::Zero(StateLayout::kSize, StateLayout::kSize);
  b = Eigen::VectorXd::Zero(StateLayout::kSize);
  if (!estimator_initialized_) {
    return;
  }

  const double sr = std::max(1e-6, prior_rot_sigma_);
  const double sp = std::max(1e-6, prior_pos_sigma_);
  const Eigen::Matrix3d Hr = Eigen::Matrix3d::Identity() / (sr * sr);
  Eigen::Matrix3d Hp = Eigen::Matrix3d::Identity() / (sp * sp);

  // 退化感知：沿退化方向把位置先验松开到 σ_degen，垂直方向收紧到 σ_healthy。
  // 没有这一步，LiDAR 退化时"每帧把状态拉回漂移中的 lio/odom"会抵消视觉修正，
  // 视觉最多把漂移速率减半（旧上限 50/50），无法真正接管。
  double sev = 0.0;
  Eigen::Vector3d u = Eigen::Vector3d::Zero();
  (void)degeneracySignature(sev, u);
  if (prior_directional_en_) {
    // 沿退化方向松开、垂直方向收紧，再按 sev 与各向同性基线插值（A/B 共用实现）。
    Hp = superlivo_common::directionalPositionInformation(
        Hp, sev, u, prior_pos_sigma_healthy_, prior_pos_sigma_degen_);
  }

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

  // 速度观测：视觉是 pose_only（dx 的速度块恒为 0），先验又只约束 R/p，
  // 状态速度此前完全不可观 —— IMU 传播留下的加速度残差没人纠，|v| 自由随机游走
  // （实测 130 s 到 120 m/s，own 位姿飘出图像 → 上色率 0、地图只剩前 57 s）。
  // 用 lio/odom 的差分速度补上这一维观测（刚体上 IMU 原点的位置差分即其速度）。
  if (prior_vel_sigma_ > 0.0) {
    Eigen::Vector3d v_lio = Eigen::Vector3d::Zero();
    const bool vel_ok = lioVelocity(t_pose, v_lio);
    if (vel_ok) {
      const double sv = std::max(1e-3, prior_vel_sigma_);
      const Eigen::Matrix3d Hv = Eigen::Matrix3d::Identity() / (sv * sv);
      H.block<3, 3>(StateLayout::kVel, StateLayout::kVel) = Hv;
      b.segment<3>(StateLayout::kVel) = Hv * (s.v - v_lio);
    }
    static int vel_dbg_n = 0;
    if (++vel_dbg_n % 10 == 1) {
      logf(LogLevel::kInfo,
                  "[先验] vel_ok=%d |v_lio|=%.3f |v_cur|=%.3f t_pose=%.3f cache=%zu",
                  vel_ok ? 1 : 0, v_lio.norm(), s.v.norm(), t_pose, pose_cache_.size());
    }
  }
}

bool VisualPipeline::lioVelocity(double t_pose, Eigen::Vector3d& v_lio) const {
  const double kBaseline = 0.10;  // 差分基线（s）：太短被位姿噪声主导，太长切弯
  const colorizer::PoseSample* a = nullptr;
  const colorizer::PoseSample* b = nullptr;
  for (const auto& s : pose_cache_) {
    if (s.stamp <= t_pose - kBaseline) {
      a = &s;
    }
    if (s.stamp <= t_pose) {
      b = &s;
    }
  }
  if (a == nullptr || b == nullptr || b->stamp <= a->stamp) {
    return false;
  }
  const double dt = b->stamp - a->stamp;
  if (dt < 0.05 || dt > 0.30) {
    return false;  // 位姿空洞：差分速度不可信
  }
  const Eigen::Vector3d p_a = (a->T_wl * T_li_).block<3, 1>(0, 3);
  const Eigen::Vector3d p_b = (b->T_wl * T_li_).block<3, 1>(0, 3);
  v_lio = (p_b - p_a) / dt;
  return v_lio.allFinite() && v_lio.norm() < 10.0;
}

std::vector<FeatureTrack> VisualPipeline::assembleTracks(
    const std::vector<PatchObservation>& obs, double stamp) const {
  // 本帧每条观测组装成"单观测轨迹"，跨帧合并交给 KeyframeManager::updateTracks。
  const Eigen::Matrix4d T_wc = T_wl_frame_ * T_lc_;
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

void VisualPipeline::publishColoredCloud(const std::vector<colorizer::ColoredPoint>& points,
                                         double stamp) {
  if (timing_en_ && img_enqueue_wall_.time_since_epoch().count() != 0) {
    latency_timer_.Record(
        "[Img->VisCloud]",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  img_enqueue_wall_)
            .count());
  }
  // 点表达在 LiDAR 系；ROS 消息的组装与发布由 app 完成。
  if (colored_cb_) {
    colored_cb_(points, stamp);
  }
}

void VisualPipeline::accumulateColoredMap(const std::vector<colorizer::ColoredPoint>& points,
                                       double stamp) {
  if (points.empty()) {
    return;
  }
  if (colored_map_.size() >= max_colored_map_points_) {
    if (!pcd_capacity_warned_) {
      pcd_capacity_warned_ = true;
      logf(LogLevel::kWarn, "彩色地图点数达到上限 %zu，后续帧不再累积",
                  max_colored_map_points_);
    }
    return;
  }

  // 与彩色流水线一致：lio/body/cloud 已去畸变，逐点时间都取扫描帧头，
  // 因此在 stamp 时刻取一次 T_wl，把点从 LiDAR 系搬到世界系。
  // 禁止回退到最新位姿：那会把点云搬到错误时刻的世界系，产生跨帧错位的"重影"。
  Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
  if (!poseAtStamp(stamp, T_wl)) {
    logfThrottled(LogLevel::kWarn, 5000,
                         "彩色[丢弃] 累积位姿插值失败: %.4f 缓存=%zu", stamp,
                         pose_cache_.size());
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

  // 分片落盘：每 N 个真正上过色的帧把一个增量段交给落盘线程（参照 Super-LIO
  // lio.map.save_interval）。段内只含本时间窗新增点，写盘不阻塞工作线程。
  if (save_interval_frames_ > 0 && added > 0 &&
      ++frames_since_shard_ >= save_interval_frames_) {
    frames_since_shard_ = 0;
    requestColoredMapSave();
  }
}

void VisualPipeline::saveColoredMap() {
  if (colored_map_.empty() || colored_map_.size() == pcd_saved_points_) {
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories(pcd_save_dir_, ec);
  if (ec) {
    logfThrottled(LogLevel::kWarn, 5000,
                         "无法创建 PCD 输出目录 %s: %s", pcd_save_dir_.c_str(),
                         ec.message().c_str());
    return;
  }

  const std::string path =
      (std::filesystem::path(pcd_save_dir_) / "super_livo_visual_b_colored.pcd").string();

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
    logf(LogLevel::kInfo, "彩色地图已保存: %s（%zu 点，世界系%s）", path.c_str(),
                colored_map_.size(),
                median_map_.enabled() ? "，时间中位数回填色" : "");
  } else {
    logfThrottled(LogLevel::kWarn, 5000,
                         "保存 PCD 失败: %s", path.c_str());
  }
}

void VisualPipeline::publishOdomAndPath(const Eigen::Matrix4d& T_wl, double stamp) {
  if (timing_en_ && img_enqueue_wall_.time_since_epoch().count() != 0) {
    latency_timer_.Record(
        "[Img->VisOdom]",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  img_enqueue_wall_)
            .count());
  }
  // odom 与 path 的发布（含 path 累积）在 app 侧完成，这里只输出位姿。
  if (odom_cb_) {
    odom_cb_(T_wl, stamp);
  }
}

cv::Mat VisualPipeline::rectifyBgr(const cv::Mat& bgr) const {
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

Eigen::Matrix4d VisualPipeline::makeTlcFromRclPcl() const {
  // Rcl/Pcl 表示 p_cam = Rcl·p_lidar + Pcl（LiDAR -> Camera），
  // 本节点内部用 T_lc（Camera -> LiDAR），因此需求逆（同方案 A 的说明）。
  const auto rcl = params_.getDoubleArray(
      "Rcl", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
  const auto pcl = params_.getDoubleArray(
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

Eigen::Matrix3d VisualPipeline::whiteningFromCov(const Eigen::Matrix3d& cov) const {
  // Cw = Σ^{-1/2}：特征分解后逐轴取 1/σ。σ 下限 1e-4 rad（≈0.0057°）只防数值爆掉，
  // 实测逐区间主轴 σ 在 0.02–0.06°（§36.12），离该下限还有一个量级。
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  if (es.info() != Eigen::Success) {
    return RotationRefiner::isotropicWhitening(0.05);
  }
  const double floor_var = 1e-4 * 1e-4;
  const Eigen::Vector3d var = es.eigenvalues().cwiseMax(floor_var);
  return es.eigenvectors() * var.cwiseInverse().cwiseSqrt().asDiagonal() *
         es.eigenvectors().transpose();
}

void VisualPipeline::refineRotation(double stamp, double t_pose,
                                 const std::vector<PatchObservation>& obs) {
  // 输入全部来自"该图像时刻"的量：obs 是当前帧像素，T_wl_frame_ 是 t_pose 处
  // 插值的 LIO 位姿；上一帧的观测/位姿在 rot_prev_ 里。视觉—惯导的时间错位
  // 已在拿位姿时处理（img_time_offset_），这里不再做时间补偿。
  const Eigen::Matrix3d R_wb2 = T_wl_frame_.block<3, 3>(0, 0);
  const Eigen::Vector3d p_wb2 = T_wl_frame_.block<3, 1>(0, 3);

  bool usable = false;
  Eigen::Vector3d r_vis_body = Eigen::Vector3d::Zero();
  Eigen::Matrix3d Cw = RotationRefiner::isotropicWhitening(0.05);
  RotationEstimator::Result est;

  const double gap = rot_prev_.valid ? (t_pose - rot_prev_.t_pose) : 0.0;
  if (rot_prev_.valid && gap > 0.0 && gap < 0.35) {
    std::vector<Eigen::Vector2d> a;
    std::vector<Eigen::Vector2d> b;
    a.reserve(obs.size());
    b.reserve(obs.size());
    for (const auto& o : obs) {
      const auto it = rot_prev_.obs.find(o.feature_id);
      if (it == rot_prev_.obs.end()) {
        continue;
      }
      a.emplace_back(it->second);
      b.emplace_back(o.u, o.v);
    }
    // LIO 相机系帧间运动：R_wc = R_wb·R_bc，C_w = p_wb + R_wb·t_bc（t_bc 为相机
    // 原点在 body 系的位置，即 T_lc 的平移）。
    const Eigen::Matrix3d R_wb1 = rot_prev_.T_wl.block<3, 3>(0, 0);
    const Eigen::Vector3d p_wb1 = rot_prev_.T_wl.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_wc1 = R_wb1 * R_bc_;
    const Eigen::Matrix3d R_wc2 = R_wb2 * R_bc_;
    const Eigen::Vector3d C1 = p_wb1 + R_wb1 * t_bc_;
    const Eigen::Vector3d C2 = p_wb2 + R_wb2 * t_bc_;
    const Eigen::Matrix3d R_c2c1 = R_wc2.transpose() * R_wc1;  // 相机 2 ← 相机 1
    const Eigen::Vector3d t_c = R_wc2.transpose() * (C1 - C2);

    est = rot_estimator_->estimate(a, b, t_c);
    if (est.usable) {
      // 相机系 -> body 系：R_b2b1 = R_bc·R_c2c1·R_bcᵀ，协方差同式旋转。
      const Eigen::Matrix3d R_b2b1 = R_bc_ * est.R * R_bc_.transpose();
      r_vis_body = so3Log(R_b2b1);
      const Eigen::Matrix3d Sigma_cam =
          (rot_noise_mode_ == "cov") ? est.cov : visualNoiseW3();
      Cw = whiteningFromCov(R_bc_ * Sigma_cam * R_bc_.transpose());
      usable = true;
    }
  } else if (rot_prev_.valid) {
    // 时间不连续（丢帧/乱序）：重置滑窗，绝不把跨空洞的区间当相邻帧因子。
    rot_refiner_->reset();
  }

  const Eigen::Vector3d u =
      rot_refiner_->addFrame(R_wb2, r_vis_body, Cw, usable);
  ++stat_rot_frames_;
  if (usable) {
    ++stat_rot_usable_;
  }

  // 精修 odom：位置沿用 LIO，旋转换成 Exp(u)·R_lio（只做旋转后处理的承诺）。
  {
    Eigen::Matrix4d T_refined = Eigen::Matrix4d::Identity();
    T_refined.block<3, 3>(0, 0) = rot_refiner_->correctedRotation();
    T_refined.block<3, 1>(0, 3) = p_wb2;
    if (rot_odom_cb_) {
      rot_odom_cb_(T_refined, t_pose);
    }
  }

  if (rot_refine_ofs_.is_open()) {
    rot_refine_ofs_ << std::setprecision(17) << stamp << ',' << t_pose
                    << std::setprecision(9) << ',' << est.n_pairs << ',' << est.n_inl
                    << ',' << est.t_norm << ',' << est.th_deg << ',' << r_vis_body.x()
                    << ',' << r_vis_body.y() << ',' << r_vis_body.z() << ',' << u.x()
                    << ',' << u.y() << ',' << u.z() << ',' << est.sigma_rad.x() << ','
                    << est.sigma_rad.y() << ',' << est.sigma_rad.z() << ','
                    << est.px_med << ',' << est.px_p90 << ',' << est.d_med << ','
                    << static_cast<int>(usable) << '\n';
    rot_refine_ofs_.flush();
  }

  rot_prev_.valid = true;
  rot_prev_.t_pose = t_pose;
  rot_prev_.T_wl = T_wl_frame_;
  rot_prev_.obs.clear();
  for (const auto& o : obs) {
    rot_prev_.obs.emplace(o.feature_id, Eigen::Vector2d(o.u, o.v));
  }
}

Eigen::Matrix4d VisualPipeline::applyLeftPerturbation(const Eigen::Matrix4d& T,
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

// 可执行入口在 src/app/visual_b_app.cpp：本文件只承载 ROS 无关的流水线实现，
// 不定义 main，避免与入口重复。
