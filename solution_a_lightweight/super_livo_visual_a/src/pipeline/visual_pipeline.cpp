#include "super_livo_visual_a/pipeline/visual_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

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

// ============================================================================
// 构造 / 析构
// ============================================================================

VisualPipeline::VisualPipeline(const VisualPipelineOptions& options) : opt_(options) {
  intrinsics_ = opt_.intrinsics;
  distortion_ = opt_.distortion;
  T_lc_ = makeTlcFromRclPcl(opt_.Rcl, opt_.Pcl);
  img_time_offset_.store(opt_.img_time_offset);
  visual_decimate_ = std::max(1, opt_.visual_decimate);
  max_queue_latency_ = opt_.max_queue_latency;

  // ---- 前端 ----
  frontend_ = std::make_unique<KltFrontend>(opt_.klt);

  // ---- 深度关联（内参由相机内参统一注入） ----
  LidarDepthAssociation::Options depth = opt_.depth;
  depth.fx = intrinsics_.fx;
  depth.fy = intrinsics_.fy;
  depth.cx = intrinsics_.cx;
  depth.cy = intrinsics_.cy;
  depth.image_width = intrinsics_.width;
  depth.image_height = intrinsics_.height;
  depth_assoc_ = std::make_unique<LidarDepthAssociation>(depth);

  // ---- 重投影 ----
  ReprojectionConstraint::Options reproj = opt_.reproj;
  reproj.fx = intrinsics_.fx;
  reproj.fy = intrinsics_.fy;
  reproj.cx = intrinsics_.cx;
  reproj.cy = intrinsics_.cy;
  reproj_ = std::make_unique<ReprojectionConstraint>(reproj);

  // ---- 门控 / 更新 ----
  gate_ = std::make_unique<DegeneracyGate>(opt_.gate);
  visual_update_ = std::make_unique<VisualUpdate>(opt_.update);

  // ---- 彩色流水线（每帧在 runColorizer 里按当前相机位姿重建） ----
  colorizer_options_ = opt_.colorizer;
  colorizer_options_.intrinsics = intrinsics_;
  colorizer_options_.T_cl = Eigen::Matrix4d::Identity();

  median_map_.set_voxel_size(opt_.median_voxel_size);
  save_interval_frames_ = opt_.pcd_save_interval_frames;
  shard_interval_sec_ = opt_.pcd_shard_interval_sec;
  shard_mode_ = save_interval_frames_ > 0 || shard_interval_sec_ > 0.0;
  last_shard_time_ = std::chrono::steady_clock::now();

  // ---- 轨迹 / 图像落盘（目录解析与文件打开，参照 FAST-LIVO2 evo/image_save） ----
  if (opt_.pose_output_en) {
    const std::string dir =
        opt_.pose_save_dir.empty() ? opt_.pcd_save_dir : opt_.pose_save_dir;
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    std::string seq = opt_.pose_seq_name.empty() ? "scene_move" : opt_.pose_seq_name;
    if (seq.size() < 4 || seq.substr(seq.size() - 4) != ".txt") {
      seq += ".txt";
    }
    const std::string path = (std::filesystem::path(dir) / seq).string();
    if (!ec) {
      pose_file_.open(path, std::ios::out | std::ios::trunc);
    }
    if (pose_file_.is_open()) {
      log(LogLevel::kInfo, "轨迹输出: " + path);
    } else {
      log(LogLevel::kWarn, "无法创建轨迹输出文件 " + path + ": " + ec.message());
    }
  }
  if (opt_.img_save_en) {
    img_save_dir_resolved_ = opt_.img_save_dir.empty()
                                 ? (std::filesystem::path(opt_.pcd_save_dir) / "image").string()
                                 : opt_.img_save_dir;
    std::error_code ec;
    std::filesystem::create_directories(img_save_dir_resolved_, ec);
    if (ec) {
      log(LogLevel::kWarn,
          "无法创建图像输出目录 " + img_save_dir_resolved_ + ": " + ec.message());
      img_save_dir_resolved_.clear();
    } else {
      log(LogLevel::kInfo, "图像输出: " + img_save_dir_resolved_);
    }
  }

  colored_map_.reserve(1024);
  startWorkerThreads();
  log(LogLevel::kInfo, "VisualPipeline 初始化完成（进程内流水线，无 DDS）");
}

VisualPipeline::~VisualPipeline() { shutdownAndSave(); }

// ============================================================================
// 公共接口
// ============================================================================

void VisualPipeline::setColoredCloudCallback(ColoredCloudCallback cb) {
  std::lock_guard<std::mutex> lk(cb_mutex_);
  colored_cb_ = std::move(cb);
}

void VisualPipeline::setOdomCallback(OdomCallback cb) {
  std::lock_guard<std::mutex> lk(cb_mutex_);
  odom_cb_ = std::move(cb);
}

void VisualPipeline::setImageTimeOffset(double offset) { img_time_offset_.store(offset); }

double VisualPipeline::imageTimeOffset() const { return img_time_offset_.load(); }

void VisualPipeline::setLogCallback(LogCallback cb) {
  std::lock_guard<std::mutex> lk(cb_mutex_);
  log_cb_ = std::move(cb);
}

void VisualPipeline::submitImage(const cv::Mat& bgr, double stamp,
                                 std::shared_ptr<const void> keepalive) {
  if (bgr.empty()) {
    return;
  }
  InputEvent ev;
  ev.kind = InputEvent::Kind::kImage;
  ev.image = bgr;  // 浅拷贝（引用计数），代价与指针拷贝相同。
  ev.image_keepalive = std::move(keepalive);
  ev.stamp = stamp;
  pushInputEvent(std::move(ev));
}

void VisualPipeline::submitOdom(const Eigen::Matrix4d& T_wl, double stamp) {
  InputEvent ev;
  ev.kind = InputEvent::Kind::kOdom;
  ev.T = T_wl;
  ev.stamp = stamp;
  pushInputEvent(std::move(ev));
}

void VisualPipeline::submitCamOdom(const Eigen::Matrix4d& T_wl, double stamp) {
  InputEvent ev;
  ev.kind = InputEvent::Kind::kCamOdom;
  ev.T = T_wl;
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

void VisualPipeline::submitDegeneracy(const superlivo_common::DegeneracySignature& sig) {
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
    log(LogLevel::kWarn, "flush 超时：工作线程可能已停止");
  }
}

std::string VisualPipeline::statsString() const {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "调用=%zu 实际着色帧=%zu 丢弃(无图像)=%zu 丢弃(等位姿超时)=%zu "
                "丢弃(累积无位姿)=%zu 跳过(重复帧)=%zu 延后等待次数=%zu 未完成=%zu "
                "地图点数=%zu 输入丢弃=%zu 跳过(视觉抽帧)=%zu 丢弃(排队超时)=%zu",
                stat_colorizer_calls_, stat_colored_frames_, stat_drop_no_image_,
                stat_drop_pose_, stat_drop_accum_, stat_skip_duplicate_, stat_deferred_,
                pending_color_.size(), colored_map_.size(),
                stat_input_dropped_.load(std::memory_order_relaxed), stat_visual_skipped_,
                stat_drop_stale_);
  return std::string(buf);
}

// ============================================================================
// 线程化设施（worker 消费 InputEvent 队列；save 线程只做彩色地图增量写盘）
// ============================================================================

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
    save_thread_.join();  // 落盘线程写完手上剩余的段再退出
  }
}

bool VisualPipeline::pushInputEvent(InputEvent event) {
  event.enqueued = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (input_stop_) {
      return false;  // 停机后不再接收，避免向已退出的 worker 投递
    }
    // 上限兜底：真积压说明 CPU 被抢或算法退化，此时丢最旧事件以守住内存上界
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
    }

    // 延时保护：单线程 worker 一旦跟不上（图像 4 MB × 10 Hz + 单帧上百毫秒），
    // 队列会越积越长，最终所有配对都因时间差过大而失败（实测端到端 30 s 级）。
    // 这里只对"重算法触发类"事件做过期丢弃：超时的图像/点云已经没有实时价值，
    // 丢掉它们可以立刻把队列拉回实时，而不是让延迟无限增长。
    if (max_queue_latency_ > 0.0 &&
        (ev.kind == InputEvent::Kind::kImage || ev.kind == InputEvent::Kind::kBodyCloud)) {
      const double age =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - ev.enqueued).count();
      if (age > max_queue_latency_) {
        ++stat_drop_stale_;
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "延时保护: 丢弃排队 %.2fs 的%s事件（阈值 %.2fs，累计 %zu 次）", age,
                      ev.kind == InputEvent::Kind::kImage ? "图像" : "点云",
                      max_queue_latency_, stat_drop_stale_);
        logThrottled("stale_event", 2000, LogLevel::kWarn, buf);
        continue;
      }
    }

    // 全部算法状态只在这条线程上访问，因此 handleXxx 内部无需任何锁。
    switch (ev.kind) {
      case InputEvent::Kind::kImage:
        img_enqueue_wall_ = ev.enqueued;  // 延时计时锚点：图像入队墙钟时刻
        handleImage(ev.image, ev.stamp);
        break;
      case InputEvent::Kind::kOdom:
        handleOdom(ev.T, ev.stamp);
        break;
      case InputEvent::Kind::kCamOdom:
        handleCamOdom(ev.T, ev.stamp);
        break;
      case InputEvent::Kind::kCloud:
        handleWorldCloud(ev.points_world);
        break;
      case InputEvent::Kind::kBodyCloud:
        handleBodyCloud(ev.points_body, ev.stamp);
        break;
      case InputEvent::Kind::kDegeneracy:
        handleDegeneracy(ev.degen);
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
    maybePrintLatencyWindow();
  }
}

void VisualPipeline::maybePrintLatencyWindow() {
  if (!opt_.timing_en) {
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
  // 只切"上次移交以来新增的那一段"：彩色地图是 append-only（accumulateColoredMap
  // 只有 push_back，达到上限后直接 return），所以按下标切段等价于整图快照，
  // 但工作线程的代价从 O(全图) 降到 O(一个保存周期新增)。
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
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    save_segments_.push_back(std::move(seg));
    save_pending_ = true;
  }
  save_cv_.notify_one();
}

void VisualPipeline::writeColoredSnapshot(std::deque<ColorSegment> segments) {
  std::error_code ec;
  std::filesystem::create_directories(opt_.pcd_save_dir, ec);
  if (ec) {
    logThrottled("pcd_dir", 5000, LogLevel::kWarn,
                 "无法创建 PCD 输出目录 " + opt_.pcd_save_dir + ": " + ec.message());
    return;
  }
  const std::string path =
      (std::filesystem::path(opt_.pcd_save_dir) / "super_livo_visual_a_colored.pcd")
          .string();

  // ---- 分片模式（参照 Super-LIO scans_*.pcd）：一段写一个文件，纯增量 ----
  // 每片只含本时间窗新增点，落盘线程异步写；分片按时间有序，可直接做时序滤波。
  if (shard_mode_) {
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
          (std::filesystem::path(opt_.pcd_save_dir) / name).string();
      if (pcl::io::savePCDFileBinary(shard_path, cloud) == 0) {
        log(LogLevel::kInfo, "彩色分片已保存: " + shard_path + "（" +
                                 std::to_string(cloud.size()) + " 点）");
      } else {
        logThrottled("pcd_save", 5000, LogLevel::kWarn,
                     "保存 PCD 分片失败: " + shard_path);
      }
    }
    return;
  }

  // 合并增量段到常驻缓冲。正常一个保存周期只有一段；落盘慢于移交时会有多段。
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
    log(LogLevel::kInfo, "彩色地图已保存: " + path + "（" +
                             std::to_string(save_buffer_.size()) + " 点，世界系" +
                             ((save_median_ && save_median_->enabled())
                                  ? "，时间中位数回填色"
                                  : "") +
                             "）");
  } else {
    logThrottled("pcd_save", 5000, LogLevel::kWarn, "保存 PCD 失败: " + path);
  }
}

void VisualPipeline::shutdownAndSave() {
  // 用 call_once 保证只执行一次：shutdown 钩子跑在信号线程，主线程在
  // executor.spin() 返回后也会调一次，两者可能并发；后到者阻塞到那次执行
  // 结束，从而保证进程退出前文件已经落盘。
  std::call_once(shutdown_once_, [this]() {
    // 先停 worker（保留落盘线程）：worker 排空输入队列后退出。
    stopWorkerThreads(/*keep_save_thread=*/true);
    // 收尾：bag 结束时队列里可能还有等图像/等位姿的帧，尽量补完再落盘
    // （配不到图/位的按超时丢弃并计数，与运行期口径一致）。
    drainPendingClouds();
    drainPendingColorization();
    // 分片模式：把最后不足一个间隔的尾段也交给落盘线程写一片，
    // 保证「分片并集 == 合并图」，时序滤波不会漏掉结尾。
    if (shard_mode_) {
      requestColoredMapSave();
    }
    stopSaveThread();
    // 同步写出整张彩色地图。长序列（几百万点 + 中位数回填）要几秒，
    // 这一步必须在本进程内完成，不能交给 save 线程（进程退出时线程被杀）。
    saveColoredMap();
    if (pose_file_.is_open()) {
      pose_file_.flush();
      pose_file_.close();
    }
    // 延时总报告（worker 已 join，无并发 Record；60 s 窗口报告之外的尾量在这）
    if (opt_.timing_en) {
      latency_timer_.PrintAll();
    }
  });
}

// ============================================================================
// 输入处理（仅工作线程）
// ============================================================================

void VisualPipeline::handleImage(const cv::Mat& bgr, double stamp) {
  const cv::Mat rectified = rectifyBgr(bgr);
  if (rectified.empty()) {
    return;
  }
  // 文件名用 LIO 时间域（header + img_time_offset），与轨迹 T_wl 的时间戳同域，
  // 便于后续按时间与 scene_move.txt / GT 关联。
  saveImage(rectified, stamp + img_time_offset_.load());

  // 图像 header 时刻：与 dense 点云 header 处于同一时钟基准（实测差 < 7 ms），
  // 因此**只**用它做"图像-点云配对"。图像真实采集时刻（header + img_time_offset_）
  // 由 runColorizer 单独计算，仅用于取相机位姿，绝不能把两者混成一个时间戳。
  colorizer::TimedImage timed;
  timed.image = rectified;
  timed.exposure_mid_stamp = stamp;
  timed.arrival_stamp = stamp;
  timed.valid = true;
  image_cache_.push_back(timed);
  if (image_cache_.size() > opt_.max_image_cache) {
    image_cache_.erase(image_cache_.begin(),
                       image_cache_.begin() +
                           (image_cache_.size() - opt_.max_image_cache));
  }

  // 视觉抽帧：图像全部保留在缓存里（彩色点云每帧都要用），但前端/深度/更新
  // 只按 visual_decimate 抽帧执行。实测开灯后单帧视觉计算 ~120 ms，10 Hz 输入
  // 根本吃不下；降到 5 Hz 既能保住定位修正，又不会把队列拖成秒级积压。
  ++img_seen_;
  if (visual_decimate_ > 1 && (img_seen_ % static_cast<std::size_t>(visual_decimate_)) != 0) {
    ++stat_visual_skipped_;
    return;
  }

  cv::Mat gray;
  cv::cvtColor(rectified, gray, cv::COLOR_BGR2GRAY);

  // 新图像到达后可以尝试补齐"等图像"的帧（与原节点一致：配对/着色统一在
  // runColorizer 内部完成，这里不再额外 drain，避免改变 S0/S1 的时序口径）。
  processVisualFrame(gray, stamp);
}

void VisualPipeline::handleOdom(const Eigen::Matrix4d& T_wl, double stamp) {
  T_wl_ = T_wl;
  last_odom_stamp_ = stamp;
  savePose(T_wl, stamp);

  // 累积位姿采样供彩色点云运动补偿。
  colorizer::PoseSample sample;
  sample.stamp = stamp;
  sample.T_wl = T_wl;
  pose_cache_.push_back(sample);
  if (pose_cache_.size() > opt_.max_pose_samples) {
    pose_cache_.erase(pose_cache_.begin(),
                      pose_cache_.begin() + (pose_cache_.size() - opt_.max_pose_samples));
  }

  // 状态回灌：把本条 LIO 的增量挂接到修正轨迹尾部。
  // dw = T_lio(k) · T_lio(k-1)⁻¹（世界系左乘增量），own(k) = dw · own(k-1)。
  // 视觉 dx 写回 own 尾采样后，下一条 LIO 增量自然继承修正。
  if (opt_.reinject) {
    colorizer::PoseSample own;
    own.stamp = stamp;
    if (lio_prev_raw_valid_ && !own_pose_cache_.empty()) {
      const Eigen::Matrix4d dw = T_wl * T_lio_prev_raw_.inverse();
      own.T_wl = dw * own_pose_cache_.back().T_wl;
    } else {
      own.T_wl = T_wl;  // 首条：直接以原始 LIO 位姿作种子
    }
    T_lio_prev_raw_ = T_wl;
    lio_prev_raw_valid_ = true;
    own_pose_cache_.push_back(own);
    if (own_pose_cache_.size() > opt_.max_pose_samples) {
      own_pose_cache_.erase(own_pose_cache_.begin(),
                            own_pose_cache_.begin() +
                                (own_pose_cache_.size() - opt_.max_pose_samples));
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

void VisualPipeline::handleCamOdom(const Eigen::Matrix4d& T_wl, double stamp) {
  colorizer::PoseSample sample;
  sample.stamp = stamp;
  sample.T_wl = T_wl;
  cam_pose_cache_.push_back(sample);
  if (cam_pose_cache_.size() > opt_.max_pose_samples) {
    cam_pose_cache_.erase(cam_pose_cache_.begin(),
                          cam_pose_cache_.begin() +
                              (cam_pose_cache_.size() - opt_.max_pose_samples));
  }
  if (!pending_color_.empty()) {
    drainPendingColorization();
  }
}

void VisualPipeline::handleWorldCloud(const std::vector<Eigen::Vector3d>& points_world) {
  sweep_.clear();
  sweep_.reserve(points_world.size());
  for (const auto& p : points_world) {
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
      continue;
    }
    sweep_.push_back(p);
  }

  // 累积局部地图（抽稀 + 上限裁剪）：深度关联会逐点投影，必须控制规模。
  const std::size_t stride = static_cast<std::size_t>(std::max(1, opt_.map_append_stride));
  for (std::size_t i = 0; i < sweep_.size(); i += stride) {
    local_map_.push_back(sweep_[i]);
  }
  if (local_map_.size() > opt_.max_map_points) {
    local_map_.erase(local_map_.begin(),
                     local_map_.begin() + (local_map_.size() - opt_.max_map_points));
  }
  world_cloud_valid_ = true;
}

void VisualPipeline::handleBodyCloud(const std::vector<Eigen::Vector3f>& points_lidar,
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

  // 新扫描到达即入队等图像/位姿（bag2 的图像回调比云慢约 1 帧，不能等图像回调才入队）。
  runColorizer();
}

void VisualPipeline::handleDegeneracy(const superlivo_common::DegeneracySignature& sig) {
  bool first = false;
  {
    std::lock_guard<std::mutex> lk(degen_mutex_);
    degen_sig_ = sig;
    degen_recv_time_ = steadyNowSec();
  }
  if (!degen_seen_) {
    degen_seen_ = true;
    first = true;
  }
  if (first) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "收到退化签名(进程内): cond=%.2f axes=%d active=%d",
                  sig.cond_t, sig.axes, sig.active ? 1 : 0);
    log(LogLevel::kInfo, buf);
  }
}

// ============================================================================
// 视觉主流程
// ============================================================================

void VisualPipeline::processVisualFrame(const cv::Mat& gray, double stamp) {
  const auto t_start = std::chrono::steady_clock::now();

  // 1) 前端跟踪
  const TrackResult track = frontend_->process(gray, stamp);
  const auto t_front = std::chrono::steady_clock::now();

  // 2) 计算当前相机位姿 T_wc = T_wl · T_lc
  //    关键：必须取**图像曝光时刻**的位姿。此前直接用 T_wl_（最新一条 lio/odom），
  //    两者相差可达 0.05~0.15 s；按 0.5 m/s 平移 + 中等角速度折算就是几厘米的
  //    系统性锚点偏移，在 z≈5 m 处对应 2~3 px，正好是重投影 RMSE 长期卡在
  //    2.8 px、门控（阈值 3.0）整帧全拒的主要原因。
  const double t_capture = stamp + img_time_offset_.load();
  Eigen::Matrix4d T_wl_img = Eigen::Matrix4d::Identity();
  if (opt_.reinject) {
    // 回灌模式：基准位姿直接取修正轨迹在曝光时刻的插值，不叠加 pose_offset_。
    if (!poseAtStampCorrected(t_capture, T_wl_img, /*clamp_future=*/true)) {
      // 只有启动初期还没有任何采样才会走到这里。
      T_wl_img = own_pose_cache_.empty() ? T_wl_ : own_pose_cache_.back().T_wl;
    }
  } else if (!opt_.use_exposure_pose) {
    T_wl_img = T_wl_;
  } else {
    (void)poseAtStamp(t_capture, T_wl_img, /*clamp_future=*/true);
    if (pose_cache_.empty()) {
      T_wl_img = T_wl_;  // 启动初期尚无采样
    }
  }
  // 相机与雷达是硬件级触发同步（同一时刻曝光/扫描），输出**一律**按图像采集时刻
  // 打戳；不允许回退到"最新采样自身的时刻"。
  const double pose_stamp = t_capture;
  // 2.1) 叠加此前视觉修正的持久偏移：T_wl_base = pose_offset_ · T_wl_lio。
  //      回灌模式下修正已沉淀进 own_pose_cache_，不再维护持久偏移。
  const Eigen::Matrix4d T_wl_base = opt_.reinject ? T_wl_img : pose_offset_ * T_wl_img;
  const Eigen::Matrix4d T_wc = T_wl_base * T_lc_;

  // 3) 深度关联
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

  // 5) 退化门控
  double degen_severity = 0.0;
  Eigen::Vector3d degen_dir = Eigen::Vector3d::Zero();
  if (!degeneracySignature(degen_severity, degen_dir) && !degen_stale_warned_) {
    degen_stale_warned_ = true;
    log(LogLevel::kWarn,
        "未收到退化签名（超时 " + std::to_string(opt_.degen_timeout) +
            "s）：退化门控将恒不触发。请给 Super-LIO 开启 "
            "lio.observe.degeneracy_detect_en 且 degen_trans_only=1。");
  }
  GateInputs inputs;
  inputs.lidar_degeneracy = degen_severity;
  inputs.tracked_features = static_cast<int>(track.tracked.size());

  // 5.1) 特征空间散布度
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
  inputs.motion_observability =
      std::min(1.0, spread / std::max(1e-6, opt_.gate.min_spread));

  // 5.2) 重投影残差 RMS（像素），并分解"系统性偏差 ⊥ 随机噪声"。
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

  // 5.3) 雅可比条件数（必须在无量纲矩阵上算）
  if (H.rows() == 6 && H.cols() == 6) {
    const double s_rot = std::max(1e-9, opt_.update.prior_rot_sigma);
    const double s_pos = std::max(1e-9, opt_.update.prior_pos_sigma);
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
  Eigen::Matrix4d T_wl_out = T_wl_base;
  if (output.accepted) {
    const Eigen::Matrix4d T_wc_corrected = applyLeftPerturbation(T_wc, output.dx);
    T_wl_out = T_wc_corrected * T_lc_.inverse();
    if (opt_.reinject) {
      if (!own_pose_cache_.empty() &&
          std::abs(own_pose_cache_.back().stamp - pose_stamp) <= 0.15) {
        own_pose_cache_.back().T_wl = T_wl_out;
      }
    } else {
      const Eigen::Matrix4d T_lio_inv = T_wl_img.inverse();
      pose_offset_ = T_wl_out * T_lio_inv;
    }
  }
  // 7.1) 持久偏移的泄漏
  {
    const double dt =
        last_visual_stamp_ < 0.0 ? 0.0 : std::clamp(stamp - last_visual_stamp_, 0.0, 0.5);
    last_visual_stamp_ = stamp;
    if (!opt_.reinject) {
      decayPoseOffset(dt, degen_severity, degen_dir);
    }
  }
  if (!opt_.reinject) {
    T_wl_ = T_wl_out;  // 回灌模式 T_wl_ 保持最新原始 lio/odom，供 onOdom 挂链接量
  }

  // 8) 发布修正后里程计与轨迹（按实际取用的位姿时刻打戳）。
  publishOdomAndPath(T_wl_out, pose_stamp);
  const auto t_pub = std::chrono::steady_clock::now();

  // 8.5) 缓存本帧修正位姿（按 pose_stamp），供 color.use_own_odom=true 时着色插值。
  if (!opt_.reinject) {
    colorizer::PoseSample own;
    own.stamp = pose_stamp;
    own.T_wl = T_wl_;
    own_pose_cache_.push_back(own);
    if (own_pose_cache_.size() > opt_.max_pose_samples) {
      own_pose_cache_.erase(own_pose_cache_.begin(),
                            own_pose_cache_.begin() +
                                (own_pose_cache_.size() - opt_.max_pose_samples));
    }
  }

  // 9) 若缓存了机体系点云，触发彩色点云（与视觉定位解耦，独立工作）
  if (!body_points_.empty() && world_cloud_valid_) {
    runColorizer();
  }
  const auto t_end = std::chrono::steady_clock::now();
  auto ms = [](const std::chrono::steady_clock::time_point& a,
               const std::chrono::steady_clock::time_point& b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };

  // 10) 诊断日志（1 Hz 节流）
  {
    char buf[768];
    std::snprintf(
        buf, sizeof(buf),
        "视觉帧: tracked=%zu/%d n=%d spread=%.3f rmse=%.2f bias=%.2f cond=%.1f deg=%.2f "
        "mode=%d w=%.2f %s | 耗时 ms: 前端=%.1f 深度=%.1f 重投影=%.1f 门控=%.1f "
        "更新=%.1f 发布=%.1f 彩色=%.1f 合计=%.1f",
        track.tracked.size(), track.detected, valid_blocks, inputs.spatial_spread,
        inputs.reprojection_rmse, bias_px, inputs.jacobian_condition,
        inputs.lidar_degeneracy, static_cast<int>(decision.mode), decision.visual_weight,
        decision.reason.c_str(), ms(t_start, t_front), ms(t_front, t_depth),
        ms(t_depth, t_reproj), ms(t_reproj, t_gate), ms(t_gate, t_update),
        ms(t_update, t_pub), ms(t_pub, t_end), ms(t_start, t_end));
    logThrottled("visual_frame", 1000, LogLevel::kInfo, buf);
  }

  // 10.1) 退化/回灌状态（1 Hz）
  {
    const double dir_norm = degen_dir.norm();
    const Eigen::Vector3d u_w =
        dir_norm > 1e-6 ? Eigen::Vector3d(degen_dir / dir_norm) : Eigen::Vector3d::Zero();
    const double corr_along_u =
        output.accepted ? 1e3 * output.dx.tail<3>().dot(u_w) : 0.0;
    Eigen::Matrix4d off_disp = pose_offset_;
    if (opt_.reinject && !own_pose_cache_.empty() && !pose_cache_.empty()) {
      off_disp = own_pose_cache_.back().T_wl * pose_cache_.back().T_wl.inverse();
    }
    const double off_along_u = 1e3 * off_disp.block<3, 1>(0, 3).dot(u_w);
    char buf[512];
    std::snprintf(
        buf, sizeof(buf),
        "退化状态: sev=%.2f dir=(%+.2f %+.2f %+.2f) 修正=%.1fmm(沿u%+.1f) | "
        "持久偏移=%.1fmm(沿u%+.1f) %.2f°",
        degen_severity, degen_dir.x(), degen_dir.y(), degen_dir.z(),
        output.accepted ? 1e3 * output.dx.tail<3>().norm() : 0.0, corr_along_u,
        1e3 * off_disp.block<3, 1>(0, 3).norm(), off_along_u,
        off_disp.block<3, 3>(0, 0).eulerAngles(0, 1, 2).norm() * 180.0 / M_PI);
    logThrottled("degen_state", 1000, LogLevel::kInfo, buf);
  }
}

// ============================================================================
// 彩色点云
// ============================================================================

void VisualPipeline::runColorizer() {
  ++stat_colorizer_calls_;
  if (body_points_.empty()) {
    return;
  }

  const double t_cloud = body_stamp_;
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
      log(LogLevel::kWarn, "彩色[丢弃] 等图像队列溢出，丢弃最早一帧");
    }
  }

  drainPendingClouds();
  drainPendingColorization();
}

void VisualPipeline::drainPendingClouds() {
  const auto now = std::chrono::steady_clock::now();
  for (auto it = pending_clouds_.begin(); it != pending_clouds_.end();) {
    // 官方口径配对：比较 |header + img_time_offset - t_cloud|。
    const colorizer::TimedImage* chosen = nullptr;
    double best_dt = std::numeric_limits<double>::max();
    for (const auto& image : image_cache_) {
      if (!image.valid || image.image.empty()) {
        continue;
      }
      const double dt = std::abs(image.arrival_stamp + img_time_offset_.load() - it->t_cloud);
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
      frame.t_img = frame.t_pair + img_time_offset_.load();
      frame.image = chosen->image;
      frame.queued_at = now;
      pending_color_.push_back(std::move(frame));
      while (pending_color_.size() > kMaxPendingColorFrames) {
        pending_color_.pop_front();
        ++stat_drop_pose_;
        log(LogLevel::kWarn, "彩色[丢弃] 待处理队列溢出，丢弃最早一帧");
      }
      it = pending_clouds_.erase(it);
      continue;
    }
    // 图像还没到：等下一帧图像/云/里程计再试；等太久（例如相机断流）才真丢弃。
    const double waited = std::chrono::duration<double>(now - it->queued_at).count();
    if (waited > kPendingCloudTimeoutSec) {
      ++stat_drop_no_image_;
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "彩色[丢弃] 等待图像超时: t_cloud=%.4f 最近差=%.4f 阈值=%.4f 已等 %.2fs",
                    it->t_cloud, best_dt, colorizer_options_.sync.max_offset, waited);
      logThrottled("drop_no_image", 5000, LogLevel::kWarn, buf);
      it = pending_clouds_.erase(it);
      continue;
    }
    ++it;
  }
}

void VisualPipeline::drainPendingColorization() {
  const auto now = std::chrono::steady_clock::now();
  for (auto it = pending_color_.begin(); it != pending_color_.end();) {
    if (colorizeFrame(*it)) {
      it = pending_color_.erase(it);
      continue;
    }
    // 位姿仍未到齐：等一会儿再来（由 odom 触发）。超过上限说明确实等不到。
    const double waited = std::chrono::duration<double>(now - it->queued_at).count();
    if (waited > kPendingColorTimeoutSec) {
      ++stat_drop_pose_;
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "彩色[丢弃] 等待位姿超时: t_cloud=%.4f t_img=%.4f 已等 %.2fs",
                    it->t_cloud, it->t_img, waited);
      logThrottled("drop_pose", 5000, LogLevel::kWarn, buf);
      it = pending_color_.erase(it);
      continue;
    }
    ++stat_deferred_;
    ++it;
  }
}

bool VisualPipeline::colorizeFrame(const PendingColorFrame& frame) {
  const double t_cloud = frame.t_cloud;
  const double t_img = frame.t_img;

  // 两个时刻的位姿都必须由缓存插值得到，任一失败就稍后重试。
  Eigen::Matrix4d T_wl_cloud = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_wl_img = Eigen::Matrix4d::Identity();
  if (!poseAtStamp(t_cloud, T_wl_cloud) || !camPoseAtStamp(t_img, T_wl_img)) {
    return false;
  }

  // 统一时间基准：相机位姿取 t_img，点云世界系变换取 t_cloud。
  colorizer::ColorizerPipeline::Options options = colorizer_options_;
  const Eigen::Matrix4d T_wc_img = T_wl_img * T_lc_;
  options.T_cl = T_wc_img.inverse();  // 实现语义为 T_cw

  colorizer::TimedImage image;
  image.image = frame.image;
  image.exposure_mid_stamp = frame.t_img;
  image.arrival_stamp = frame.t_img;
  image.valid = true;
  std::vector<colorizer::TimedImage> images_for_frame{std::move(image)};
  std::vector<double> point_stamps(frame.points_lidar.size(), t_cloud);

  const colorizer::ColorizerPipeline pipeline(options);
  const auto t_proj0 = std::chrono::steady_clock::now();
  const std::vector<colorizer::ColoredPoint> colored =
      pipeline.process(frame.points_lidar, point_stamps, images_for_frame, pose_cache_);
  const auto t_proj1 = std::chrono::steady_clock::now();
  if (colored.empty()) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "彩色点云为空（t_cloud=%.4f t_img=%.4f）", t_cloud,
                  t_img);
    logThrottled("empty_colored", 5000, LogLevel::kWarn, buf);
  }

  std::size_t colored_count = 0;
  for (const auto& p : colored) {
    if (p.quality > 0.0F) {
      ++colored_count;
    }
  }
  ++stat_colored_frames_;
  last_colored_cloud_stamp_ = t_cloud;
  {
    char buf[320];
    std::snprintf(buf, sizeof(buf),
                  "彩色: t_cloud=%.4f t_pair=%.4f t_img=%.4f(offset=%+.3f) "
                  "dt_pair_cloud=%+.4f dt_img_cloud=%+.4f %zu/%zu 点上色 帧#%zu 待处理=%zu",
                  t_cloud, frame.t_pair, t_img, img_time_offset_.load(), frame.t_pair - t_cloud,
                  t_img - t_cloud, colored_count, colored.size(), stat_colored_frames_,
                  pending_color_.size());
    logThrottled("color_frame", 1000, LogLevel::kInfo, buf);
  }

  // 累积到世界系彩色地图（定时器/析构时统一写 PCD）。
  accumulateColoredMap(colored, t_cloud);
  const auto t_acc1 = std::chrono::steady_clock::now();
  publishColoredCloud(colored, t_cloud);
  const auto t_pub1 = std::chrono::steady_clock::now();
  if (opt_.timing_en) {
    const auto ms = [](std::chrono::steady_clock::time_point a,
                       std::chrono::steady_clock::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    char buf[192];
    std::snprintf(buf, sizeof(buf), "[彩色耗时] 投影=%.1f 累积=%.1f 发布=%.1f ms (点数=%zu 缓存位姿=%zu)",
                  ms(t_proj0, t_proj1), ms(t_proj1, t_acc1), ms(t_acc1, t_pub1),
                  frame.points_lidar.size(), pose_cache_.size());
    logThrottled("color_cost", 1000, LogLevel::kInfo, buf);
  }
  return true;
}

// ============================================================================
// 位姿查询
// ============================================================================

bool VisualPipeline::poseAtStamp(double stamp, Eigen::Matrix4d& T_wl,
                                 bool clamp_future) const {
  const std::vector<colorizer::PoseSample>& cache =
      opt_.color_use_own_odom ? own_pose_cache_ : pose_cache_;
  if (cache.empty()) {
    return false;
  }
  const colorizer::PoseSample* newest = &cache.front();
  for (const auto& sample : cache) {
    if (sample.stamp > newest->stamp) {
      newest = &sample;
    }
  }
  if (stamp > newest->stamp) {
    if (!clamp_future) {
      return false;
    }
    // 硬件级触发同步：最新采样就是当前可用位姿，直接取，绝不推迟到下一帧。
    T_wl = newest->T_wl;
    return true;
  }

  colorizer::LidarPoseInterpolator interpolator(
      colorizer::LidarPoseInterpolator::Options{kPoseHistoryMaxGapSec});
  for (const auto& sample : cache) {
    interpolator.addSample(sample);
  }
  return interpolator.poseAt(stamp, T_wl);
}

bool VisualPipeline::camPoseAtStamp(double stamp, Eigen::Matrix4d& T_wl) const {
  if (cam_pose_cache_.empty()) {
    return poseAtStamp(stamp, T_wl);  // 未配置独立相机位姿源：沿用旧行为
  }
  double newest = -std::numeric_limits<double>::max();
  for (const auto& sample : cam_pose_cache_) {
    newest = std::max(newest, sample.stamp);
  }
  if (stamp > newest) {
    return false;  // 未来时刻：由调用方延后重试
  }
  colorizer::LidarPoseInterpolator interpolator(
      colorizer::LidarPoseInterpolator::Options{kPoseHistoryMaxGapSec});
  for (const auto& sample : cam_pose_cache_) {
    interpolator.addSample(sample);
  }
  return interpolator.poseAt(stamp, T_wl);
}

bool VisualPipeline::poseAtStampCorrected(double stamp, Eigen::Matrix4d& T_wl,
                                          bool clamp_future) const {
  if (own_pose_cache_.empty()) {
    return false;
  }
  const colorizer::PoseSample* newest = &own_pose_cache_.front();
  for (const auto& sample : own_pose_cache_) {
    if (sample.stamp > newest->stamp) {
      newest = &sample;
    }
  }
  if (stamp > newest->stamp) {
    if (!clamp_future) {
      return false;
    }
    T_wl = newest->T_wl;
    return true;
  }
  colorizer::LidarPoseInterpolator interpolator(
      colorizer::LidarPoseInterpolator::Options{kPoseHistoryMaxGapSec});
  for (const auto& sample : own_pose_cache_) {
    interpolator.addSample(sample);
  }
  return interpolator.poseAt(stamp, T_wl);
}

// ============================================================================
// 输出与地图累积
// ============================================================================

void VisualPipeline::publishColoredCloud(
    const std::vector<colorizer::ColoredPoint>& points, double stamp) {
  if (opt_.timing_en && img_enqueue_wall_.time_since_epoch().count() != 0) {
    latency_timer_.Record(
        "[Img->VisCloud]",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  img_enqueue_wall_)
            .count());
  }
  ColoredCloudCallback cb;
  {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    cb = colored_cb_;
  }
  if (cb) {
    cb(points, stamp);
  }
}

void VisualPipeline::accumulateColoredMap(
    const std::vector<colorizer::ColoredPoint>& points, double stamp) {
  if (points.empty()) {
    return;
  }
  if (colored_map_.size() >= opt_.max_colored_map_points) {
    if (!pcd_capacity_warned_) {
      pcd_capacity_warned_ = true;
      log(LogLevel::kWarn, "彩色地图点数达到上限 " +
                               std::to_string(opt_.max_colored_map_points) +
                               "，后续帧不再累积");
    }
    return;
  }

  // 与彩色流水线保持一致：点已去畸变到扫描帧头，这里只需在 stamp 时刻取一次 T_wl。
  colorizer::LidarPoseInterpolator interp(
      colorizer::LidarPoseInterpolator::Options{kPoseHistoryMaxGapSec});
  for (const auto& sample : pose_cache_) {
    interp.addSample(sample);
  }
  Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
  if (!interp.poseAt(stamp, T_wl)) {
    // 禁止回退到最新位姿：那会把点云搬到错误时刻的世界系，产生跨帧错位的"重影"。
    ++stat_drop_accum_;
    char buf[160];
    std::snprintf(buf, sizeof(buf), "彩色[丢弃] 累积位姿插值失败: stamp=%.4f 缓存=%zu",
                  stamp, pose_cache_.size());
    logThrottled("drop_accum", 5000, LogLevel::kWarn, buf);
    return;
  }

  const Eigen::Matrix3d R = T_wl.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T_wl.block<3, 1>(0, 3);

  // ---- 时间中位数：只统计颜色，几何仍按原逻辑全量 append ----
  if (median_map_.enabled()) {
    for (const auto& cp : points) {
      if (cp.quality <= 0.0F) continue;
      const Eigen::Vector3f pw = (R * cp.xyz.cast<double>() + t).cast<float>();
      median_map_.insert(pw, cp.rgb);
    }
  }

  const std::size_t room = opt_.max_colored_map_points - colored_map_.size();
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

  // 分片落盘：每 N 个真正上过色的帧（或每 N 秒）把一个增量段交给落盘线程
  // （参照 Super-LIO lio.map.save_interval）。段内只含本时间窗新增点，写盘不阻塞工作线程。
  if (shard_mode_ && added > 0) {
    bool due = false;
    if (save_interval_frames_ > 0 && ++frames_since_shard_ >= save_interval_frames_) {
      frames_since_shard_ = 0;
      due = true;
    }
    if (shard_interval_sec_ > 0.0) {
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now - last_shard_time_).count() >=
          shard_interval_sec_) {
        last_shard_time_ = now;
        due = true;
      }
    }
    if (due) {
      requestColoredMapSave();
    }
  }
}

void VisualPipeline::saveColoredMap() {
  if (colored_map_.empty() || colored_map_.size() == pcd_saved_points_) {
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories(opt_.pcd_save_dir, ec);
  if (ec) {
    logThrottled("pcd_dir", 5000, LogLevel::kWarn,
                 "无法创建 PCD 输出目录 " + opt_.pcd_save_dir + ": " + ec.message());
    return;
  }

  const std::string path =
      (std::filesystem::path(opt_.pcd_save_dir) / "super_livo_visual_a_colored.pcd")
          .string();

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
    log(LogLevel::kInfo, "彩色地图已保存: " + path + "（" +
                             std::to_string(colored_map_.size()) + " 点，世界系" +
                             (median_map_.enabled() ? "，时间中位数回填色" : "") + "）");
  } else {
    logThrottled("pcd_save", 5000, LogLevel::kWarn, "保存 PCD 失败: " + path);
  }
}

void VisualPipeline::savePose(const Eigen::Matrix4d& T_wl, double stamp) {
  if (!pose_file_.is_open() || opt_.pose_save_interval <= 0) {
    return;
  }
  if (pose_frame_count_++ % static_cast<std::size_t>(opt_.pose_save_interval) != 0) {
    return;
  }
  const Eigen::Quaterniond q(T_wl.block<3, 3>(0, 0));
  pose_file_ << std::fixed << std::setprecision(6)
             << stamp << " " << T_wl(0, 3) << " " << T_wl(1, 3) << " " << T_wl(2, 3)
             << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
             << std::endl;
}

void VisualPipeline::saveImage(const cv::Mat& bgr, double stamp) {
  if (img_save_dir_resolved_.empty() || opt_.img_save_interval <= 0) {
    return;
  }
  if (img_frame_count_++ % static_cast<std::size_t>(opt_.img_save_interval) != 0) {
    return;
  }
  std::ostringstream name;
  name << std::fixed << std::setprecision(6) << stamp << ".png";
  const std::string path =
      (std::filesystem::path(img_save_dir_resolved_) / name.str()).string();
  if (!cv::imwrite(path, bgr)) {
    logThrottled("img_save", 5000, LogLevel::kWarn, "保存图像失败: " + path);
  }
}

void VisualPipeline::publishOdomAndPath(const Eigen::Matrix4d& T_wl, double stamp) {
  if (opt_.timing_en && img_enqueue_wall_.time_since_epoch().count() != 0) {
    latency_timer_.Record(
        "[Img->VisOdom]",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  img_enqueue_wall_)
            .count());
  }
  OdomCallback cb;
  {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    cb = odom_cb_;
  }
  if (cb) {
    cb(T_wl, stamp);
  }
}

// ============================================================================
// 辅助
// ============================================================================

cv::Mat VisualPipeline::rectifyBgr(const cv::Mat& bgr) const {
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
    cv::initUndistortRectifyMap(K, D, cv::Mat(), K, size, CV_32FC1, rectify_map1_,
                                rectify_map2_);
    rectify_maps_ready_ = true;
  }
  cv::Mat rectified;
  cv::remap(bgr, rectified, rectify_map1_, rectify_map2_, cv::INTER_LINEAR);
  return rectified;
}

Eigen::Matrix4d VisualPipeline::makeTlcFromRclPcl(const std::vector<double>& rcl,
                                                  const std::vector<double>& pcl) const {
  // avia_lvi.yaml 的 Rcl/Pcl 与 FAST-LIVO2 用法一致，表示
  //   p_cam = Rcl · p_lidar + Pcl        （即 T_cam_lidar，LiDAR -> Camera）
  // 而流水线内部统一使用 T_lc（Camera -> LiDAR），所以这里要求逆。
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

Eigen::Matrix4d VisualPipeline::applyLeftPerturbation(const Eigen::Matrix4d& T,
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

bool VisualPipeline::degeneracySignature(double& severity, Eigen::Vector3d& dir_w) const {
  severity = 0.0;
  dir_w.setZero();
  std::lock_guard<std::mutex> lock(degen_mutex_);
  if (degen_recv_time_ < 0.0 || steadyNowSec() - degen_recv_time_ > opt_.degen_timeout) {
    return false;  // 没有签名可用 -> 调用方按"不退化"处理
  }
  // cond=0 说明 Super-LIO 侧根本没跑检测（detect_en 关闭），或非 trans_only 模式。
  if (!degen_sig_.valid()) {
    return false;
  }
  dir_w = degen_sig_.dir;
  severity = degen_sig_.severity(opt_.gate_cond_lo, opt_.gate_cond_hi);
  return true;
}

void VisualPipeline::decayPoseOffset(double dt, double severity,
                                     const Eigen::Vector3d& dir_w) {
  // 把持久偏移改成带泄漏的累积器，衰减率只用 Super-LIO 的退化签名来定：
  //   * 旋转：trans_only 检测不给旋转方向，各向同性按 α_perp 衰减；
  //   * 平移垂直 u：LiDAR 一直看得住，按 α_perp 衰减；
  //   * 平移沿 u：α_perp·(1-sev)，完全退化（sev=1）时不衰减，允许视觉累积。
  if (!opt_.offset_decay_en || opt_.offset_decay_rate <= 0.0 || dt <= 0.0) {
    return;
  }
  const double sev = std::clamp(severity, 0.0, 1.0);
  const double alpha_perp = opt_.offset_decay_rate;
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

// ============================================================================
// 日志
// ============================================================================

double VisualPipeline::steadyNowSec() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void VisualPipeline::log(LogLevel level, const std::string& msg) const {
  LogCallback cb;
  {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    cb = log_cb_;
  }
  if (cb) {
    cb(level, msg);
  }
}

void VisualPipeline::logThrottled(const std::string& key, int period_ms, LogLevel level,
                                  const std::string& msg) const {
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lk(log_last_mutex_);
    auto it = log_last_.find(key);
    if (it != log_last_.end()) {
      const double dt = std::chrono::duration<double>(now - it->second).count();
      if (dt * 1000.0 < static_cast<double>(period_ms)) {
        return;
      }
    }
    log_last_[key] = now;
  }
  log(level, msg);
}

}  // namespace superlivo_a
