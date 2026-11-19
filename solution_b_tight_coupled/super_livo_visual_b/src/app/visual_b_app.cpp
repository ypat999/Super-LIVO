// ============================================================================
// 方案 B 独立可执行入口（单体化 S2：单节点）
// ----------------------------------------------------------------------------
// 架构约定：super_lio 是公共代码库（liblio），本项目只"引用"它的资源：
//   * 通过 find_package(super_lio) + target_link_libraries(... super_lio::lio)
//     在编译期链接；
//   * 通过继承 LI2Sup::SuperLIO 并 override ExternalObserve()，在代码级
//     （而非话题级）把视觉约束写回 ESKF。
// 因此本可执行文件自身就是完整方案：一个进程、一个 ROS 节点内同时运行
//   (1) Super-LIO 主干（建图/配准/输出，全部来自公共库）
//   (2) 方案 B 视觉支路（ROS 无关 VisualPipeline）
// 视觉支路的输入由 SuperLIO 的进程内钩子直推（onLioOutput / onDenseCloud /
// degeneracySnapshot），输出由本文件里的回调发布；不再有 DDS 回环。
// ============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lio/ESKF.h"
#include "lio/params.h"
#include "lio/super_lio.h"
#include "ros/ROSWrapper.h"
#include "super_livo_common/degeneracy_signature.hpp"

#include "super_livo_visual_b/bridge/constraint_bridge.hpp"
#include "super_livo_visual_b/pipeline/visual_pipeline.hpp"

namespace {

/// 取参数：未声明则用默认值声明，已声明（YAML 覆盖）则直接读。
template <typename T>
T getOrDeclare(rclcpp::Node& node, const std::string& name, const T& def) {
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, def);
}

/// 把 VisualPipeline 的参数读取接到 ROS 参数服务上。
/// 流水线内部保持原有参数名与默认值，YAML 的 /** 通配块可直接覆盖。
class RosParamSource : public superlivo_b::ParamSource {
 public:
  explicit RosParamSource(rclcpp::Node& node) : node_(node) {}
  bool getBool(const std::string& n, bool d) override { return getOrDeclare<bool>(node_, n, d); }
  int getInt(const std::string& n, int d) override { return getOrDeclare<int>(node_, n, d); }
  double getDouble(const std::string& n, double d) override {
    return getOrDeclare<double>(node_, n, d);
  }
  float getFloat(const std::string& n, float d) override {
    return getOrDeclare<float>(node_, n, d);
  }
  std::string getString(const std::string& n, const std::string& d) override {
    return getOrDeclare<std::string>(node_, n, d);
  }
  std::vector<double> getDoubleArray(const std::string& n,
                                     const std::vector<double>& d) override {
    return getOrDeclare<std::vector<double>>(node_, n, d);
  }

 private:
  rclcpp::Node& node_;
};

/// double 秒 -> ROS 时间戳。
builtin_interfaces::msg::Time toRosStamp(double sec) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(std::floor(sec));
  stamp.nanosec = static_cast<uint32_t>((sec - static_cast<double>(stamp.sec)) * 1e9);
  return stamp;
}

/// 方案 B 的 LIO 主体：只覆写公共库预留的通用扩展点。
/// 约束写回与视觉输入直推都放在这里，super_lio 内部不出现任何视觉概念。
class VisualBSuperLIO : public LI2Sup::SuperLIO {
 public:
  /// enable=false 或 bridge 为空时 ExternalObserve() 立即返回，
  /// 整条链路与纯 LIO 逐位一致（回归口径）。
  void setConstraintBridge(const std::shared_ptr<superlivo_b::ConstraintBridge>& bridge,
                           bool enable, double max_dt) {
    bridge_ = bridge;
    inject_enable_ = enable;
    inject_max_dt_ = max_dt;
  }

  /// 单体化：视觉流水线（裸指针；生命周期由 main 中的 shared_ptr 保证）。
  void setVisualPipeline(superlivo_b::VisualPipeline* pipeline) {
    pipeline_.store(pipeline, std::memory_order_release);
  }

  long applied() const { return applied_; }
  long rejected() const { return rejected_; }
  long stale() const { return stale_; }
  /// 探针：ExternalObserve 的调用次数 / 间隔（= LIO 雷达帧处理节奏），以及
  /// 约束被取用时的时差（>0 表示约束落后于 ESKF）。
  long extCalls() const { return ext_calls_; }
  double extGapAvgMs() const { return gap_n_ > 0 ? gap_sum_ms_ / static_cast<double>(gap_n_) : 0.0; }
  double extGapMaxMs() const { return gap_max_ms_; }
  double takeAgeAvgMs() const { return age_n_ > 0 ? 1000.0 * age_sum_ / static_cast<double>(age_n_) : 0.0; }
  double takeAgeAbsMaxMs() const { return 1000.0 * age_abs_max_; }
  /// 取用时的 |age| 直方图：<0.1 / <0.2 / <0.3 / <0.4 / ≥0.4 s。
  const std::array<long, 5>& takeAgeHist() const { return age_hist_; }

  /// 每帧雷达观测（含信任域裁决）之后、建图与输出之前调用。
  /// 视觉约束通过 eskf()->UpdateObserveFull(H, r, R) 在此写入。
  void ExternalObserve() override {
    if (!inject_enable_ || !bridge_) {
      return;
    }
    // ---- 探针：调用节奏。ExternalObserve 挂在 LIO 的 process() 之后、每帧雷达
    // 观测调用一次，因此它和上一帧的间隔就是 LIO 主干的"处理间隙"；间隙被拉长
    // 说明主线程被阻塞（视觉回调抢占 / 建图 / 落盘），此时视觉约束会被拖旧。
    {
      const auto now = std::chrono::steady_clock::now();
      if (last_call_.time_since_epoch().count() != 0) {
        const double gap =
            std::chrono::duration<double, std::milli>(now - last_call_).count();
        gap_sum_ms_ += gap;
        gap_max_ms_ = std::max(gap_max_ms_, gap);
        ++gap_n_;
      }
      last_call_ = now;
      ++ext_calls_;
    }
    superlivo_b::VisualObs obs;
    if (!bridge_->take(obs)) {
      return;
    }
    // 约束取用时的时差：正值表示约束时间戳落后 ESKF 当前时刻。
    {
      const double age = eskf()->GetTime() - obs.stamp;
      age_sum_ += age;
      age_abs_max_ = std::max(age_abs_max_, std::abs(age));
      ++age_n_;
      const double a = std::abs(age);
      ++age_hist_[a < 0.1 ? 0 : (a < 0.2 ? 1 : (a < 0.3 ? 2 : (a < 0.4 ? 3 : 4)))];
    }
    // 视觉帧异步于雷达帧：时间差过大说明这条约束已过期，直接丢弃（不重试）。
    if (std::abs(obs.stamp - eskf()->GetTime()) > inject_max_dt_) {
      ++stale_;
      return;
    }
    const bool ok = eskf()->UpdateObserveFull(
        [&obs](const auto& /*kf_state*/, Eigen::MatrixXd& H, Eigen::VectorXd& r,
               Eigen::MatrixXd& Rm) {
          H = obs.H;
          r = obs.r;
          Rm = obs.R;
        });
    if (ok) {
      ++applied_;
    } else {
      ++rejected_;
    }
  }

 protected:
  /// 每帧输出线程取到 OutputData 后调用：位姿 + 世界系点云 + 退化签名。
  /// 等价于旧路径订阅的 /lio/odom、/lio/cloud_world、/lio/degeneracy。
  void onLioOutput(const OutputData& data) override {
    auto* pipeline = pipeline_.load(std::memory_order_acquire);
    if (pipeline == nullptr) {
      return;
    }
    // 位姿：等价于原 lio/odom（pub_odom 仅在非 undistort-only 帧发布）。
    if (!data.is_undistort_only) {
      Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
      T_wl.block<3, 3>(0, 0) = data.state.R.R().cast<double>();
      T_wl.block<3, 1>(0, 3) = data.state.p.cast<double>();
      pipeline->submitOdom(T_wl, data.state.timestamp);
    }

    // 世界系点云：等价于原 lio/cloud_world（undistort-only 帧是雷达系，不能用）。
    if (!data.is_undistort_only && data.has_world_pc && data.world_pc) {
      std::vector<Eigen::Vector3d> points;
      points.reserve(data.world_pc->size());
      for (const auto& p : data.world_pc->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          continue;
        }
        points.emplace_back(p.x, p.y, p.z);
      }
      pipeline->submitWorldCloud(points);
    }

    // 退化签名：等价于原 /lio/degeneracy 话题载荷。
    const DegeneracySnapshot snap = degeneracySnapshot();
    if (snap.ok) {
      superlivo_common::DegeneracySignature sig;
      sig.cond6 = snap.cond;
      sig.cond_t = snap.cond;
      sig.axes = snap.axes;
      sig.active = snap.active;
      const Eigen::Vector3d dir(snap.dir_w.cast<double>());
      sig.dir = dir.norm() > 1e-6 ? dir.normalized() : Eigen::Vector3d::Zero();
      pipeline->submitDegeneracy(sig);
    }
  }

  /// DenseThread 产出全量机体系点云后调用：等价于原 lio/cloud_body_dense。
  void onDenseCloud(const BASIC::CloudPtr& cloud, double stamp) override {
    auto* pipeline = pipeline_.load(std::memory_order_acquire);
    if (pipeline == nullptr || !cloud) {
      return;
    }
    std::vector<Eigen::Vector3f> points;
    points.reserve(cloud->size());
    for (const auto& p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      points.emplace_back(p.x, p.y, p.z);
    }
    pipeline->submitBodyCloud(points, stamp);
  }

 private:
  std::atomic<superlivo_b::VisualPipeline*> pipeline_{nullptr};
  std::shared_ptr<superlivo_b::ConstraintBridge> bridge_;
  bool inject_enable_{false};
  double inject_max_dt_{0.2};
  long applied_{0};
  long rejected_{0};
  long stale_{0};
  long ext_calls_{0};
  long gap_n_{0};
  double gap_sum_ms_{0.0};
  double gap_max_ms_{0.0};
  long age_n_{0};
  double age_sum_{0.0};
  double age_abs_max_{0.0};
  std::array<long, 5> age_hist_{};
  std::chrono::steady_clock::time_point last_call_{};
};

/// 单体化模式的 IO 句柄：外部输入订阅 + 视觉输出发布 + 周期落盘。
struct MonolithVisualIo {
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cam_odom_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rot_odom_pub;

  rclcpp::TimerBase::SharedPtr save_timer;
  nav_msgs::msg::Path path_msg;
};

/// 把 pipeline 的输出接到唯一节点 wrapper 的发布器上，并订阅外部输入。
std::shared_ptr<MonolithVisualIo> setupMonolithIo(
    LI2Sup::ROSWrapper& node, const std::shared_ptr<superlivo_b::VisualPipeline>& pipeline) {
  auto io = std::make_shared<MonolithVisualIo>();
  io->colored_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "super_livo_visual_b/colored_cloud", rclcpp::SensorDataQoS());
  io->odom_pub = node.create_publisher<nav_msgs::msg::Odometry>(
      "super_livo_visual_b/odom", rclcpp::SensorDataQoS());
  io->path_pub = node.create_publisher<nav_msgs::msg::Path>(
      "super_livo_visual_b/path", rclcpp::SensorDataQoS());
  io->rot_odom_pub = node.create_publisher<nav_msgs::msg::Odometry>(
      "super_livo_visual_b/odom_rot_refined", rclcpp::SensorDataQoS());
  io->path_msg.header.frame_id = "world";

  // ---- 彩色点云输出 ----
  auto colored_pub = io->colored_pub;
  pipeline->setColoredCloudCallback(
      [colored_pub](const std::vector<colorizer::ColoredPoint>& points, double stamp) {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.reserve(points.size());
        const auto clamp_u8 = [](float v) {
          return static_cast<std::uint8_t>(std::clamp(v, 0.0F, 255.0F));
        };
        for (const auto& cp : points) {
          pcl::PointXYZRGB p;
          p.x = cp.xyz.x();
          p.y = cp.xyz.y();
          p.z = cp.xyz.z();
          p.r = clamp_u8(cp.rgb.x());
          p.g = clamp_u8(cp.rgb.y());
          p.b = clamp_u8(cp.rgb.z());
          cloud.push_back(p);
        }
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = toRosStamp(stamp);
        // 点仍在 LiDAR 系；Super-LIO 名义上把该坐标系叫 "imu"。
        msg.header.frame_id = "imu";
        colored_pub->publish(msg);
      });

  // ---- 位姿 + 轨迹输出 ----
  auto odom_pub = io->odom_pub;
  auto path_pub = io->path_pub;
  pipeline->setOdomCallback([io, odom_pub, path_pub](const Eigen::Matrix4d& T_wl,
                                                     double stamp) {
    const auto ros_stamp = toRosStamp(stamp);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = ros_stamp;
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
    odom_pub->publish(odom);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    io->path_msg.header.stamp = ros_stamp;
    io->path_msg.poses.push_back(pose);
    path_pub->publish(io->path_msg);
  });

  // ---- 旋转精修通道（rot_refine.enable=false 时不会触发） ----
  auto rot_odom_pub = io->rot_odom_pub;
  pipeline->setRotRefinedOdomCallback([rot_odom_pub](const Eigen::Matrix4d& T_wl,
                                                     double stamp) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = toRosStamp(stamp);
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
    rot_odom_pub->publish(odom);
  });

  const auto logger = node.get_logger();
  pipeline->setLogCallback(
      [logger](superlivo_b::VisualPipeline::LogLevel level, const std::string& msg) {
        switch (level) {
          case superlivo_b::VisualPipeline::LogLevel::kInfo:
            RCLCPP_INFO(logger, "%s", msg.c_str());
            break;
          case superlivo_b::VisualPipeline::LogLevel::kWarn:
            RCLCPP_WARN(logger, "%s", msg.c_str());
            break;
          case superlivo_b::VisualPipeline::LogLevel::kError:
            RCLCPP_ERROR(logger, "%s", msg.c_str());
            break;
        }
      });

  // ---- 图像订阅（外部输入，仍走 ROS） ----
  const std::string image_topic =
      getOrDeclare<std::string>(node, "image_topic", "/left_camera/image");
  const bool image_qos_reliable = getOrDeclare<bool>(node, "image.qos_reliable", true);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group =
      node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto image_qos = image_qos_reliable
                       ? rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
                       : rclcpp::QoS(rclcpp::SensorDataQoS());
  const auto image_logger = node.get_logger();
  io->image_sub = node.create_subscription<sensor_msgs::msg::Image>(
      image_topic, image_qos,
      [pipeline, image_logger](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
          const cv_bridge::CvImagePtr cv_ptr =
              cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          if (!cv_ptr || cv_ptr->image.empty()) {
            return;
          }
          pipeline->submitImage(cv_ptr->image,
                                rclcpp::Time(msg->header.stamp).seconds());
        } catch (const cv_bridge::Exception& e) {
          static rclcpp::Clock throttle_clock;
          RCLCPP_WARN_THROTTLE(image_logger, throttle_clock, 5000,
                               "cv_bridge 转换失败: %s", e.what());
        }
      },
      sub_options);

  // ---- IMU 订阅（外部输入） ----
  const std::string imu_topic = getOrDeclare<std::string>(node, "imu_topic", "/livox/imu");
  io->imu_sub = node.create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [pipeline](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        const Eigen::Vector3d acc(msg->linear_acceleration.x,
                                  msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);
        const Eigen::Vector3d gyro(msg->angular_velocity.x,
                                   msg->angular_velocity.y,
                                   msg->angular_velocity.z);
        pipeline->submitImu(acc, gyro, rclcpp::Time(msg->header.stamp).seconds());
      });

  // ---- 可选：独立相机位姿源（color.cam_odom_topic） ----
  const std::string cam_odom_topic =
      getOrDeclare<std::string>(node, "color.cam_odom_topic", "");
  if (!cam_odom_topic.empty()) {
    io->cam_odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(
        cam_odom_topic, rclcpp::SensorDataQoS(),
        [pipeline](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
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
          pipeline->submitCamOdom(T_wl, rclcpp::Time(msg->header.stamp).seconds());
        });
  }

  // ---- 周期性彩色地图落盘 ----
  const double save_interval =
      getOrDeclare<double>(node, "pcd.save_interval_sec", 30.0);
  if (save_interval > 0.0) {
    io->save_timer = node.create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(save_interval)),
        [pipeline]() { pipeline->requestSaveColoredMap(); },
        node.getReportCallbackGroup());
  }

  return io;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 先声明 pipeline 容器：保证它晚于 lio 析构（lio 内持裸指针）。
  std::shared_ptr<superlivo_b::VisualPipeline> pipeline;

  // ---- Super-LIO 主干（公共库）------------------------------------------
  auto wrapper = std::make_shared<LI2Sup::ROSWrapper>();
  auto lio = std::make_shared<VisualBSuperLIO>();
  lio->setROSWrapper(wrapper);
  wrapper->setSuperLIO(lio);
  lio->init();

  // ---- 视觉支路开关 ------------------------------------------------------
  const bool embed_visual =
      getOrDeclare<bool>(*wrapper, "app.embed_visual", true);
  // app.visual_inject：是否把视觉约束经 ExternalObserve() 写回 ESKF。
  const bool inject_visual =
      getOrDeclare<bool>(*wrapper, "app.visual_inject", true);
  const double inject_max_dt =
      getOrDeclare<double>(*wrapper, "app.visual_inject_max_dt", 0.2);

  std::shared_ptr<superlivo_b::ConstraintBridge> bridge;
  if (embed_visual && inject_visual) {
    bridge = std::make_shared<superlivo_b::ConstraintBridge>();
  }
  lio->setConstraintBridge(bridge, bridge != nullptr, inject_max_dt);

  std::shared_ptr<MonolithVisualIo> io;
  if (!embed_visual) {
    RCLCPP_INFO(wrapper->get_logger(), "app.embed_visual=false：只运行纯 LIO");
  } else {
    RosParamSource param_source(*wrapper);
    pipeline = std::make_shared<superlivo_b::VisualPipeline>(param_source, bridge);
    io = setupMonolithIo(*wrapper, pipeline);
    lio->setVisualPipeline(pipeline.get());
    RCLCPP_INFO(wrapper->get_logger(),
                "Super-LIO → VisualPipeline 进程内直连（无 DDS）");
  }

  // process 定时器使用独立回调组，与 IMU/lidar 回调并行
  auto process_timer = wrapper->create_wall_timer(
      std::chrono::milliseconds(2),
      [lio]() { lio->process(); },
      wrapper->getProcessCallbackGroup());

  // 1Hz 周期任务：看门狗 + 60s 性能窗口（每 10s 顺带打印注入通路统计）
  auto report_timer = wrapper->create_wall_timer(
      std::chrono::seconds(1),
      [lio, bridge]() {
        lio->periodicTimeReport();
        static int tick = 0;
        if (bridge && (++tick % 10) == 0) {
          const auto [pushed, taken] = bridge->stats();
          RCLCPP_INFO(
              rclcpp::get_logger("visual_b_app"),
              "[注入] push=%ld take=%ld 生效=%ld 被拒=%ld 过期=%ld | 覆盖=%ld "
              "| 取用时差ms avg=%.1f absmax=%.1f | ExternalObserve次数=%ld 间隔ms avg=%.1f max=%.1f",
              pushed, taken, lio->applied(), lio->rejected(), lio->stale(),
              bridge->overwritten(), lio->takeAgeAvgMs(), lio->takeAgeAbsMaxMs(),
              lio->extCalls(), lio->extGapAvgMs(), lio->extGapMaxMs());
          // 同一趟数据内的 |age| 分布：直接读出不同门限能多接多少条约束。
          const auto& h = lio->takeAgeHist();
          const long tot = h[0] + h[1] + h[2] + h[3] + h[4];
          RCLCPP_INFO(
              rclcpp::get_logger("visual_b_app"),
              "[年龄] n=%ld <0.1=%.1f%% <0.2=%.1f%% <0.3=%.1f%% <0.4=%.1f%% >=0.4=%.1f%%",
              tot, tot ? 100.0 * h[0] / tot : 0.0, tot ? 100.0 * h[1] / tot : 0.0,
              tot ? 100.0 * h[2] / tot : 0.0, tot ? 100.0 * h[3] / tot : 0.0,
              tot ? 100.0 * h[4] / tot : 0.0);
        }
      },
      wrapper->getReportCallbackGroup());

  rclcpp::on_shutdown([lio, pipeline]() {
    lio->printTimeRecord();
    if (pipeline) {
      // 先排空队列，再**同步**停机落盘：退出时的保存不能交给异步 save 线程，
      // 长序列的大图还没写完进程就退出了，线程被杀、产物全部丢失。
      pipeline->flush();
      pipeline->shutdownAndSave();
      RCLCPP_INFO(rclcpp::get_logger("visual_b_app"), "[彩色统计] %s",
                  pipeline->statsString().c_str());
    }
  });

  // 多线程执行器：IMU / LiDAR / process / 报告 / 视觉各占一线程。
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(wrapper);
  executor.spin();

  // 断开钩子，避免 lio 线程在 pipeline 析构后仍访问它。
  lio->setVisualPipeline(nullptr);

  // 兜底：shutdown 钩子挂在全局 context 上，与主线程的退出时序不保证有序
  // （实测两者可能并发）。这里再同步保存一次；shutdownAndSave() 幂等且
  // 会让后到者阻塞到落盘完成，确保进程退出前文件已经写好。
  if (pipeline) {
    pipeline->shutdownAndSave();
  }

  rclcpp::shutdown();
  return 0;
}
