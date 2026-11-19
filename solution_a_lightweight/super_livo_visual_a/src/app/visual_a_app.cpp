// ============================================================================
// 方案 A 独立可执行入口
// ----------------------------------------------------------------------------
// 架构约定：super_lio 是公共代码库（liblio），本项目只"引用"它的资源：
//   * 通过 find_package(super_lio) + target_link_libraries(... super_lio::lio)
//     在编译期链接；
//   * 通过继承 LI2Sup::SuperLIO 保留代码级扩展点 ExternalObserve()。
//
// 单体化（app.lio_source=内部推送，默认）：
//   * 整个进程只有一个 rclcpp::Node（ROSWrapper）；
//   * Super-LIO 的位姿/点云/退化签名通过 SuperLIO::onLioOutput / onDenseCloud
//     钩子直接推给 VisualPipeline，**不经过任何 DDS 话题**；
//   * 图像仍然从 ROS 话题订阅（相机是外部输入，必须走 ROS）；
//   * 视觉输出（colored_cloud / odom / path）由同一节点对外发布。
//
// 对照观察窗口（app.lio_source=topic）：
//   * 仍然创建旧的 VisualANode 子节点，走 lio/odom、lio/cloud_world 等话题，
//     用于 S0（DDS 回环）与 S1（进程内）逐位对比，确认无误后再删除。
// ============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lio/super_lio.h"
#include "ros/ROSWrapper.h"

#include "super_livo_visual_a/node/visual_a_node.hpp"
#include "super_livo_visual_a/pipeline/visual_pipeline.hpp"

namespace {

using superlivo_a::vp;

/// 秒 -> ROS 时间戳（bag 的时间戳为传感器时钟，秒值 ~9.4e8，仍在 int32 内）。
builtin_interfaces::msg::Time toRosStamp(double sec) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(std::floor(sec));
  stamp.nanosec = static_cast<uint32_t>((sec - static_cast<double>(stamp.sec)) * 1e9);
  return stamp;
}

/// 读取参数：已声明则取值，否则按默认值声明。
/// 之所以不直接 declare：本节点复用 ROSWrapper（Super-LIO 参数已在 init 里声明），
/// 个别参数名可能与 LIO 侧重合，直接 declare 会抛 ParameterAlreadyDeclared。
/// name 必须是 super_livo_visual.<参数名>（调用点用 vp 生成），与节点路径完全一致。
template <typename T>
T getOrDeclare(rclcpp::Node& node, const std::string& name, const T& def) {
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, def);
}

/// 从 nav_msgs::Odometry 取 T_wl；四元数非法时返回 false。
bool odomToMatrix(const nav_msgs::msg::Odometry& msg, Eigen::Matrix4d& T_wl) {
  const auto& pose = msg.pose.pose;
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                       pose.orientation.z);
  if (q.norm() < 1e-9) {
    return false;
  }
  q.normalize();
  T_wl.setIdentity();
  T_wl.block<3, 3>(0, 0) = q.toRotationMatrix();
  T_wl(0, 3) = pose.position.x;
  T_wl(1, 3) = pose.position.y;
  T_wl(2, 3) = pose.position.z;
  return true;
}

/// 把 VisualANode::loadParameters() 的声明映射成普通配置结构体。
/// 参数名与默认值与原节点逐项一致，因此同一份 yaml 可以直接复用。
superlivo_a::VisualPipelineOptions loadPipelineOptions(rclcpp::Node& node) {
  superlivo_a::VisualPipelineOptions opt;

  // ---- 相机内参 / 畸变 / 外参 ----
  opt.intrinsics.width = getOrDeclare<int>(node, vp("cam_width"), 640);
  opt.intrinsics.height = getOrDeclare<int>(node, vp("cam_height"), 512);
  opt.intrinsics.fx = getOrDeclare<double>(node, vp("cam_fx"), 646.78);
  opt.intrinsics.fy = getOrDeclare<double>(node, vp("cam_fy"), 646.66);
  opt.intrinsics.cx = getOrDeclare<double>(node, vp("cam_cx"), 313.46);
  opt.intrinsics.cy = getOrDeclare<double>(node, vp("cam_cy"), 261.40);
  opt.distortion = getOrDeclare<std::vector<double>>(
      node, vp("cam_d"), std::vector<double>{-0.076160, 0.123001, -0.00113, 0.000251});
  opt.Rcl = getOrDeclare<std::vector<double>>(
      node, vp("Rcl"), std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
  opt.Pcl = getOrDeclare<std::vector<double>>(node, vp("Pcl"), std::vector<double>{0, 0, 0});
  opt.img_time_offset = getOrDeclare<double>(node, vp("img_time_offset"), 0.1);

  // ---- 前端 ----
  opt.klt.max_features = getOrDeclare<int>(node, vp("klt.max_features"), 150);
  opt.klt.pyramid_levels = getOrDeclare<int>(node, vp("klt.pyramid_levels"), 3);
  opt.klt.min_distance = getOrDeclare<double>(node, vp("klt.min_distance"), 10.0);
  opt.klt.fb_error_threshold = getOrDeclare<double>(node, vp("klt.fb_error_threshold"), 1.0);
  opt.klt.min_track_length = getOrDeclare<int>(node, vp("klt.min_track_length"), 3);
  opt.klt.trace_window = getOrDeclare<int>(node, vp("klt.trace_window"), 5);
  opt.klt.use_fast = getOrDeclare<bool>(node, vp("klt.use_fast"), true);
  opt.klt.fast_threshold = getOrDeclare<int>(node, vp("klt.fast_threshold"), 20);
  opt.klt.quality_level = getOrDeclare<double>(node, vp("klt.quality_level"), 0.01);
  opt.klt.lk_window = getOrDeclare<int>(node, vp("klt.lk_window"), 21);

  // ---- 深度关联 ----
  opt.depth.pixel_radius = getOrDeclare<int>(node, vp("depth.pixel_radius"), 3);
  opt.depth.search_radius = getOrDeclare<double>(node, vp("depth.search_radius"), 0.3);
  opt.depth.min_depth = getOrDeclare<double>(node, vp("depth.min_depth"), 0.5);
  opt.depth.max_depth = getOrDeclare<double>(node, vp("depth.max_depth"), 80.0);
  opt.depth.max_neighbors = getOrDeclare<int>(node, vp("depth.max_neighbors"), 5);
  opt.depth.anchor_window = getOrDeclare<int>(node, vp("depth.anchor_window"), 5);

  // ---- 重投影 ----
  opt.reproj.huber_delta = getOrDeclare<double>(node, vp("reproj.huber_delta"), 1.0);
  opt.reproj.min_constraints = getOrDeclare<int>(node, vp("reproj.min_constraints"), 8);
  opt.reproj.max_pixel_error = getOrDeclare<double>(node, vp("reproj.max_pixel_error"), 8.0);
  opt.reproj.sigma_pix = getOrDeclare<double>(node, vp("reproj.sigma_pix"), 1.5);
  opt.reproj.sigma_anchor = getOrDeclare<double>(node, vp("reproj.sigma_anchor"), 0.05);

  // ---- 门控 ----
  opt.gate.min_tracked_features = getOrDeclare<int>(node, vp("gate.min_tracked_features"), 30);
  opt.gate.max_reprojection_rmse = getOrDeclare<double>(node, vp("gate.max_reprojection_rmse"), 2.0);
  opt.gate.min_spread = getOrDeclare<double>(node, vp("gate.min_spread"), 0.15);
  opt.gate.max_condition = getOrDeclare<double>(node, vp("gate.max_condition"), 1.0e3);
  opt.gate.assist_weight = getOrDeclare<double>(node, vp("gate.assist_weight"), 0.3);
  opt.gate.high_weight = getOrDeclare<double>(node, vp("gate.high_weight"), 0.8);
  opt.gate.assist_degeneracy = getOrDeclare<double>(node, vp("gate.assist_degeneracy"), 0.3);
  opt.gate.high_degeneracy = getOrDeclare<double>(node, vp("gate.high_degeneracy"), 0.7);
  opt.gate_cond_lo = getOrDeclare<double>(node, vp("gate.cond_lo"),
                                          superlivo_common::DegeneracyScale{}.cond_lo);
  opt.gate_cond_hi = getOrDeclare<double>(node, vp("gate.cond_hi"),
                                          superlivo_common::DegeneracyScale{}.cond_hi);
  opt.degen_timeout = getOrDeclare<double>(node, vp("gate.degen_timeout"),
                                           superlivo_common::DegeneracyScale{}.timeout_sec);

  // ---- 视觉更新 ----
  opt.update.enable = getOrDeclare<bool>(node, vp("update.enable"), true);
  opt.update.max_dx_norm = getOrDeclare<double>(node, vp("update.max_dx_norm"), 0.5);
  opt.update.prior_rot_sigma = getOrDeclare<double>(node, vp("update.prior_rot_sigma"), 0.02);
  opt.update.prior_pos_sigma = getOrDeclare<double>(node, vp("update.prior_pos_sigma"), 0.05);
  opt.update.directional_prior_en =
      getOrDeclare<bool>(node, vp("update.directional_prior_en"), true);
  opt.update.prior_pos_sigma_healthy =
      getOrDeclare<double>(node, vp("update.prior_pos_sigma_healthy"), 0.005);
  opt.update.prior_pos_sigma_degen =
      getOrDeclare<double>(node, vp("update.prior_pos_sigma_degen"), 1.0);
  opt.offset_decay_en = getOrDeclare<bool>(node, vp("update.offset_decay_en"), true);
  opt.offset_decay_rate = getOrDeclare<double>(node, vp("update.offset_decay_rate"), 1.0);
  opt.reinject = getOrDeclare<bool>(node, vp("update.reinject"), true);

  // ---- 彩色点云 ----
  opt.color_use_own_odom = getOrDeclare<bool>(node, vp("color.use_own_odom"), false);
  opt.use_exposure_pose = getOrDeclare<bool>(node, vp("visual.use_exposure_pose"), true);
  opt.colorizer.sync.max_offset = getOrDeclare<double>(node, vp("color.sync_max_offset"), 0.02);
  opt.colorizer.sync.use_exposure_mid =
      getOrDeclare<bool>(node, vp("color.use_exposure_mid"), true);
  opt.colorizer.occlusion.depth_tolerance =
      getOrDeclare<double>(node, vp("color.depth_tolerance"), 0.1);
  opt.colorizer.occlusion.check_depth_edges =
      getOrDeclare<bool>(node, vp("color.check_depth_edges"), true);
  opt.colorizer.occlusion.edge_threshold =
      getOrDeclare<double>(node, vp("color.edge_threshold"), 0.5);
  opt.colorizer.rgb.bilinear = getOrDeclare<bool>(node, vp("color.bilinear"), true);
  opt.colorizer.rgb.keep_best_quality =
      getOrDeclare<bool>(node, vp("color.keep_best_quality"), true);
  opt.colorizer.rgb.min_quality = getOrDeclare<float>(node, vp("color.min_quality"), 0.2F);
  opt.colorizer.rgb.patch_radius = getOrDeclare<int>(node, vp("color.patch_radius"), 0);
  opt.colorizer.rgb.patch_color_tol =
      getOrDeclare<float>(node, vp("color.patch_color_tol"), 20.0F);

  // ---- 落盘 ----
  opt.pcd_save_dir =
      getOrDeclare<std::string>(node, vp("pcd.save_dir"), "/home/ywj/slam_data/pcd");
  opt.max_colored_map_points = static_cast<std::size_t>(
      getOrDeclare<int>(node, vp("pcd.max_points"), 20000000));
  opt.median_voxel_size =
      static_cast<float>(getOrDeclare<double>(node, vp("pcd.median_voxel_size"), 0.01));

  // ---- 轨迹 / 图像落盘（与 FAST-LIVO2 的 evo / image_save 同口径） ----
  // 轨迹：<evo.save_dir>/<evo.seq_name>.txt，TUM 格式，每行
  //       timestamp tx ty tz qx qy qz qw（camera_init 系下 LiDAR 位姿）。
  // 图像：<image_save.dir>/<timestamp>.png，时间戳固定 6 位小数。
  // 目录留空时分别落到 pcd.save_dir 与 <pcd.save_dir>/image。
  opt.pose_output_en = getOrDeclare<bool>(node, vp("evo.pose_output_en"), false);
  opt.pose_save_dir = getOrDeclare<std::string>(node, vp("evo.save_dir"), "");
  opt.pose_seq_name = getOrDeclare<std::string>(node, vp("evo.seq_name"), "scene_move");
  opt.pose_save_interval = getOrDeclare<int>(node, vp("evo.interval"), 1);
  opt.img_save_en = getOrDeclare<bool>(node, vp("image_save.img_save_en"), false);
  opt.img_save_dir = getOrDeclare<std::string>(node, vp("image_save.dir"), "");
  opt.img_save_interval = getOrDeclare<int>(node, vp("image_save.interval"), 1);

  // 启动自检：分支式 YAML 一旦键名/层级写错会静默退回默认值，这里把关键项打出来。
  RCLCPP_INFO(node.get_logger(),
              "视觉参数：相机 %dx%d fx=%.2f fy=%.2f cx=%.2f cy=%.2f "
              "img_time_offset=%.3f pcd=%s",
              opt.intrinsics.width, opt.intrinsics.height, opt.intrinsics.fx,
              opt.intrinsics.fy, opt.intrinsics.cx, opt.intrinsics.cy,
              opt.img_time_offset, opt.pcd_save_dir.c_str());
  return opt;
}

/// 视觉流水线的 ROS I/O 适配层：只负责"话题 <-> 普通类型"的转换。
/// 所有视觉处理都在 VisualPipeline 的内部工作线程完成，这里不做任何算法。
struct MonolithVisualIo {
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cam_odom_sub;
  rclcpp::TimerBase::SharedPtr save_timer;
  nav_msgs::msg::Path path_msg;
};

/// 方案 A 的 LIO 主体：只覆写公共库预留的通用扩展点。
/// 单体模式下用它把 LIO 输出"推"给视觉流水线（进程内，无 DDS）。
class VisualASuperLIO : public LI2Sup::SuperLIO {
 public:
  void setPipeline(superlivo_a::VisualPipeline* pipeline) {
    pipeline_.store(pipeline);
  }

 protected:
  void onLioOutput(const OutputData& data) override {
    auto* pipeline = pipeline_.load();
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
        points.emplace_back(p.x, p.y, p.z);
      }
      pipeline->submitWorldCloud(points);
    }

    // 退化签名：等价于原 /lio/degeneracy 话题载荷（该话题在原树中并未发布）。
    const DegeneracySnapshot snap = degeneracySnapshot();
    if (snap.ok) {
      superlivo_common::DegeneracySignature sig;
      // 快照里的 cond 就是 trans_only 模式的平移块条件数（cond_t）；
      // 6 维条件数在单体内没有单独保留，这里同步填入，仅用于 valid() 判定。
      sig.cond6 = snap.cond;
      sig.cond_t = snap.cond;
      sig.axes = snap.axes;
      sig.active = snap.active;
      const Eigen::Vector3d dir(snap.dir_w.cast<double>());
      sig.dir = dir.norm() > 1e-6 ? dir.normalized() : Eigen::Vector3d::Zero();
      pipeline->submitDegeneracy(sig);
    }
  }

  void onDenseCloud(const BASIC::CloudPtr& cloud, double stamp) override {
    auto* pipeline = pipeline_.load();
    if (pipeline == nullptr || !cloud) {
      return;
    }
    std::vector<Eigen::Vector3f> points;
    points.reserve(cloud->size());
    for (const auto& p : cloud->points) {
      points.emplace_back(p.x, p.y, p.z);
    }
    pipeline->submitBodyCloud(points, stamp);
  }

 private:
  std::atomic<superlivo_a::VisualPipeline*> pipeline_{nullptr};
};

/// 建立订阅/发布器并接上流水线回调。
std::shared_ptr<MonolithVisualIo> setupMonolithIo(
    LI2Sup::ROSWrapper& node, std::shared_ptr<superlivo_a::VisualPipeline> pipeline) {
  auto io = std::make_shared<MonolithVisualIo>();

  io->colored_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "super_livo_visual_a/colored_cloud", rclcpp::SensorDataQoS());
  io->odom_pub = node.create_publisher<nav_msgs::msg::Odometry>(
      "super_livo_visual_a/odom", rclcpp::SensorDataQoS());
  io->path_pub = node.create_publisher<nav_msgs::msg::Path>(
      "super_livo_visual_a/path", rclcpp::SensorDataQoS());
  io->path_msg.header.frame_id = "world";

  // ---- 输出回调（在流水线工作线程里执行，rclcpp 发布是线程安全的） ----
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
        // 点仍在 LiDAR 系；Super-LIO 把该坐标系命名为 "imu"（world->imu 有 TF）。
        msg.header.frame_id = "imu";
        colored_pub->publish(msg);
      });

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

  const auto logger = node.get_logger();
  pipeline->setLogCallback(
      [logger](superlivo_a::VisualPipeline::LogLevel level, const std::string& msg) {
        switch (level) {
          case superlivo_a::VisualPipeline::LogLevel::kInfo:
            RCLCPP_INFO(logger, "%s", msg.c_str());
            break;
          case superlivo_a::VisualPipeline::LogLevel::kWarn:
            RCLCPP_WARN(logger, "%s", msg.c_str());
            break;
          case superlivo_a::VisualPipeline::LogLevel::kError:
            RCLCPP_ERROR(logger, "%s", msg.c_str());
            break;
        }
      });

  // ---- 图像订阅（外部输入，仍走 ROS） ----
  const std::string image_topic =
      getOrDeclare<std::string>(node, vp("image_topic"), "/left_camera/image");
  const bool image_qos_reliable = getOrDeclare<bool>(node, vp("image.qos_reliable"), true);
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

  // ---- 可选：独立相机位姿源（color.cam_odom_topic） ----
  const std::string cam_odom_topic =
      getOrDeclare<std::string>(node, vp("color.cam_odom_topic"), "");
  if (!cam_odom_topic.empty()) {
    io->cam_odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(
        cam_odom_topic, rclcpp::SensorDataQoS(),
        [pipeline](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
          Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
          if (odomToMatrix(*msg, T_wl)) {
            pipeline->submitCamOdom(T_wl, rclcpp::Time(msg->header.stamp).seconds());
          }
        });
  }

  // ---- 周期性彩色地图落盘 ----
  const double save_interval =
      getOrDeclare<double>(node, vp("pcd.save_interval_sec"), 30.0);
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

  // 先声明 pipeline 容器：保证它晚于 lio 析构（lio 内持 raw 指针）。
  std::shared_ptr<superlivo_a::VisualPipeline> pipeline;

  // ---- Super-LIO 主干（公共库）------------------------------------------
  auto wrapper = std::make_shared<LI2Sup::ROSWrapper>();
  auto lio = std::make_shared<VisualASuperLIO>();
  lio->setROSWrapper(wrapper);
  wrapper->setSuperLIO(lio);
  lio->init();

  // ---- 选择 LIO 数据来源 -------------------------------------------------
  // internal：进程内钩子直推（默认，无 DDS 回环）
  // topic   ：保留旧 VisualANode 子节点，走 lio/* 话题（S0 对照，验证后删除）
  const bool embed_visual = getOrDeclare<bool>(*wrapper, vp("embed"), true);
  const std::string lio_source =
      getOrDeclare<std::string>(*wrapper, vp("lio_source"), "internal");

  std::shared_ptr<superlivo_a::VisualANode> visual_node;
  std::shared_ptr<MonolithVisualIo> io;
  if (!embed_visual) {
    RCLCPP_INFO(wrapper->get_logger(), "app.embed_visual=false：只运行纯 LIO");
  } else if (lio_source == "topic") {
    RCLCPP_WARN(wrapper->get_logger(),
                "app.lio_source=topic：使用旧 VisualANode（DDS 回环），仅用于对照验证");
    visual_node = std::make_shared<superlivo_a::VisualANode>();
  } else {
    if (lio_source != "internal") {
      RCLCPP_WARN(wrapper->get_logger(), "未知 app.lio_source='%s'，回退 internal",
                  lio_source.c_str());
    }
    pipeline = std::make_shared<superlivo_a::VisualPipeline>(loadPipelineOptions(*wrapper));
    io = setupMonolithIo(*wrapper, pipeline);
    lio->setPipeline(pipeline.get());
    RCLCPP_INFO(wrapper->get_logger(),
                "app.lio_source=internal：Super-LIO → VisualPipeline 进程内直连（无 DDS）");
  }

  auto process_timer = wrapper->create_wall_timer(
      std::chrono::milliseconds(2),
      [lio]() { lio->process(); },
      wrapper->getProcessCallbackGroup());

  auto report_timer = wrapper->create_wall_timer(
      std::chrono::seconds(1),
      [lio]() { lio->periodicTimeReport(); },
      wrapper->getReportCallbackGroup());

  rclcpp::on_shutdown([lio, pipeline]() {
    lio->printTimeRecord();
    if (pipeline) {
      // bag 结束时队列里可能还有等图像/等位姿的帧，尽量补完再落盘。
      pipeline->flush();
      pipeline->requestSaveColoredMap();
      pipeline->flush();
      RCLCPP_INFO(rclcpp::get_logger("visual_a_app"), "[彩色统计] %s",
                  pipeline->statsString().c_str());
    }
  });

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(wrapper);
  if (visual_node) {
    executor.add_node(visual_node);
  }
  executor.spin();

  // 断开钩子，避免 lio 线程在 pipeline 析构后仍访问它。
  lio->setPipeline(nullptr);

  rclcpp::shutdown();
  return 0;
}
