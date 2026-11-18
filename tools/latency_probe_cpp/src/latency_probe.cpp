// 轻量延时探针（C++）：只记「消息到达墙钟」与「header.stamp」，不做反序列化。
//
// 输出与 tools/latency_probe.py 完全相同的 CSV（kind,recv_ns,stamp_ns），
// 可直接交给 tools/latency_summary.py 后处理：
//   algo_ms = odom 到达 − 对应雷达帧到达
//
// 为什么必须有 C++ 版：Python 探针在重负载序列上自身会滞后——rclpy 回调里要
// 反序列化 ~0.8 MB 的 CustomMsg 并逐帧写 CSV，CPU 被映射节点抢占后队列积压，
// 实测 lidar 的 recv−stamp 抖动达 41 s、odom 达 90~110 s，algo 出现负值
// （retail/sl 中位 −2.1 s、rtk/b 中位 −1.5 s），而同期 rosbag2（C++）记录的
// recv−stamp 全程恒定在 30~46 ms。这里用 generic subscription 直接拿序列化
// 字节，只读 12 字节头，单帧开销亚微秒级，不会成为瓶颈。
//
// 用法：
//   latency_probe --lidar-topic /livox/lidar --lidar-type custom
//                 --odom-topic /lio/odom --out /path/latency.csv
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"

namespace
{

int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch()).count();
}

// std_msgs/Header 是 CustomMsg / PointCloud2 / Odometry 的首字段，CDR 布局：
//   [0..3] 封装头（b[1]==0x01 为 CDR_LE）  [4..7] sec(int32)  [8..11] nanosec(uint32)
// 只读这 12 字节，避免反序列化整帧点云。
bool read_stamp_ns(const rclcpp::SerializedMessage & msg, int64_t & stamp_ns)
{
  const auto & raw = msg.get_rcl_serialized_message();
  if (raw.buffer == nullptr || raw.buffer_length < 12) {
    return false;
  }
  const uint8_t * b = raw.buffer;
  const bool little = (b[1] != 0x00);
  auto rd32 = [](const uint8_t * p, bool le) -> uint32_t {
      return le ? (uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) |
                  (uint32_t(p[3]) << 24))
                : ((uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) |
                  uint32_t(p[3]));
    };
  const uint32_t sec = rd32(b + 4, little);
  const uint32_t nsec = rd32(b + 8, little);
  stamp_ns = int64_t(static_cast<int32_t>(sec)) * 1000000000LL + int64_t(nsec);
  return true;
}

void on_signal(int) { rclcpp::shutdown(); }

class Probe : public rclcpp::Node
{
public:
  Probe(
    const std::string & out, const std::string & lidar_topic,
    const std::string & lidar_type, const std::string & odom_topic)
  : Node("paper_latency_probe_cpp")
  {
    f_.open(out, std::ios::out | std::ios::trunc);
    f_ << "kind,recv_ns,stamp_ns\n";

    // 与 Python 版同口径：都取 BEST_EFFORT。odom 深队列只为兼容发布端；
    // C++ 回调开销可忽略，正常情况下队列不会积压（积压会在 recv 时刻体现出来）。
    const auto qos_lidar = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();
    const auto qos_odom = rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort();

    const std::string lidar_msg_type =
      (lidar_type == "custom") ? "livox_ros_driver2/msg/CustomMsg"
                               : "sensor_msgs/msg/PointCloud2";

    lidar_sub_ = create_generic_subscription(
      lidar_topic, lidar_msg_type, qos_lidar,
      [this](std::shared_ptr<rclcpp::SerializedMessage> m) {on_lidar(m);});
    odom_sub_ = create_generic_subscription(
      odom_topic, "nav_msgs/msg/Odometry", qos_odom,
      [this](std::shared_ptr<rclcpp::SerializedMessage> m) {on_odom(m);});

    flush_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {f_.flush();});

    RCLCPP_INFO(
      get_logger(), "probe(cpp): lidar=%s(%s) odom=%s -> %s",
      lidar_topic.c_str(), lidar_type.c_str(), odom_topic.c_str(), out.c_str());
  }

  void finish()
  {
    f_.flush();
    f_.close();
    fprintf(stdout, "[latency_probe] lidar=%ld odom=%ld\n", n_lidar_, n_odom_);
    fflush(stdout);
  }

private:
  void write_row(const char * kind, int64_t recv_ns, const rclcpp::SerializedMessage & msg)
  {
    int64_t stamp_ns = -1;
    if (!read_stamp_ns(msg, stamp_ns)) {
      stamp_ns = -1;
    }
    char buf[96];
    const int n = snprintf(
      buf, sizeof(buf), "%s,%lld,%lld\n", kind,
      static_cast<long long>(recv_ns), static_cast<long long>(stamp_ns));
    if (n > 0) {
      f_.write(buf, n);
    }
  }

  void on_lidar(const std::shared_ptr<rclcpp::SerializedMessage> & msg)
  {
    const int64_t t = now_ns();
    write_row("lidar", t, *msg);
    ++n_lidar_;
  }

  void on_odom(const std::shared_ptr<rclcpp::SerializedMessage> & msg)
  {
    const int64_t t = now_ns();
    write_row("odom", t, *msg);
    ++n_odom_;
  }

  std::ofstream f_;
  long n_lidar_ = 0;
  long n_odom_ = 0;
  rclcpp::GenericSubscription::SharedPtr lidar_sub_;
  rclcpp::GenericSubscription::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
};

void usage()
{
  fprintf(stderr,
          "用法: latency_probe --lidar-topic <topic> [--lidar-type custom|pc2] "
          "--odom-topic <topic> --out <csv>\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string lidar_topic, odom_topic, out, lidar_type = "pc2";
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](std::string & dst) {
        if (i + 1 >= argc) {
          usage();
          exit(2);
        }
        dst = argv[++i];
      };
    if (a == "--lidar-topic") {
      next(lidar_topic);
    } else if (a == "--lidar-type") {
      next(lidar_type);
    } else if (a == "--odom-topic") {
      next(odom_topic);
    } else if (a == "--out") {
      next(out);
    } else {
      fprintf(stderr, "未知参数: %s\n", a.c_str());
      usage();
      return 2;
    }
  }
  if (lidar_topic.empty() || odom_topic.empty() || out.empty()) {
    usage();
    return 2;
  }

  rclcpp::init(0, nullptr);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  try {
    auto node = std::make_shared<Probe>(out, lidar_topic, lidar_type, odom_topic);
    rclcpp::spin(node);
    node->finish();
  } catch (const std::exception & e) {
    fprintf(stderr, "probe(cpp) 失败: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
