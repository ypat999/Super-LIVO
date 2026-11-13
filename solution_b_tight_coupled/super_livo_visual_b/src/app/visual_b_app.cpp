// ============================================================================
// 方案 B 独立可执行入口
// ----------------------------------------------------------------------------
// 架构约定：super_lio 是公共代码库（liblio），本项目只"引用"它的资源：
//   * 通过 find_package(super_lio) + target_link_libraries(... super_lio::lio)
//     在编译期链接；
//   * 通过继承 LI2Sup::SuperLIO 并 override ExternalObserve()，在代码级
//     （而非话题级）把视觉约束写回 ESKF。
// 因此本可执行文件自身就是完整方案：一个进程内同时运行
//   (1) Super-LIO 主干（建图/配准/输出，全部来自公共库）
//   (2) 方案 B 视觉支路（前端/特征图/约束/统一估计器，全部在本包内）
// 不再需要先单独启动 super_lio_node 再启动 visual_b_node。
//
// 说明：视觉支路仍通过本进程内的话题（/lio/odom、/lio/cloud_world 等）取
// LIO 输出，这不是"约束回灌"——约束写回走 ExternalObserve() 的直接函数调用。
// ============================================================================

#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lio/ESKF.h"
#include "lio/super_lio.h"
#include "ros/ROSWrapper.h"

#include "super_livo_visual_b/bridge/constraint_bridge.hpp"
#include "super_livo_visual_b/node/visual_b_node.hpp"

namespace {

/// 方案 B 的 LIO 主体：只覆写公共库预留的通用扩展点。
/// 约束写回统一放在这里，super_lio 内部不出现任何视觉概念。
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

  long applied() const { return applied_; }
  long rejected() const { return rejected_; }
  long stale() const { return stale_; }

  /// 每帧雷达观测（含信任域裁决）之后、建图与输出之前调用。
  /// 视觉约束通过 eskf()->UpdateObserveFull(H, r, R) 在此写入。
  void ExternalObserve() override {
    if (!inject_enable_ || !bridge_) {
      return;
    }
    superlivo_b::VisualObs obs;
    if (!bridge_->take(obs)) {
      return;
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

 private:
  std::shared_ptr<superlivo_b::ConstraintBridge> bridge_;
  bool inject_enable_{false};
  double inject_max_dt_{0.2};
  long applied_{0};
  long rejected_{0};
  long stale_{0};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // ---- Super-LIO 主干（公共库）------------------------------------------
  auto wrapper = std::make_shared<LI2Sup::ROSWrapper>();
  auto lio = std::make_shared<VisualBSuperLIO>();
  lio->setROSWrapper(wrapper);
  wrapper->setSuperLIO(lio);
  lio->init();

  rclcpp::on_shutdown([lio]() { lio->printTimeRecord(); });

  // ---- 方案 B 视觉支路（本包）-------------------------------------------
  // app.embed_visual=false 时只跑纯 LIO：用于"关闭注入逐位回归"的口径，
  // 排除视觉回调抢占 CPU 对 process 时序的任何影响。
  const bool embed_visual =
      wrapper->declare_parameter<bool>("app.embed_visual", true);
  // app.visual_inject：是否把视觉约束经 ExternalObserve() 写回 ESKF。
  // 关闭（或 bridge 为空）时整条链路与纯 LIO 逐位一致。
  const bool inject_visual =
      wrapper->declare_parameter<bool>("app.visual_inject", true);
  const double inject_max_dt =
      wrapper->declare_parameter<double>("app.visual_inject_max_dt", 0.2);

  std::shared_ptr<superlivo_b::ConstraintBridge> bridge;
  if (embed_visual && inject_visual) {
    bridge = std::make_shared<superlivo_b::ConstraintBridge>();
  }
  lio->setConstraintBridge(bridge, bridge != nullptr, inject_max_dt);

  std::shared_ptr<superlivo_b::VisualBNode> visual;
  if (embed_visual) {
    visual = std::make_shared<superlivo_b::VisualBNode>();
    visual->setConstraintBridge(bridge);  // 为空时节点行为与原先一致
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
          RCLCPP_INFO(rclcpp::get_logger("visual_b_app"),
                      "[注入] push=%ld take=%ld 生效=%ld 被拒=%ld 过期=%ld",
                      pushed, taken, lio->applied(), lio->rejected(), lio->stale());
        }
      },
      wrapper->getReportCallbackGroup());

  // 多线程执行器：IMU / LiDAR / process / 报告 / 视觉各占一线程。
  // 视觉节点的回调仍在其默认互斥回调组内串行（与原先单线程 spin 行为一致）。
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(wrapper);
  if (visual) {
    executor.add_node(visual);
  }
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
