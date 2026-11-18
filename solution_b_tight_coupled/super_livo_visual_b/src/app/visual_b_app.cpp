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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "lio/ESKF.h"
#include "lio/params.h"
#include "lio/super_lio.h"
#include "ros/ROSWrapper.h"

#include "super_livo_visual_b/bridge/constraint_bridge.hpp"
#include "super_livo_visual_b/node/visual_b_node.hpp"

namespace {

/// 把 YAML 分支 <prefix>.<name> 的原始参数覆盖去掉前缀，作为 VisualBNode 的
/// 参数覆盖注入。这样配置文件里视觉参数可以统一挂在
/// super_lio → ros__parameters → super_livo_visual_b 下，而节点内部仍然按扁平
/// 参数名（cam_fx / pcd.save_dir ...）声明与读取，不需要改造节点。
rclcpp::NodeOptions visualNodeOptionsFromBranch(rclcpp::Node& source,
                                                const std::string& prefix) {
  rclcpp::NodeOptions options;
  const auto& overrides =
      source.get_node_parameters_interface()->get_parameter_overrides();
  const std::string head = prefix + ".";
  std::vector<rclcpp::Parameter> params;
  for (const auto& kv : overrides) {
    if (kv.first.compare(0, head.size(), head) == 0) {
      params.emplace_back(kv.first.substr(head.size()), kv.second);
    }
  }
  options.parameter_overrides(params);
  return options;
}

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
  /// 探针：ExternalObserve 的调用次数 / 间隔（= LIO 雷达帧处理节奏），以及
  /// 约束被取用时的时差（>0 表示约束落后于 ESKF）。
  long extCalls() const { return ext_calls_; }
  double extGapAvgMs() const { return gap_n_ > 0 ? gap_sum_ms_ / static_cast<double>(gap_n_) : 0.0; }
  double extGapMaxMs() const { return gap_max_ms_; }
  double takeAgeAvgMs() const { return age_n_ > 0 ? 1000.0 * age_sum_ / static_cast<double>(age_n_) : 0.0; }
  double takeAgeAbsMaxMs() const { return 1000.0 * age_abs_max_; }
  /// 取用时的 |age| 直方图：<0.1 / <0.2 / <0.3 / <0.4 / ≥0.4 s。
  /// 阈值放宽的边际收益完全由这个分布决定（同一趟数据内比较，避开逐次波动）。
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

 private:
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
    // 视觉参数统一挂在 YAML 的 super_lio → ros__parameters → super_livo_visual_b
    // 分支下；这里把该分支的覆盖去掉前缀后注入 VisualBNode，节点内部仍按原参数名
    // （cam_fx / pcd.save_dir ...）声明与读取。
    visual = std::make_shared<superlivo_b::VisualBNode>(
        visualNodeOptionsFromBranch(*wrapper, "super_livo_visual_b"));
    visual->setConstraintBridge(bridge);  // 为空时节点行为与原先一致

    // /lio/odom 发布的是 base_link 系（LIO 已施加 lio.extrinsic.odom_robo），而 B 的
    // 估计全在 IMU 系：把 LIO 参数里同一份外参交给节点，由 onOdom 换回 IMU 系。
    Eigen::Matrix4d T_imu_body = Eigen::Matrix4d::Identity();
    T_imu_body.block<3, 3>(0, 0) = LI2Sup::g_odom_robo.R_.template cast<double>();
    T_imu_body.block<3, 1>(0, 3) = LI2Sup::g_odom_robo.t_.template cast<double>();
    visual->setOdomBodyExtrinsic(T_imu_body);
    RCLCPP_INFO(rclcpp::get_logger("visual_b_app"),
                "[odom] odom_robo t=(%.4f,%.4f,%.4f) ⇒ base_link 位姿将换回 IMU 系",
                T_imu_body(0, 3), T_imu_body(1, 3), T_imu_body(2, 3));
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
