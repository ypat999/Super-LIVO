// ============================================================================
// 方案 A 独立可执行入口
// ----------------------------------------------------------------------------
// 架构约定：super_lio 是公共代码库（liblio），本项目只"引用"它的资源：
//   * 通过 find_package(super_lio) + target_link_libraries(... super_lio::lio)
//     在编译期链接；
//   * 通过继承 LI2Sup::SuperLIO 保留代码级扩展点 ExternalObserve()。
// 因此本可执行文件自身就是完整方案：一个进程内同时运行
//   (1) Super-LIO 主干（建图/配准/输出，全部来自公共库）
//   (2) 方案 A 视觉支路（KLT 前端/深度关联/重投影约束/更新，全部在本包内）
// 不再需要先单独启动 super_lio_node 再启动 visual_a_node。
//
// 方案 A 定位是"轻量、只做可观测性增强"，首版不向 ESKF 注入约束；
// 若后续需要，同样在 ExternalObserve() 内调用 eskf()->UpdateObserveFull(...)。
// ============================================================================

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lio/super_lio.h"
#include "ros/ROSWrapper.h"

#include "super_livo_visual_a/node/visual_a_node.hpp"

namespace {

/// 方案 A 的 LIO 主体：只覆写公共库预留的通用扩展点（当前不注入）。
class VisualASuperLIO : public LI2Sup::SuperLIO {
 public:
  void ExternalObserve() override {}
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // ---- Super-LIO 主干（公共库）------------------------------------------
  auto wrapper = std::make_shared<LI2Sup::ROSWrapper>();
  auto lio = std::make_shared<VisualASuperLIO>();
  lio->setROSWrapper(wrapper);
  wrapper->setSuperLIO(lio);
  lio->init();

  rclcpp::on_shutdown([lio]() { lio->printTimeRecord(); });

  // ---- 方案 A 视觉支路（本包）-------------------------------------------
  // app.embed_visual=false 时只跑纯 LIO：用于"关闭注入逐位回归"的口径。
  const bool embed_visual =
      wrapper->declare_parameter<bool>("app.embed_visual", true);
  std::shared_ptr<superlivo_a::VisualANode> visual;
  if (embed_visual) {
    visual = std::make_shared<superlivo_a::VisualANode>();
  }

  auto process_timer = wrapper->create_wall_timer(
      std::chrono::milliseconds(2),
      [lio]() { lio->process(); },
      wrapper->getProcessCallbackGroup());

  auto report_timer = wrapper->create_wall_timer(
      std::chrono::seconds(1),
      [lio]() { lio->periodicTimeReport(); },
      wrapper->getReportCallbackGroup());

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(wrapper);
  if (visual) {
    executor.add_node(visual);
  }
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
