// 方案 B 对话题版入口：只跑视觉节点，LIO 由外部 super_lio_node 提供。
// 完整独立方案请用同进程入口 visual_b_app。

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "super_livo_visual_b/node/visual_b_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<superlivo_b::VisualBNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
