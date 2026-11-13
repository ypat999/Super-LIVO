// 方案 A 对话题版入口：只跑视觉节点，LIO 由外部 super_lio_node 提供。
// 完整独立方案请用同进程入口 visual_a_app。

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "super_livo_visual_a/node/visual_a_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<superlivo_a::VisualANode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
