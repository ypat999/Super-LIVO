// nn_color_transfer_app：独立运行的最近邻染色工具（不依赖 ROS）。
//
// 对未染色点云的每个点，在彩色点云中查找最近点：距离 <= max_dist（默认 10 cm）
// 则借用该彩色点的颜色写入结果；超界的点视为仍未染色，不写入输出 PCD。
// 与方案 A 退出时的 super_livo_visual_a_nn_colored.pcd 走同一套核心逻辑
// （superlivo_a::transferNNColor），也可离线对任意两份历史 PCD 单独补做。
//
// 用法:
//   nn_color_transfer_app <colored.pcd> <uncolored.pcd> <out.pcd> [max_dist=0.10]
#include <cstdio>
#include <cstdlib>
#include <string>

#include "super_livo_visual_a/color/nn_color_transfer.hpp"

namespace {

void usage(const char* argv0) {
  std::fprintf(stderr,
               "用法: %s <colored.pcd> <uncolored.pcd> <out.pcd> [max_dist]\n"
               "  colored.pcd    彩色源点云（必须含 RGB 字段）\n"
               "  uncolored.pcd  待染色点云（无 RGB 字段时按 XYZI 读入置灰）\n"
               "  out.pcd        输出（只含半径内借色成功的点，超界点不写入）\n"
               "  max_dist       最近邻借色距离上限，米；默认 0.10（10 cm）\n",
               argv0);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4 || argc > 5) {
    usage(argv[0]);
    return 1;
  }
  double max_dist = 0.10;
  if (argc == 5) {
    max_dist = std::strtod(argv[4], nullptr);
    if (!(max_dist > 0.0)) {
      std::fprintf(stderr, "max_dist 必须为正数（米），收到: %s\n", argv[4]);
      return 1;
    }
  }

  superlivo_a::NNColorTransferStats stats;
  std::string err;
  if (!superlivo_a::transferNNColorFiles(argv[1], argv[2], argv[3], max_dist, stats, err)) {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 1;
  }
  std::printf("NN 染色完成: %zu/%zu 点在 %.3f m 内借色，已写出 %s\n", stats.colored,
              stats.total, max_dist, argv[3]);
  return 0;
}
