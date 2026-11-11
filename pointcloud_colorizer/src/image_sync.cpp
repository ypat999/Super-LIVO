#include "pointcloud_colorizer/image_sync.hpp"

#include <cmath>

namespace colorizer {

ImageSync::ImageSync(const Options& options) : options_(options) {}

TimedImage ImageSync::match(const std::vector<TimedImage>& images,
                            double sweep_start,
                            double sweep_end) const {
  TimedImage best;
  double best_offset = 0.0;

  // 目标时刻：优先取扫描窗口中点，使图像曝光中点尽量贴近扫描的"平均采集时刻"。
  const double target = 0.5 * (sweep_start + sweep_end);

  for (const auto& img : images) {
    if (!img.valid || img.image.empty()) {
      continue;
    }
    // 优先使用曝光中点；缺失时退回到驱动到达时间。
    double stamp = -1.0;
    if (options_.use_exposure_mid && img.exposure_mid_stamp > 0.0) {
      stamp = img.exposure_mid_stamp;
    } else if (img.arrival_stamp > 0.0) {
      stamp = img.arrival_stamp;
    }
    if (stamp < 0.0) {
      continue;  // 没有任何可用时间戳
    }

    // 偏移量：先看是否落在扫描窗口内（偏移 0），否则取到最近端点的距离。
    double offset = 0.0;
    if (stamp < sweep_start) {
      offset = sweep_start - stamp;
    } else if (stamp > sweep_end) {
      offset = stamp - sweep_end;
    }
    if (offset > options_.max_offset) {
      continue;  // 超出允许的时间差
    }

    // 同为窗口内时，按到窗口中点的距离择优。
    const double distance = std::fabs(stamp - target);

    if (!best.valid || offset < best_offset ||
        (offset == best_offset && distance < std::fabs(
            (options_.use_exposure_mid ? best.exposure_mid_stamp : best.arrival_stamp) - target))) {
      best = img;
      best_offset = offset;
    }
  }

  return best;
}

const ImageSync::Options& ImageSync::options() const { return options_; }

}  // namespace colorizer
