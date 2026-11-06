#include "pointcloud_colorizer/image_sync.hpp"

namespace colorizer {

ImageSync::ImageSync(const Options& options) : options_(options) {}

TimedImage ImageSync::match(const std::vector<TimedImage>& /*images*/,
                            double /*sweep_start*/,
                            double /*sweep_end*/) const {
  // TODO(阶段 1): 以曝光中点时间为主键，选择与扫描窗口重叠且偏移最小的图像
  return {};
}

const ImageSync::Options& ImageSync::options() const { return options_; }

}  // namespace colorizer
