#include "super_livo_visual_b/constraint/photometric_constraint.hpp"

namespace superlivo_b {

PhotometricConstraint::PhotometricConstraint(const Options& options) : options_(options) {}

std::vector<PhotometricResidual> PhotometricConstraint::build(
    const cv::Mat& /*image*/,
    const std::vector<FeatureTrack>& /*tracks*/) const {
  // TODO(阶段 B4): 可选能力，借鉴 FAST-LIVO2 的 Patch 光度残差
  return {};
}

const PhotometricConstraint::Options& PhotometricConstraint::options() const { return options_; }

}  // namespace superlivo_b
