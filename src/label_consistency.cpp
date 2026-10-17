#include "vg_core/label_consistency.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace vg {

void LabelConsistency::add(const DepthFrame& frame, const Eigen::Isometry3d& world_from_camera) {
  if (frame.labels.size() != frame.depth.size()) {
    return;
  }
  const CameraIntrinsics& k = frame.intrinsics;
  const int stride = std::max(1, stride_);
  for (int v = stride / 2; v < k.height; v += stride) {
    for (int u = stride / 2; u < k.width; u += stride) {
      const std::size_t i = frame.index(u, v);
      const float d = frame.depth[i];
      const std::uint8_t label = frame.labels[i];
      if (!std::isfinite(d) || d <= 0.0f || d > max_depth_ || label == 0 ||
          label >= kSemanticClasses || ((ignore_ >> label) & 1u) != 0) {
        continue;
      }
      const Eigen::Vector3d ray((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
      points_.push_back((world_from_camera * (ray * d)).cast<float>());
      labels_.push_back(label);
    }
  }
}

LabelConsistency::Result LabelConsistency::score(const TsdfVolume& volume, int min_votes) const {
  Result result;
  const double voxel = volume.config().voxel_size;
  for (std::size_t i = 0; i < labels_.size(); ++i) {
    const Eigen::Vector3i index =
        (points_[i].cast<double>() / voxel).array().floor().cast<int>();
    // The reading lies on the surface, which may run through the voxel it
    // falls in or the one just behind it along any axis.
    std::optional<SemanticClass> mapped = volume.label_of(index, min_votes);
    for (int axis = 0; axis < 3 && !mapped; ++axis) {
      for (const int side : {-1, 1}) {
        mapped = volume.label_of(index + side * Eigen::Vector3i::Unit(axis), min_votes);
        if (mapped) {
          break;
        }
      }
    }
    if (!mapped) {
      ++result.unmapped;
      continue;
    }
    ++result.readings[labels_[i]];
    result.agree[labels_[i]] += static_cast<std::size_t>(*mapped) == labels_[i] ? 1 : 0;
  }
  return result;
}

std::size_t LabelConsistency::Result::total() const {
  std::size_t n = 0;
  for (const auto r : readings) {
    n += r;
  }
  return n;
}

double LabelConsistency::Result::overall() const {
  std::size_t a = 0;
  for (const auto x : agree) {
    a += x;
  }
  const std::size_t n = total();
  return n == 0 ? 0.0 : static_cast<double>(a) / static_cast<double>(n);
}

double LabelConsistency::Result::of_class(SemanticClass c) const {
  const auto i = static_cast<std::size_t>(c);
  return readings[i] == 0 ? 0.0
                          : static_cast<double>(agree[i]) / static_cast<double>(readings[i]);
}

}  // namespace vg
