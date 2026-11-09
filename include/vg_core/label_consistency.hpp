#pragma once

// How well a recording's semantic labels agree with themselves, without any
// ground truth: each labeled depth reading is compared with the class the
// map's votes settled on for the place it hit. A model whose labels hold
// steady from frame to frame and from view to view agrees with the map; one
// that flips (e.g. leaf litter as ground, then as an animal) doesn't. It
// scores models and class mappings against each other on the same walk.

#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "vg_core/semantics.hpp"
#include "vg_core/sensor_data.hpp"
#include "vg_core/tsdf_volume.hpp"

namespace vg {

class LabelConsistency {
 public:
  // Every pixel_stride-th labeled pixel in each direction is sampled, up to
  // max_depth meters away. Readings of the classes in ignore are left out:
  // those the map drops (TsdfConfig::drop_labels) can never agree with it.
  explicit LabelConsistency(int pixel_stride = 4, double max_depth = 3.5,
                            SemanticMask ignore = 0)
      : stride_(pixel_stride), max_depth_(max_depth), ignore_(ignore) {}

  // Samples a labeled depth frame taken from world_from_camera (does nothing
  // for an unlabeled frame).
  void add(const DepthFrame& frame, const Eigen::Isometry3d& world_from_camera);

  struct Result {
    // Readings per class, and how many of them agree with the map's class
    // where they land.
    std::array<std::size_t, kSemanticClasses> readings{};
    std::array<std::size_t, kSemanticClasses> agree{};
    // Readings where the map has no class (no voxel, or too few votes).
    std::size_t unmapped = 0;

    std::size_t total() const;
    // Share of readings (with a mapped class) that agree, overall and per class.
    double overall() const;
    double of_class(SemanticClass c) const;
  };

  // Compares the samples with the volume's votes (TsdfVolume::label_of with
  // min_votes). Unknown readings are left out.
  Result score(const TsdfVolume& volume, int min_votes) const;

  std::size_t samples() const { return labels_.size(); }

 private:
  int stride_;
  double max_depth_;
  SemanticMask ignore_;
  std::vector<Eigen::Vector3f> points_;  // World points.
  std::vector<std::uint8_t> labels_;
};

}  // namespace vg
