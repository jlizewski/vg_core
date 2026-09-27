#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "vg_core/sensor_data.hpp"

namespace vg {

struct TsdfConfig {
  // Edge length of one voxel, in meters.
  double voxel_size = 0.02;
  // Distances beyond this from the surface are clamped. A few voxels is typical.
  double truncation_distance = 0.08;
  // Depth readings outside this range are ignored.
  double min_depth = 0.1;
  double max_depth = 5.0;
  // Pixels with confidence below this are ignored (when confidence is present).
  std::uint8_t min_confidence = 0;
  // Cap on the per-voxel weight, so the map can still adapt to change.
  float max_weight = 64.0f;
};

// Truncated signed distance field over a sparse voxel grid (voxel hashing).
// Space is split into blocks of kBlockSize^3 voxels, allocated only near
// observed surfaces. Distances are positive in front of a surface and negative
// behind it.
class TsdfVolume {
 public:
  static constexpr int kBlockSize = 8;

  explicit TsdfVolume(const TsdfConfig& config = {});

  const TsdfConfig& config() const { return config_; }

  // Fuses one depth frame taken by a camera at world_from_camera.
  void integrate(const DepthFrame& frame, const Eigen::Isometry3d& world_from_camera);

  // Signed distance in meters at the voxel containing point, or nullopt if that
  // voxel has not been observed.
  std::optional<double> distance_at(const Eigen::Vector3d& point) const;

  // Points on the zero crossing of the field, interpolated between voxel
  // centers. Roughly one point per surface voxel.
  std::vector<Eigen::Vector3d> extract_surface_points() const;

  std::size_t num_blocks() const { return blocks_.size(); }

 private:
  struct Voxel {
    float tsdf = 0.0f;  // Normalized to [-1, 1] by the truncation distance.
    float weight = 0.0f;
  };
  using Block = std::array<Voxel, kBlockSize * kBlockSize * kBlockSize>;

  struct IndexHash {
    std::size_t operator()(const Eigen::Vector3i& i) const;
  };

  Eigen::Vector3i voxel_index(const Eigen::Vector3d& point) const;
  Eigen::Vector3d voxel_center(const Eigen::Vector3i& voxel) const;
  const Voxel* find_voxel(const Eigen::Vector3i& voxel) const;
  void integrate_block(const Eigen::Vector3i& block_index, Block& block, const DepthFrame& frame,
                       const Eigen::Isometry3d& camera_from_world);

  TsdfConfig config_;
  std::unordered_map<Eigen::Vector3i, Block, IndexHash> blocks_;
};

}  // namespace vg
