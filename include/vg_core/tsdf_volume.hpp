#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <unordered_set>
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
  // Only every pixel_stride-th pixel in each direction places new blocks. The
  // blocks it finds are still fused from every pixel, so at coarse voxels a
  // stride of 2 or more loses nothing and saves most of the allocation work.
  int pixel_stride = 1;
  // Threads that fuse a frame's blocks in parallel; 0 uses all hardware
  // threads. Results are the same for any count.
  int threads = 1;
};

// Truncated signed distance field over a sparse voxel grid (voxel hashing).
// Space is split into blocks of kBlockSize^3 voxels, allocated only near
// observed surfaces. Distances are positive in front of a surface and negative
// behind it.
class TsdfVolume {
 public:
  static constexpr int kBlockSize = 8;

  struct Voxel {
    float tsdf = 0.0f;  // Normalized to [-1, 1] by the truncation distance.
    float weight = 0.0f;
  };
  // Voxel (x, y, z) of a block is at [(z * kBlockSize + y) * kBlockSize + x].
  using Block = std::array<Voxel, kBlockSize * kBlockSize * kBlockSize>;

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

  // The surface points belonging to one block (those between a voxel of the
  // block and its +x, +y or +z neighbor). Empty if the block doesn't exist.
  std::vector<Eigen::Vector3d> extract_surface_points(const Eigen::Vector3i& block_index) const;

  // Occupied voxels: the shell of voxels just behind each observed surface
  // (signed distance below zero with an observed in-front neighbor). Together
  // they form a 3D voxel map of everything seen. Indices are global; voxel
  // (i, j, k) spans [i, j, k] * voxel_size to [i + 1, j + 1, k + 1] * voxel_size.
  std::vector<Eigen::Vector3i> occupied_voxels() const;

  // The occupied voxels inside one block. Empty if the block doesn't exist.
  std::vector<Eigen::Vector3i> occupied_voxels(const Eigen::Vector3i& block_index) const;

  // Blocks whose surface points or occupied voxels may have changed since the
  // last call, for incremental consumers such as a live view. Includes the six
  // face neighbors of updated blocks, since results near a block's edge read
  // voxels across it.
  std::vector<Eigen::Vector3i> take_changed_blocks();

  // Edge length of a block, in meters. Block (i, j, k) spans
  // [i, j, k] * block_edge() to [i + 1, j + 1, k + 1] * block_edge().
  double block_edge() const { return config_.voxel_size * kBlockSize; }

  std::size_t num_blocks() const { return blocks_.size(); }

  // Indices of all allocated blocks, sorted (by z, then y, then x).
  std::vector<Eigen::Vector3i> block_indices() const;

  // Raw block access, e.g. for saving and loading a volume. find_block() returns
  // nullptr if the block doesn't exist; insert_block() creates it (empty) if
  // needed.
  const Block* find_block(const Eigen::Vector3i& index) const;
  Block& insert_block(const Eigen::Vector3i& index);

 private:

  struct IndexHash {
    std::size_t operator()(const Eigen::Vector3i& i) const;
  };

  Eigen::Vector3i voxel_index(const Eigen::Vector3d& point) const;
  Eigen::Vector3d voxel_center(const Eigen::Vector3i& voxel) const;
  const Voxel* find_voxel(const Eigen::Vector3i& voxel) const;
  void integrate_block(const Eigen::Vector3i& block_index, Block& block, const DepthFrame& frame,
                       const Eigen::Isometry3d& camera_from_world);
  void append_surface_points(const Eigen::Vector3i& block_index, const Block& block,
                             std::vector<Eigen::Vector3d>& points) const;
  void append_occupied_voxels(const Eigen::Vector3i& block_index, const Block& block,
                              std::vector<Eigen::Vector3i>& voxels) const;

  TsdfConfig config_;
  std::unordered_map<Eigen::Vector3i, Block, IndexHash> blocks_;
  std::unordered_set<Eigen::Vector3i, IndexHash> changed_;
};

}  // namespace vg
