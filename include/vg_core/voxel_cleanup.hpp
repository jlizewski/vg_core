#pragma once

// Removing specks from a voxel map: small groups of voxels connected to
// nothing else. The ground and everything standing on it form large
// connected pieces; a few voxels floating on their own are left by noisy
// depth (or are too small to matter).

#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace vg {

struct CleanupResult {
  std::vector<Eigen::Vector3i> voxels;  // Kept, in input order.
  std::size_t removed_components = 0;
  std::size_t removed_voxels = 0;
};

// Keeps the voxels in 26-connected components of at least min_voxels.
CleanupResult remove_small_components(const std::vector<Eigen::Vector3i>& voxels,
                                      std::size_t min_voxels);

}  // namespace vg
