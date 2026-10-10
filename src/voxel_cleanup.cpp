#include "vg_core/voxel_cleanup.hpp"

#include <unordered_map>

namespace vg {
namespace {

struct VoxelHash {
  std::size_t operator()(const Eigen::Vector3i& v) const {
    return static_cast<std::size_t>(v.x()) * 73856093u ^
           static_cast<std::size_t>(v.y()) * 19349669u ^
           static_cast<std::size_t>(v.z()) * 83492791u;
  }
};

}  // namespace

CleanupResult remove_small_components(const std::vector<Eigen::Vector3i>& voxels,
                                      std::size_t min_voxels) {
  CleanupResult result;
  if (min_voxels <= 1) {
    result.voxels = voxels;
    return result;
  }
  // Component of each voxel, labeled by flood fill in input order.
  std::unordered_map<Eigen::Vector3i, int, VoxelHash> label;
  label.reserve(voxels.size());
  for (const auto& v : voxels) {
    label.emplace(v, -1);
  }
  std::vector<std::size_t> sizes;
  std::vector<Eigen::Vector3i> stack;
  for (const auto& seed : voxels) {
    if (label[seed] >= 0) {
      continue;
    }
    const int id = static_cast<int>(sizes.size());
    std::size_t size = 0;
    label[seed] = id;
    stack.push_back(seed);
    while (!stack.empty()) {
      const Eigen::Vector3i v = stack.back();
      stack.pop_back();
      ++size;
      for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const auto it = label.find(v + Eigen::Vector3i(dx, dy, dz));
            if (it != label.end() && it->second < 0) {
              it->second = id;
              stack.push_back(it->first);
            }
          }
        }
      }
    }
    sizes.push_back(size);
  }

  result.voxels.reserve(voxels.size());
  for (const auto& v : voxels) {
    if (sizes[static_cast<std::size_t>(label[v])] >= min_voxels) {
      result.voxels.push_back(v);
    } else {
      ++result.removed_voxels;
    }
  }
  for (const std::size_t size : sizes) {
    result.removed_components += size < min_voxels ? 1 : 0;
  }
  return result;
}

}  // namespace vg
