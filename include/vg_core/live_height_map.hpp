#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

#include "vg_core/height_map.hpp"
#include "vg_core/tsdf_volume.hpp"

namespace vg {

// One height map cell. Cell (x, y) covers [x, x + 1) * cell_size by
// [y, y + 1) * cell_size in world coordinates.
struct HeightCell {
  int x = 0;
  int y = 0;
  float height = 0.0f;  // NaN if the cell no longer has a surface.
};

// A height map kept up to date from a TsdfVolume one block at a time, so a
// live view only redraws what changed. Its cells line up with the grid that
// make_height_map() uses for the same cell size.
class LiveHeightMap {
 public:
  explicit LiveHeightMap(double cell_size);

  double cell_size() const { return cell_size_; }

  // Re-reads the surface points of changed_blocks from volume and recomputes
  // the cells they cover. Returns the cells whose height changed.
  std::vector<HeightCell> update(const TsdfVolume& volume,
                                 const std::vector<Eigen::Vector3i>& changed_blocks);

  // Number of cells with a surface.
  std::size_t size() const { return cells_.size(); }

  // Dense copy of the current map, e.g. for export.
  HeightMap snapshot() const;

 private:
  struct Hash {
    std::size_t operator()(const Eigen::Vector2i& i) const;
  };
  using Column = std::unordered_map<int, std::vector<Eigen::Vector3d>>;  // By block z.

  Eigen::Vector2i cell_of(const Eigen::Vector3d& point) const;

  double cell_size_;
  double block_edge_ = 0.0;
  double voxel_size_ = 0.0;
  std::unordered_map<Eigen::Vector2i, Column, Hash> columns_;  // By block (x, y).
  std::unordered_map<Eigen::Vector2i, float, Hash> cells_;
};

}  // namespace vg
