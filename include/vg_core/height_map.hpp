#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <vector>

namespace vg {

// 2.5D grid of the highest surface point per cell, in the world xy plane.
// Cell (x, y) covers [origin + (x, y) * cell_size, origin + (x + 1, y + 1) * cell_size).
struct HeightMap {
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();
  double cell_size = 0.0;
  int width = 0;
  int height = 0;
  // Row-major (row = y). NaN where no surface was seen.
  std::vector<float> heights;

  float at(int x, int y) const {
    return heights[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                   static_cast<std::size_t>(x)];
  }
  bool has_data(int x, int y) const { return !std::isnan(at(x, y)); }
};

// Builds a height map covering all points, keeping the highest z per cell.
HeightMap make_height_map(const std::vector<Eigen::Vector3d>& points, double cell_size);

}  // namespace vg
