#include "vg_core/height_map.hpp"

#include <limits>

namespace vg {

HeightMap make_height_map(const std::vector<Eigen::Vector3d>& points, double cell_size) {
  HeightMap map;
  map.cell_size = cell_size;
  if (points.empty()) {
    return map;
  }

  // Cells are indexed on a global grid (cell i covers [i, i + 1) * cell_size),
  // so maps built at different times, or live, line up exactly.
  auto cell_of = [cell_size](const Eigen::Vector3d& p) -> Eigen::Vector2i {
    return (p.head<2>() / cell_size).array().floor().cast<int>();
  };
  Eigen::Vector2i min = cell_of(points.front());
  Eigen::Vector2i max = min;
  for (const Eigen::Vector3d& p : points) {
    const Eigen::Vector2i cell = cell_of(p);
    min = min.cwiseMin(cell);
    max = max.cwiseMax(cell);
  }

  map.origin = min.cast<double>() * cell_size;
  map.width = max.x() - min.x() + 1;
  map.height = max.y() - min.y() + 1;
  map.heights.assign(static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height),
                     std::numeric_limits<float>::quiet_NaN());

  for (const Eigen::Vector3d& p : points) {
    const Eigen::Vector2i cell = cell_of(p) - min;
    const auto x = static_cast<std::size_t>(cell.x());
    const auto y = static_cast<std::size_t>(cell.y());
    float& h = map.heights[y * static_cast<std::size_t>(map.width) + x];
    const auto z = static_cast<float>(p.z());
    if (std::isnan(h) || z > h) {
      h = z;
    }
  }
  return map;
}

}  // namespace vg
