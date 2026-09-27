#include "vg_core/height_map.hpp"

#include <limits>

namespace vg {

HeightMap make_height_map(const std::vector<Eigen::Vector3d>& points, double cell_size) {
  HeightMap map;
  map.cell_size = cell_size;
  if (points.empty()) {
    return map;
  }

  Eigen::Vector2d min = points.front().head<2>();
  Eigen::Vector2d max = min;
  for (const Eigen::Vector3d& p : points) {
    min = min.cwiseMin(p.head<2>());
    max = max.cwiseMax(p.head<2>());
  }

  // Snap the origin to the cell grid so maps built at different times line up.
  map.origin = (min / cell_size).array().floor() * cell_size;
  map.width = static_cast<int>(std::floor((max.x() - map.origin.x()) / cell_size)) + 1;
  map.height = static_cast<int>(std::floor((max.y() - map.origin.y()) / cell_size)) + 1;
  map.heights.assign(static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height),
                     std::numeric_limits<float>::quiet_NaN());

  for (const Eigen::Vector3d& p : points) {
    const Eigen::Vector2d cell = ((p.head<2>() - map.origin) / cell_size).array().floor();
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
