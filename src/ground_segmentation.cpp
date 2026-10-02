#include "vg_core/ground_segmentation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_set>
#include <utility>

namespace vg {
namespace {

constexpr int kNoVoxel = std::numeric_limits<int>::max();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

struct VoxelHash {
  std::size_t operator()(const Eigen::Vector3i& v) const {
    // Large primes, as in the usual voxel hashing scheme.
    return static_cast<std::size_t>(v.x()) * 73856093u ^
           static_cast<std::size_t>(v.y()) * 19349669u ^
           static_cast<std::size_t>(v.z()) * 83492791u;
  }
};
using VoxelSet = std::unordered_set<Eigen::Vector3i, VoxelHash>;

constexpr std::array<std::array<int, 2>, 8> kNeighbors8 = {
    {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}}};
constexpr std::array<std::array<int, 2>, 4> kNeighbors4 = {{{0, -1}, {-1, 0}, {1, 0}, {0, 1}}};

// A dense 2D grid of cells over [min, min + size), indexed by global cell
// coordinates.
struct Grid {
  Eigen::Vector2i min = Eigen::Vector2i::Zero();
  int width = 0;
  int height = 0;

  std::size_t cells() const {
    return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  }
  bool contains(int x, int y) const {
    return x >= min.x() && y >= min.y() && x < min.x() + width && y < min.y() + height;
  }
  std::size_t index(int x, int y) const {
    return static_cast<std::size_t>(y - min.y()) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x - min.x());
  }
  Eigen::Vector2i cell(std::size_t i) const {
    const auto w = static_cast<std::size_t>(width);
    return {min.x() + static_cast<int>(i % w), min.y() + static_cast<int>(i / w)};
  }
};

// Largest height change, in whole voxels, allowed between two ground columns
// dist voxels apart.
int max_step_voxels(const GroundConfig& config, double voxel_size, double dist) {
  return static_cast<int>(std::floor(
      (config.max_slope * dist * voxel_size + config.step_tolerance) / voxel_size + 1e-6));
}

// Labels the ground columns of grid (whose bottom voxel index is bottom[i], or
// kNoVoxel). Returns, per column, whether it is ground.
std::vector<char> find_ground_columns(const Grid& grid, const std::vector<int>& bottom,
                                      double voxel_size, const GroundConfig& config) {
  const int step_axis = max_step_voxels(config, voxel_size, 1.0);
  const int step_diag = max_step_voxels(config, voxel_size, std::sqrt(2.0));

  // Regions of column bottoms connected without a step steeper than allowed.
  std::vector<int> region(grid.cells(), -1);
  std::vector<std::vector<std::size_t>> regions;
  for (std::size_t start = 0; start < grid.cells(); ++start) {
    if (bottom[start] == kNoVoxel || region[start] >= 0) {
      continue;
    }
    const int id = static_cast<int>(regions.size());
    regions.emplace_back();
    auto& members = regions.back();
    region[start] = id;
    members.push_back(start);
    for (std::size_t m = 0; m < members.size(); ++m) {
      const std::size_t i = members[m];
      const Eigen::Vector2i c = grid.cell(i);
      for (const auto& d : kNeighbors8) {
        const int nx = c.x() + d[0];
        const int ny = c.y() + d[1];
        if (!grid.contains(nx, ny)) {
          continue;
        }
        const std::size_t n = grid.index(nx, ny);
        const int step = (d[0] != 0 && d[1] != 0) ? step_diag : step_axis;
        if (bottom[n] == kNoVoxel || region[n] >= 0 || std::abs(bottom[n] - bottom[i]) > step) {
          continue;
        }
        region[n] = id;
        members.push_back(n);
      }
    }
  }

  std::vector<std::size_t> order(regions.size());
  for (std::size_t r = 0; r < order.size(); ++r) {
    order[r] = r;
  }
  std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return regions[a].size() > regions[b].size();
  });

  std::vector<char> ground(grid.cells(), 0);
  std::vector<char> accepted(regions.size(), 0);
  auto accept = [&](std::size_t r) {
    accepted[r] = 1;
    for (const std::size_t i : regions[r]) {
      ground[i] = 1;
    }
  };
  if (order.empty()) {
    return ground;
  }
  // The largest region is the ground the others are judged against.
  accept(order.front());

  const double max_gap_voxels = config.max_gap / voxel_size;
  const double cell_area = voxel_size * voxel_size;
  constexpr std::size_t kNoSource = std::numeric_limits<std::size_t>::max();
  for (bool changed = true; changed;) {
    changed = false;

    // Nearest ground column (approximately) to each cell, within max_gap.
    std::vector<std::size_t> nearest(grid.cells(), kNoSource);
    std::deque<std::size_t> queue;
    for (std::size_t i = 0; i < grid.cells(); ++i) {
      if (ground[i]) {
        nearest[i] = i;
        queue.push_back(i);
      }
    }
    while (!queue.empty()) {
      const std::size_t i = queue.front();
      queue.pop_front();
      const Eigen::Vector2i c = grid.cell(i);
      const Eigen::Vector2i source = grid.cell(nearest[i]);
      for (const auto& d : kNeighbors8) {
        const int nx = c.x() + d[0];
        const int ny = c.y() + d[1];
        if (!grid.contains(nx, ny)) {
          continue;
        }
        const std::size_t n = grid.index(nx, ny);
        if (nearest[n] != kNoSource ||
            (Eigen::Vector2i(nx, ny) - source).cast<double>().norm() > max_gap_voxels) {
          continue;
        }
        nearest[n] = nearest[i];
        queue.push_back(n);
      }
    }

    for (const std::size_t r : order) {
      if (accepted[r]) {
        continue;
      }
      // Judge the region by its edge: where it meets or faces the ground, it
      // must not rise or drop more steeply than ground may.
      std::size_t near = 0;
      std::size_t level = 0;
      for (const std::size_t i : regions[r]) {
        const Eigen::Vector2i c = grid.cell(i);
        bool edge = false;
        for (const auto& d : kNeighbors8) {
          const int nx = c.x() + d[0];
          const int ny = c.y() + d[1];
          if (!grid.contains(nx, ny) || region[grid.index(nx, ny)] != static_cast<int>(r)) {
            edge = true;
            break;
          }
        }
        if (!edge || nearest[i] == kNoSource) {
          continue;
        }
        ++near;
        const double dist = (grid.cell(nearest[i]) - c).cast<double>().norm() * voxel_size;
        const double rise = std::abs(bottom[i] - bottom[nearest[i]]) * voxel_size;
        if (rise <= config.max_slope * dist + config.step_tolerance) {
          ++level;
        }
      }
      const bool lines_up = near > 0 && 2 * level >= near;
      const bool isolated_and_large =
          near == 0 &&
          static_cast<double>(regions[r].size()) * cell_area >= config.min_isolated_area;
      if (lines_up || isolated_and_large) {
        accept(r);
        changed = true;
      }
    }
  }
  return ground;
}

// Patches the cells of map marked in fill (all NaN) by harmonic interpolation
// from the observed cells around them, and marks them in filled. Regions that
// touch no observed cell stay NaN.
void fill_holes(HeightMap& map, const std::vector<char>& fill, std::vector<std::uint8_t>& filled) {
  Grid grid;
  grid.width = map.width;
  grid.height = map.height;
  std::vector<char> seen(fill.size(), 0);
  for (std::size_t start = 0; start < fill.size(); ++start) {
    if (!fill[start] || seen[start]) {
      continue;
    }
    std::vector<std::size_t> hole{start};
    seen[start] = 1;
    Eigen::Vector2i lo = grid.cell(start);
    Eigen::Vector2i hi = lo;
    double boundary_sum = 0.0;
    std::size_t boundary_count = 0;
    for (std::size_t m = 0; m < hole.size(); ++m) {
      const Eigen::Vector2i c = grid.cell(hole[m]);
      lo = lo.cwiseMin(c);
      hi = hi.cwiseMax(c);
      for (const auto& d : kNeighbors4) {
        if (!grid.contains(c.x() + d[0], c.y() + d[1])) {
          continue;
        }
        const std::size_t n = grid.index(c.x() + d[0], c.y() + d[1]);
        if (fill[n]) {
          if (!seen[n]) {
            seen[n] = 1;
            hole.push_back(n);
          }
        } else if (!std::isnan(map.heights[n])) {
          boundary_sum += map.heights[n];
          ++boundary_count;
        }
      }
    }
    if (boundary_count == 0) {
      continue;
    }

    // Successive over-relaxation of Laplace's equation: each cell settles to
    // the mean of its neighbors, giving a smooth patch that meets the ground
    // around it. Cells at the map's edge just use the neighbors they have.
    for (const std::size_t i : hole) {
      map.heights[i] = static_cast<float>(boundary_sum / static_cast<double>(boundary_count));
      filled[i] = 1;
    }
    const double pi = std::acos(-1.0);
    const int extent = std::max(hi.x() - lo.x(), hi.y() - lo.y()) + 2;
    const double omega = 2.0 / (1.0 + std::sin(pi / extent));
    for (int iteration = 0; iteration < 10000; ++iteration) {
      double largest_change = 0.0;
      for (const std::size_t i : hole) {
        const Eigen::Vector2i c = grid.cell(i);
        double sum = 0.0;
        int count = 0;
        for (const auto& d : kNeighbors4) {
          if (!grid.contains(c.x() + d[0], c.y() + d[1])) {
            continue;
          }
          const float h = map.heights[grid.index(c.x() + d[0], c.y() + d[1])];
          if (!std::isnan(h)) {
            sum += h;
            ++count;
          }
        }
        const double change = omega * (sum / count - map.heights[i]);
        map.heights[i] = static_cast<float>(map.heights[i] + change);
        largest_change = std::max(largest_change, std::abs(change));
      }
      if (largest_change < 1e-5) {
        break;
      }
    }
  }
}

}  // namespace

GroundSegmentation segment_ground(const std::vector<Eigen::Vector3i>& voxels, double voxel_size,
                                  const GroundConfig& config) {
  GroundSegmentation result;
  result.ground.heights.cell_size = config.cell_size;
  if (voxels.empty()) {
    return result;
  }

  // Columns of voxels sharing (x, y), and the lowest voxel in each.
  Grid columns;
  columns.min = voxels.front().head<2>();
  Eigen::Vector2i max = columns.min;
  for (const auto& v : voxels) {
    columns.min = columns.min.cwiseMin(v.head<2>());
    max = max.cwiseMax(v.head<2>());
  }
  columns.width = max.x() - columns.min.x() + 1;
  columns.height = max.y() - columns.min.y() + 1;
  std::vector<int> bottom(columns.cells(), kNoVoxel);
  for (const auto& v : voxels) {
    int& b = bottom[columns.index(v.x(), v.y())];
    b = std::min(b, v.z());
  }

  const std::vector<char> ground_column = find_ground_columns(columns, bottom, voxel_size, config);

  // In each ground column, the ground is the run of voxels up from the bottom,
  // a few voxels thick where the surface slopes.
  const VoxelSet occupied(voxels.begin(), voxels.end());
  VoxelSet ground;
  const int thickness = std::max(1, max_step_voxels(config, voxel_size, 1.0));
  for (std::size_t i = 0; i < columns.cells(); ++i) {
    if (!ground_column[i]) {
      continue;
    }
    const Eigen::Vector2i c = columns.cell(i);
    for (int k = bottom[i]; k <= bottom[i] + thickness; ++k) {
      const Eigen::Vector3i v(c.x(), c.y(), k);
      if (occupied.count(v) == 0) {
        break;
      }
      ground.insert(v);
      result.ground_voxels.push_back(v);
    }
  }

  // Everything else, split into 26-connected segments.
  VoxelSet unvisited;
  for (const auto& v : voxels) {
    if (ground.count(v) == 0) {
      unvisited.insert(v);
    }
  }
  // Visit in input order so the result doesn't depend on hashing.
  for (const auto& seed : voxels) {
    if (unvisited.erase(seed) == 0) {
      continue;
    }
    MapSegment segment;
    segment.voxels.push_back(seed);
    segment.min = seed;
    segment.max = seed;
    for (std::size_t m = 0; m < segment.voxels.size(); ++m) {
      const Eigen::Vector3i v = segment.voxels[m];
      segment.min = segment.min.cwiseMin(v);
      segment.max = segment.max.cwiseMax(v);
      for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const Eigen::Vector3i n = v + Eigen::Vector3i(dx, dy, dz);
            if (unvisited.erase(n) != 0) {
              segment.voxels.push_back(n);
            } else if (!segment.grounded && ground.count(n) != 0) {
              segment.grounded = true;
            }
          }
        }
      }
    }
    result.segments.push_back(std::move(segment));
  }
  std::stable_sort(
      result.segments.begin(), result.segments.end(),
      [](const MapSegment& a, const MapSegment& b) { return a.voxels.size() > b.voxels.size(); });

  // The ground height map: per cell, the mean ground surface height of the
  // columns whose centers fall in it. The surface lies on the top face of the
  // column's bottom voxel (the ones above it are where a slope or the foot of
  // a wall crosses the column). Cells line up with make_height_map()'s grid.
  const double cs = config.cell_size;
  auto cell_of = [&](int x, int y) -> Eigen::Vector2i {
    return {static_cast<int>(std::floor((x + 0.5) * voxel_size / cs)),
            static_cast<int>(std::floor((y + 0.5) * voxel_size / cs))};
  };
  Grid cells;
  cells.min = cell_of(columns.min.x(), columns.min.y());
  const Eigen::Vector2i cells_max = cell_of(max.x(), max.y());
  cells.width = cells_max.x() - cells.min.x() + 1;
  cells.height = cells_max.y() - cells.min.y() + 1;

  HeightMap& map = result.ground.heights;
  map.origin = cells.min.cast<double>() * cs;
  map.width = cells.width;
  map.height = cells.height;
  std::vector<double> sum(cells.cells(), 0.0);
  std::vector<int> count(cells.cells(), 0);
  for (std::size_t i = 0; i < columns.cells(); ++i) {
    if (!ground_column[i]) {
      continue;
    }
    const Eigen::Vector2i c = columns.cell(i);
    const std::size_t cell = cells.index(cell_of(c.x(), c.y()).x(), cell_of(c.x(), c.y()).y());
    sum[cell] += (bottom[i] + 1) * voxel_size;
    ++count[cell];
  }
  map.heights.assign(cells.cells(), kNaN);
  for (std::size_t i = 0; i < cells.cells(); ++i) {
    if (count[i] > 0) {
      map.heights[i] = static_cast<float>(sum[i] / count[i]);
    }
  }

  // Holes to patch: everything under a segment, plus small unseen holes
  // enclosed by ground or segments.
  std::vector<char> fill(cells.cells(), 0);
  for (const auto& segment : result.segments) {
    for (const auto& v : segment.voxels) {
      const Eigen::Vector2i c = cell_of(v.x(), v.y());
      const std::size_t i = cells.index(c.x(), c.y());
      if (std::isnan(map.heights[i])) {
        fill[i] = 1;
      }
    }
  }
  std::vector<char> seen(cells.cells(), 0);
  const double cell_area = cs * cs;
  for (std::size_t start = 0; start < cells.cells(); ++start) {
    if (!std::isnan(map.heights[start]) || fill[start] || seen[start]) {
      continue;
    }
    std::vector<std::size_t> hole{start};
    seen[start] = 1;
    bool enclosed = true;
    for (std::size_t m = 0; m < hole.size(); ++m) {
      const Eigen::Vector2i c = cells.cell(hole[m]);
      for (const auto& d : kNeighbors4) {
        if (!cells.contains(c.x() + d[0], c.y() + d[1])) {
          enclosed = false;
          continue;
        }
        const std::size_t n = cells.index(c.x() + d[0], c.y() + d[1]);
        if (std::isnan(map.heights[n]) && !fill[n] && !seen[n]) {
          seen[n] = 1;
          hole.push_back(n);
        }
      }
    }
    if (enclosed && static_cast<double>(hole.size()) * cell_area <= config.max_hole_area) {
      for (const std::size_t i : hole) {
        fill[i] = 1;
      }
    }
  }
  result.ground.filled.assign(cells.cells(), 0);
  fill_holes(map, fill, result.ground.filled);

  // How high each segment sits over the (patched) ground.
  for (auto& segment : result.segments) {
    double bottom_above = std::numeric_limits<double>::infinity();
    double top_above = -std::numeric_limits<double>::infinity();
    for (const auto& v : segment.voxels) {
      const Eigen::Vector2i c = cell_of(v.x(), v.y());
      const float g = map.heights[cells.index(c.x(), c.y())];
      if (std::isnan(g)) {
        continue;
      }
      bottom_above = std::min(bottom_above, v.z() * voxel_size - g);
      top_above = std::max(top_above, (v.z() + 1) * voxel_size - g);
    }
    if (std::isinf(bottom_above)) {
      bottom_above = top_above = std::numeric_limits<double>::quiet_NaN();
    }
    segment.bottom_above_ground = bottom_above;
    segment.top_above_ground = top_above;
  }
  return result;
}

GroundSegmentation segment_ground(const TsdfVolume& volume, const GroundConfig& config) {
  return segment_ground(volume.occupied_voxels(), volume.config().voxel_size, config);
}

}  // namespace vg
