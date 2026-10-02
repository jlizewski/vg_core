#include "vg_core/sun_map.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <thread>
#include <utility>

#include "vg_core/solar.hpp"

namespace vg {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Occupied voxels as vertical runs per xy column, for casting rays.
//
// Coordinates are in voxel units: column (x, y) covers [x, x + 1) x [y, y + 1)
// relative to the grid's minimum voxel, and z is the global voxel z. Columns
// are grouped into square tiles holding the height of their tallest run, so a
// ray rising above a tile skips it whole.
class OccluderGrid {
 public:
  static constexpr int kTile = 8;

  OccluderGrid(const std::vector<Eigen::Vector3i>& voxels, double voxel_size)
      : voxel_size_(voxel_size) {
    if (voxels.empty()) {
      return;
    }
    min_ = voxels.front();
    Eigen::Vector3i max = min_;
    for (const auto& v : voxels) {
      min_ = min_.cwiseMin(v);
      max = max.cwiseMax(v);
    }
    nx_ = max.x() - min_.x() + 1;
    ny_ = max.y() - min_.y() + 1;
    top_ = max.z() + 1;
    tiles_x_ = (nx_ + kTile - 1) / kTile;
    tiles_y_ = (ny_ + kTile - 1) / kTile;

    std::vector<Eigen::Vector3i> sorted = voxels;
    std::sort(sorted.begin(), sorted.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
      if (a.y() != b.y()) return a.y() < b.y();
      if (a.x() != b.x()) return a.x() < b.x();
      return a.z() < b.z();
    });
    column_start_.assign(static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) + 1, 0);
    tile_top_.assign(static_cast<std::size_t>(tiles_x_) * static_cast<std::size_t>(tiles_y_),
                     std::numeric_limits<int>::min());
    std::size_t prev_column = 0;
    for (std::size_t i = 0; i < sorted.size(); ++i) {
      const Eigen::Vector3i& v = sorted[i];
      const int cx = v.x() - min_.x();
      const int cy = v.y() - min_.y();
      const std::size_t column = column_index(cx, cy);
      const bool same_column = i > 0 && column == prev_column;
      if (same_column && v.z() < runs_.back().second) {
        continue;  // Duplicate voxel.
      }
      if (same_column && v.z() == runs_.back().second) {
        runs_.back().second = v.z() + 1;
      } else {
        runs_.emplace_back(v.z(), v.z() + 1);
        ++column_start_[column + 1];
      }
      prev_column = column;
      int& tile_top = tile_top_[tile_index(cx / kTile, cy / kTile)];
      tile_top = std::max(tile_top, v.z() + 1);
    }
    for (std::size_t c = 1; c < column_start_.size(); ++c) {
      column_start_[c] += column_start_[c - 1];
    }
  }

  // Whether a ray from point (world meters) along unit direction dir (z > 0)
  // hits an occupied voxel.
  bool blocked(const Eigen::Vector3d& point, const Eigen::Vector3d& dir) const {
    if (runs_.empty()) {
      return false;
    }
    const double ox = point.x() / voxel_size_ - min_.x();
    const double oy = point.y() / voxel_size_ - min_.y();
    const double oz = point.z() / voxel_size_;
    const double dx = dir.x();
    const double dy = dir.y();
    const double dz = dir.z();

    // Clip to the grid's xy box and to below its highest voxel.
    double t0 = 0.0;
    double t1 = (static_cast<double>(top_) - oz) / dz;
    if (!clip(ox, dx, nx_, t0, t1) || !clip(oy, dy, ny_, t0, t1) || t0 >= t1) {
      return false;
    }

    const auto visit_tile = [&](int tx, int ty, double ta, double tb) {
      if (oz + dz * ta >= tile_top_[tile_index(tx, ty)]) {
        return true;  // The ray is above everything in this tile.
      }
      return traverse(ox, oy, dx, dy, 1.0, nx_, ny_, ta, tb,
                      [&](int cx, int cy, double ca, double cb) {
                        const double za = oz + dz * ca;
                        const double zb = oz + dz * cb;
                        const std::size_t c = column_index(cx, cy);
                        for (std::size_t r = column_start_[c]; r < column_start_[c + 1]; ++r) {
                          if (runs_[r].first >= zb) {
                            break;
                          }
                          if (runs_[r].second > za) {
                            return false;
                          }
                        }
                        return true;
                      });
    };
    return !traverse(ox, oy, dx, dy, kTile, tiles_x_, tiles_y_, t0, t1, visit_tile);
  }

 private:
  std::size_t column_index(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(nx_) +
           static_cast<std::size_t>(x);
  }
  std::size_t tile_index(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(tiles_x_) +
           static_cast<std::size_t>(x);
  }

  // Narrows [t0, t1] to where o + d * t lies in [0, n].
  static bool clip(double o, double d, int n, double& t0, double& t1) {
    if (d == 0.0) {
      return o >= 0.0 && o < n;
    }
    double a = -o / d;
    double b = (n - o) / d;
    if (a > b) std::swap(a, b);
    t0 = std::max(t0, a);
    t1 = std::min(t1, b);
    return t0 < t1;
  }

  // Walks the cells (size cell, n_x by n_y) a 2D ray crosses between t_begin
  // and t_end, calling visit(x, y, t_in, t_out) for each until it returns
  // false. Returns false if a visit did.
  template <typename Visit>
  static bool traverse(double ox, double oy, double dx, double dy, double cell, int n_x, int n_y,
                       double t_begin, double t_end, Visit&& visit) {
    // Locate the first cell from a point just inside the range, so a start on
    // a cell boundary picks the cell the ray goes into.
    const double t_probe = t_begin + std::min(1e-9, 0.5 * (t_end - t_begin));
    int x = std::clamp(static_cast<int>(std::floor((ox + dx * t_probe) / cell)), 0, n_x - 1);
    int y = std::clamp(static_cast<int>(std::floor((oy + dy * t_probe) / cell)), 0, n_y - 1);
    const int step_x = dx > 0.0 ? 1 : -1;
    const int step_y = dy > 0.0 ? 1 : -1;
    double next_x = dx != 0.0 ? ((x + (dx > 0.0 ? 1 : 0)) * cell - ox) / dx : kInf;
    double next_y = dy != 0.0 ? ((y + (dy > 0.0 ? 1 : 0)) * cell - oy) / dy : kInf;
    const double delta_x = dx != 0.0 ? cell / std::abs(dx) : kInf;
    const double delta_y = dy != 0.0 ? cell / std::abs(dy) : kInf;
    double t = t_begin;
    while (t < t_end) {
      const double t_next = std::min({next_x, next_y, t_end});
      if (!visit(x, y, t, t_next)) {
        return false;
      }
      if (next_x < next_y) {
        x += step_x;
        t = next_x;
        next_x += delta_x;
      } else {
        y += step_y;
        t = next_y;
        next_y += delta_y;
      }
      if (x < 0 || x >= n_x || y < 0 || y >= n_y) {
        break;
      }
    }
    return true;
  }

  double voxel_size_;
  Eigen::Vector3i min_ = Eigen::Vector3i::Zero();
  int nx_ = 0;
  int ny_ = 0;
  int top_ = 0;
  int tiles_x_ = 0;
  int tiles_y_ = 0;
  // Runs [first, second) of occupied voxel z, per column, bottom up.
  std::vector<std::size_t> column_start_;
  std::vector<std::pair<int, int>> runs_;
  std::vector<int> tile_top_;
};

struct SunStep {
  Eigen::Vector3d direction;  // Toward the sun, in the world frame.
  double irradiance;          // Direct normal, W/m^2.
};

// Upward unit normal of the ground at a cell, from its neighbors' heights.
Eigen::Vector3d ground_normal(const HeightMap& map, int x, int y) {
  const auto slope = [&](int dx, int dy) {
    const bool has_prev = x - dx >= 0 && y - dy >= 0 && map.has_data(x - dx, y - dy);
    const bool has_next = x + dx < map.width && y + dy < map.height && map.has_data(x + dx, y + dy);
    const double h = map.at(x, y);
    if (has_prev && has_next) {
      return (map.at(x + dx, y + dy) - map.at(x - dx, y - dy)) / (2.0 * map.cell_size);
    }
    if (has_next) {
      return (map.at(x + dx, y + dy) - h) / map.cell_size;
    }
    if (has_prev) {
      return (h - map.at(x - dx, y - dy)) / map.cell_size;
    }
    return 0.0;
  };
  return Eigen::Vector3d(-slope(1, 0), -slope(0, 1), 1.0).normalized();
}

}  // namespace

SunMap compute_sun_map(const GroundMap& ground, const std::vector<Eigen::Vector3i>& voxels,
                       double voxel_size, const GeoReference& geo, const SunMapConfig& config) {
  const HeightMap& heights = ground.heights;
  SunMap result;
  result.sun_hours = heights;
  result.irradiation = heights;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (std::size_t i = 0; i < heights.heights.size(); ++i) {
    const bool has_ground = !std::isnan(heights.heights[i]);
    result.sun_hours.heights[i] = has_ground ? 0.0f : nan;
    result.irradiation.heights[i] = has_ground ? 0.0f : nan;
  }
  if (config.time_step <= 0.0 || config.end_time <= config.start_time) {
    return result;
  }

  std::vector<SunStep> steps;
  const auto num_steps =
      static_cast<std::size_t>(std::ceil((config.end_time - config.start_time) / config.time_step));
  const double step_hours = config.time_step / 3600.0;
  for (std::size_t i = 0; i < num_steps; ++i) {
    const double t = config.start_time + (static_cast<double>(i) + 0.5) * config.time_step;
    if (t >= config.end_time) {
      break;
    }
    const SunPosition sun = sun_position(t, geo.latitude, geo.longitude);
    if (sun.elevation_deg <= config.min_elevation_deg || sun.elevation_deg <= 0.0) {
      continue;
    }
    steps.push_back({geo.world_from_enu(sun.direction_enu).normalized(),
                     clear_sky_direct_irradiance(sun.elevation_deg)});
  }
  result.sun_steps = steps.size();
  result.daylight_hours = static_cast<double>(steps.size()) * step_hours;
  if (steps.empty()) {
    return result;
  }

  const OccluderGrid occluders(voxels, voxel_size);
  const double step_kwh = step_hours / 1000.0;

  // Rows are handed out to workers one at a time.
  std::atomic<int> next_row{0};
  const auto work = [&]() {
    for (int y = next_row++; y < heights.height; y = next_row++) {
      for (int x = 0; x < heights.width; ++x) {
        if (!heights.has_data(x, y)) {
          continue;
        }
        const Eigen::Vector3d start(heights.origin.x() + (x + 0.5) * heights.cell_size,
                                    heights.origin.y() + (y + 0.5) * heights.cell_size,
                                    heights.at(x, y) + config.sample_height);
        const Eigen::Vector3d normal = ground_normal(heights, x, y);
        double hours = 0.0;
        double energy = 0.0;
        for (const SunStep& step : steps) {
          const double incidence = normal.dot(step.direction);
          if (incidence <= 0.0 || occluders.blocked(start, step.direction)) {
            continue;
          }
          hours += step_hours;
          energy += step.irradiance * incidence * step_kwh;
        }
        const std::size_t i =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(heights.width) +
            static_cast<std::size_t>(x);
        result.sun_hours.heights[i] = static_cast<float>(hours);
        result.irradiation.heights[i] = static_cast<float>(energy);
      }
    }
  };

  const unsigned hardware = std::max(1u, std::thread::hardware_concurrency());
  const unsigned num_threads =
      config.threads > 0 ? static_cast<unsigned>(config.threads) : hardware;
  std::vector<std::thread> workers;
  for (unsigned i = 1; i < num_threads; ++i) {
    workers.emplace_back(work);
  }
  work();
  for (auto& w : workers) {
    w.join();
  }
  return result;
}

SunMapConfig sun_map_config_for_year(int year, double time_step) {
  SunMapConfig config;
  config.start_time = unix_time_of_year_start(year);
  config.end_time = unix_time_of_year_start(year + 1);
  config.time_step = time_step;
  return config;
}

}  // namespace vg
