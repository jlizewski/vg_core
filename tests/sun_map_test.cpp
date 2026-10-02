#include "vg_core/sun_map.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <set>
#include <tuple>

#include "vg_core/solar.hpp"

namespace {

constexpr double kVoxel = 0.02;
constexpr double kPi = 3.14159265358979323846;
using Voxels = std::vector<Eigen::Vector3i>;

// Flat ground at z = 0 over [x0, x1) x [y0, y1) meters, in cells of cell_size,
// and the voxel layer just below it.
struct FlatGround {
  vg::GroundMap map;
  Voxels voxels;
};

FlatGround flat_ground(double x0, double y0, double x1, double y1, double cell_size) {
  FlatGround g;
  auto& h = g.map.heights;
  h.origin = {x0, y0};
  h.cell_size = cell_size;
  h.width = static_cast<int>(std::lround((x1 - x0) / cell_size));
  h.height = static_cast<int>(std::lround((y1 - y0) / cell_size));
  h.heights.assign(static_cast<std::size_t>(h.width * h.height), 0.0f);
  g.map.filled.assign(h.heights.size(), 0);
  for (int y = static_cast<int>(std::lround(y0 / kVoxel)); y < std::lround(y1 / kVoxel); ++y) {
    for (int x = static_cast<int>(std::lround(x0 / kVoxel)); x < std::lround(x1 / kVoxel); ++x) {
      g.voxels.emplace_back(x, y, -1);
    }
  }
  return g;
}

void add_box(Voxels& voxels, const Eigen::Vector3d& min, const Eigen::Vector3d& max) {
  const Eigen::Vector3i lo = (min / kVoxel).array().round().cast<int>();
  const Eigen::Vector3i hi = (max / kVoxel).array().round().cast<int>();
  for (int z = lo.z(); z < hi.z(); ++z) {
    for (int y = lo.y(); y < hi.y(); ++y) {
      for (int x = lo.x(); x < hi.x(); ++x) {
        voxels.emplace_back(x, y, z);
      }
    }
  }
}

vg::GeoReference location(double latitude, double longitude, double heading = 0.0) {
  vg::GeoReference geo;
  geo.latitude = latitude;
  geo.longitude = longitude;
  geo.heading = heading;
  return geo;
}

double mean_hours(const vg::SunMap& sun, double x0, double y0, double x1, double y1) {
  const auto& m = sun.sun_hours;
  double sum = 0.0;
  int n = 0;
  for (int y = 0; y < m.height; ++y) {
    for (int x = 0; x < m.width; ++x) {
      const double cx = m.origin.x() + (x + 0.5) * m.cell_size;
      const double cy = m.origin.y() + (y + 0.5) * m.cell_size;
      if (cx >= x0 && cx < x1 && cy >= y0 && cy < y1 && m.has_data(x, y)) {
        sum += m.at(x, y);
        ++n;
      }
    }
  }
  return n > 0 ? sum / n : std::nan("");
}

TEST(SunMap, OpenGroundGetsAllDaylight) {
  const FlatGround g = flat_ground(0.0, 0.0, 1.0, 1.0, 0.1);
  auto config = vg::sun_map_config_for_year(2026);
  const auto sun = vg::compute_sun_map(g.map, g.voxels, kVoxel, location(45.0, -122.0), config);
  // About half the hours of the year, a little more from refraction.
  EXPECT_EQ(sun.sun_steps, static_cast<std::size_t>(std::lround(sun.daylight_hours)));
  EXPECT_GT(sun.daylight_hours, 4300.0);
  EXPECT_LT(sun.daylight_hours, 4600.0);
  for (const float h : sun.sun_hours.heights) {
    EXPECT_FLOAT_EQ(h, static_cast<float>(sun.daylight_hours));
  }
  // Clear-sky direct sun on flat ground at 45 N: roughly 1500-2200 kWh/m^2.
  EXPECT_GT(sun.irradiation.at(3, 3), 1300.0f);
  EXPECT_LT(sun.irradiation.at(3, 3), 2400.0f);
}

TEST(SunMap, CellsWithoutGroundStayEmpty) {
  FlatGround g = flat_ground(0.0, 0.0, 0.4, 0.4, 0.1);
  g.map.heights.heights[5] = std::nanf("");
  const auto sun = vg::compute_sun_map(g.map, g.voxels, kVoxel, location(10.0, 0.0),
                                       vg::sun_map_config_for_year(2026, 6 * 3600.0));
  EXPECT_TRUE(std::isnan(sun.sun_hours.heights[5]));
  EXPECT_TRUE(std::isnan(sun.irradiation.heights[5]));
  EXPECT_GT(sun.sun_hours.heights[4], 0.0f);
}

TEST(SunMap, WallShadesTheSideAwayFromTheSun) {
  // An east-west wall 2 m tall along y = 0, with ground 2 m either side.
  FlatGround g = flat_ground(-2.0, -2.0, 2.0, 2.0, 0.1);
  add_box(g.voxels, {-2.0, -0.1, 0.0}, {2.0, 0.1, 2.0});
  const auto config = vg::sun_map_config_for_year(2026, 4 * 3600.0);

  // Northern hemisphere, world +x east: the sun is to the south, so the north
  // side (+y) is shaded.
  const auto north = vg::compute_sun_map(g.map, g.voxels, kVoxel, location(45.0, 0.0), config);
  const double south_side = mean_hours(north, -1.0, -1.0, 1.0, -0.3);
  const double north_side = mean_hours(north, -1.0, 0.3, 1.0, 1.0);
  EXPECT_GT(south_side, 0.8 * north.daylight_hours);
  EXPECT_LT(north_side, 0.5 * south_side);
  // Cells under the wall never see the sun.
  EXPECT_EQ(mean_hours(north, -1.0, -0.05, 1.0, 0.05), 0.0);

  // Turning the world frame around (world +x is west) swaps the sides.
  const auto turned =
      vg::compute_sun_map(g.map, g.voxels, kVoxel, location(45.0, 0.0, kPi), config);
  EXPECT_NEAR(mean_hours(turned, -1.0, 0.3, 1.0, 1.0), south_side, 1e-6);

  // In the southern hemisphere the sun is to the north.
  const auto south = vg::compute_sun_map(g.map, g.voxels, kVoxel, location(-45.0, 0.0), config);
  EXPECT_LT(mean_hours(south, -1.0, -1.0, 1.0, -0.3), 0.5 * mean_hours(south, -1.0, 0.3, 1.0, 1.0));
}

TEST(SunMap, SlopeFacingTheSunGetsMoreEnergy) {
  FlatGround g = flat_ground(0.0, 0.0, 1.0, 1.0, 0.1);
  auto& h = g.map.heights;
  // Rises 0.5 m per meter toward the north, so it faces south.
  for (int y = 0; y < h.height; ++y) {
    for (int x = 0; x < h.width; ++x) {
      h.heights[static_cast<std::size_t>(y * h.width + x)] =
          static_cast<float>(0.5 * (y + 0.5) * 0.1);
    }
  }
  FlatGround flat = flat_ground(0.0, 0.0, 1.0, 1.0, 0.1);
  const auto config = vg::sun_map_config_for_year(2026, 3 * 3600.0);
  // No occluders, so only the angle matters.
  const auto sloped = vg::compute_sun_map(g.map, {}, kVoxel, location(50.0, 0.0), config);
  const auto level = vg::compute_sun_map(flat.map, {}, kVoxel, location(50.0, 0.0), config);
  EXPECT_GT(sloped.irradiation.at(5, 5), 1.2f * level.irradiation.at(5, 5));
}

TEST(SunMap, RaysMatchBruteForceMarching) {
  // Random blobs over a patch of ground, checked against marching each ray in
  // small steps through a voxel set, one sun position at a time.
  FlatGround g = flat_ground(0.0, 0.0, 2.0, 2.0, 0.1);
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int i = 0; i < 25; ++i) {
    const Eigen::Vector3d c(-0.5 + 3.0 * u(rng), -0.5 + 3.0 * u(rng), 0.6 * u(rng));
    const Eigen::Vector3d size(0.05 + 0.3 * u(rng), 0.05 + 0.3 * u(rng), 0.05 + 0.4 * u(rng));
    add_box(g.voxels, c, c + size);
  }
  std::set<std::tuple<int, int, int>> occupied;
  for (const auto& v : g.voxels) {
    occupied.emplace(v.x(), v.y(), v.z());
  }
  Eigen::Vector3i lo = g.voxels.front();
  Eigen::Vector3i hi = lo;
  for (const auto& v : g.voxels) {
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
  }

  const auto geo = location(40.0, -75.0, 0.3);
  int mismatches = 0;
  int total = 0;
  const double day = vg::unix_time_of_year_start(2026) + 100 * 86400.0;
  for (int hour = 11; hour < 23; hour += 2) {
    vg::SunMapConfig config;
    config.start_time = day + hour * 3600.0;
    config.end_time = config.start_time + 60.0;
    config.time_step = 60.0;
    const auto sun = vg::compute_sun_map(g.map, g.voxels, kVoxel, geo, config);
    if (sun.sun_steps == 0) {
      continue;
    }
    const auto position = vg::sun_position(config.start_time + 30.0, geo.latitude, geo.longitude);
    const Eigen::Vector3d dir = geo.world_from_enu(position.direction_enu).normalized();
    const auto& m = g.map.heights;
    for (int y = 0; y < m.height; ++y) {
      for (int x = 0; x < m.width; ++x) {
        Eigen::Vector3d p(m.origin.x() + (x + 0.5) * m.cell_size,
                          m.origin.y() + (y + 0.5) * m.cell_size, m.at(x, y) + 0.04);
        bool blocked = false;
        for (double t = 0.0; t < 20.0 && !blocked; t += 0.002) {
          const Eigen::Vector3d q = p + t * dir;
          const Eigen::Vector3i v = (q / kVoxel).array().floor().cast<int>();
          if ((v.array() < lo.array()).any() || (v.array() > hi.array()).any()) {
            if (v.z() > hi.z()) break;
            continue;
          }
          blocked = occupied.count({v.x(), v.y(), v.z()}) > 0;
        }
        const bool lit = sun.sun_hours.at(x, y) > 0.0f;
        // Marching in steps can miss a voxel's corner, but never finds a hit
        // that isn't there.
        EXPECT_FALSE(lit && blocked) << "cell " << x << ", " << y << " hour " << hour;
        mismatches += (lit == blocked) ? 1 : 0;
        ++total;
      }
    }
  }
  ASSERT_GT(total, 1000);
  EXPECT_LE(mismatches, total / 200);
}

}  // namespace
