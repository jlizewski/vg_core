#pragma once

// Sun map: how much direct sunlight each part of the ground gets over a period,
// such as a whole year.
//
// For each time step the sun's position is computed for where the map is on
// the Earth, and a ray is cast from every ground cell toward the sun through
// the 3D voxel map. Cells whose ray escapes the map without hitting anything
// are in sun for that step. Space beyond the mapped area is assumed open, and
// the sky is assumed clear (no clouds or weather).

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "vg_core/geo_reference.hpp"
#include "vg_core/ground_segmentation.hpp"
#include "vg_core/height_map.hpp"

namespace vg {

struct SunMapConfig {
  // Simulated period, in UTC seconds since the Unix epoch: [start_time,
  // end_time). Each step samples the sun at its midpoint.
  double start_time = 0.0;
  double end_time = 0.0;
  // Length of a time step, in seconds.
  double time_step = 3600.0;
  // Height above the ground the rays start from, in meters. Keep it at least
  // a voxel or two so a ray doesn't hit the ground's own bumps.
  double sample_height = 0.04;
  // The sun counts only when at least this high, in degrees.
  double min_elevation_deg = 0.0;
  // Worker threads; 0 uses all hardware threads.
  int threads = 0;
};

struct SunMap {
  // Hours of direct sun per cell over the period. Same layout as the ground
  // height map; NaN where it has no ground.
  HeightMap sun_hours;
  // Clear-sky direct solar energy reaching the ground per cell over the
  // period, in kWh/m^2, allowing for the ground's slope and the sun's angle.
  HeightMap irradiation;
  // Hours the sun was up (above min_elevation_deg): the most sun a cell with
  // nothing around it could get. sun_hours / daylight_hours is the fraction of
  // possible sun.
  double daylight_hours = 0.0;
  // Number of time steps with the sun up, i.e. rays cast per cell.
  std::size_t sun_steps = 0;
};

// Computes the sun map over the ground map's cells. voxels (edge voxel_size
// meters) are the occluders, normally all occupied voxels of the map including
// the ground, so slopes and objects both cast shadows.
SunMap compute_sun_map(const GroundMap& ground, const std::vector<Eigen::Vector3i>& voxels,
                       double voxel_size, const GeoReference& geo, const SunMapConfig& config);

// The simulated period covering one calendar year (UTC), hourly by default.
SunMapConfig sun_map_config_for_year(int year, double time_step = 3600.0);

}  // namespace vg
