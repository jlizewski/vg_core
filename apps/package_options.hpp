#pragma once

// Command-line options for the derived stages of a map package, shared by
// vg_replay (which runs them all after building the map) and vg_map (which
// re-runs chosen ones).

#include <chrono>
#include <cstdlib>
#include <string>

#include "vg_core/map_package.hpp"
#include "vg_core/solar.hpp"

namespace vg::app {

inline constexpr const char* kStageOptionsHelp =
    "ground stage:\n"
    "  --max-slope R          steepest ground, rise over run (default 1.0 = 45 deg)\n"
    "  --step-tolerance M     extra step allowed between ground columns (default 0.03)\n"
    "  --max-gap M            join ground patches across unseen gaps up to this (default 0.5)\n"
    "  --min-isolated-area M2 keep separate ground patches at least this big (default 0.5)\n"
    "  --max-hole-area M2     patch unseen holes in the ground up to this big (default 1.0)\n"
    "sun stage:\n"
    "  --year Y               year to simulate (default the current one)\n"
    "  --sun-step MIN         minutes between sun positions (default 60)\n"
    "  --lat DEG --lon DEG    the map's location, when the session has no GPS\n"
    "  --x-bearing DEG        compass bearing of the map's +x axis, clockwise from north\n"
    "  --sample-height M      height above the ground of the sun rays' start (default 1 voxel)\n";

struct StageOptions {
  SurfaceOptions surface;
  GroundOptions ground;
  SunOptions sun;

  StageOptions() {
    const auto now = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    sun.year = utc_year(static_cast<double>(now));
  }

  // Sets the 2D cell size of every derived map.
  void set_cell_size(double cell_size) {
    surface.cell_size = cell_size;
    ground.cell_size = cell_size;
  }

  // Takes a stage option and its value; false if arg isn't one.
  bool parse(const std::string& arg, const char* text) {
    const double value = std::atof(text);
    GroundConfig& g = ground.config;
    if (arg == "--max-slope") {
      g.max_slope = value;
    } else if (arg == "--step-tolerance") {
      g.step_tolerance = value;
    } else if (arg == "--max-gap") {
      g.max_gap = value;
    } else if (arg == "--min-isolated-area") {
      g.min_isolated_area = value;
    } else if (arg == "--max-hole-area") {
      g.max_hole_area = value;
    } else if (arg == "--year") {
      sun.year = static_cast<int>(value);
    } else if (arg == "--sun-step") {
      sun.step_minutes = value;
    } else if (arg == "--lat") {
      sun.latitude = value;
    } else if (arg == "--lon") {
      sun.longitude = value;
    } else if (arg == "--x-bearing") {
      sun.x_bearing_deg = value;
    } else if (arg == "--sample-height") {
      sun.sample_height = value;
    } else {
      return false;
    }
    return true;
  }
};

}  // namespace vg::app
