#pragma once

// The build stage of a map package (see vg_core/map_package.hpp): replaying a
// recorded session into the 3D map, which every other stage is derived from.

#include <filesystem>
#include <ostream>

#include "vg_core/map_builder.hpp"

namespace vg::io {

struct BuildOptions {
  MapBuilderConfig builder;
  // Cell size of the 2D maps derived from the package, in meters.
  double cell_size = 0.1524;
  // Playback speed: 0 as fast as possible, 1 real time printing the map's
  // progress as it grows.
  double rate = 0.0;
};

// Rebuilds the 3D map from the session and starts a map package in dir with
// it (map.vgm, voxels.ply, manifest.json), replacing any package there. The
// map is placed on the Earth from the session's GPS fixes and poses, when it
// has GPS. Progress and timings go to log. Throws std::runtime_error if the
// session can't be read or the package can't be written.
void build_map_package(const std::filesystem::path& session, const std::filesystem::path& dir,
                       const BuildOptions& options, std::ostream& log);

}  // namespace vg::io
