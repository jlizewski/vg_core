#pragma once

// The build stage of a map package (see vg_core/map_package.hpp): replaying a
// recorded session into the 3D map, which every other stage is derived from.

#include <filesystem>
#include <limits>
#include <optional>
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
  // Undo the tracker's pose jumps (see pose_jumps.hpp) before mapping, so
  // the parts of the walk on either side of one don't map as ghost copies.
  bool remove_pose_jumps = true;
  // Fused weight a voxel needs to be mapped (TsdfConfig::min_weight). If
  // unset, it's chosen to suit the depth: confidence lowers the weight of
  // most readings, so depth with confidence (e.g. ARCore raw depth) gets
  // min_weight_with_confidence, and depth without it (smoothed) the other.
  std::optional<float> min_weight;
  float min_weight_with_confidence = 0.05f;
  float min_weight_without_confidence = 0.5f;
  // Only depth taken in this window is fused, in seconds from the session's
  // first sensor reading; e.g. to build the parts of a walk on either side
  // of a tracking jump and compare them. Poses, GPS etc. are always read.
  double from = 0.0;
  double to = std::numeric_limits<double>::infinity();
};

// Rebuilds the 3D map from the session and starts a map package in dir with
// it (map.vgm, voxels.ply, manifest.json), replacing any package there. The
// map is placed on the Earth from the session's GPS fixes and poses, when it
// has GPS. Progress and timings go to log. Throws std::runtime_error if the
// session can't be read or the package can't be written.
void build_map_package(const std::filesystem::path& session, const std::filesystem::path& dir,
                       const BuildOptions& options, std::ostream& log);

}  // namespace vg::io
