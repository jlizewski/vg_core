#pragma once

// Separating the ground from everything standing on or hanging over it.
//
// The 3D voxel map (TsdfVolume::occupied_voxels) is split into ground voxels
// and segments: connected groups of the remaining voxels (plants, raised beds,
// fences, furniture, branches, eaves, ...). Segments are left unlabeled for a
// later classification step, with the geometry it will need. The ground voxels
// become a 2.5D ground height map whose holes, where segments stood or hid the
// ground, are patched by interpolating the surrounding ground.

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "vg_core/height_map.hpp"
#include "vg_core/tsdf_volume.hpp"

namespace vg {

struct GroundConfig {
  // Steepest ground, as rise over run (1.0 = 45 degrees). Surfaces steeper
  // than this, such as walls and the sides of beds, are not ground.
  double max_slope = 1.0;
  // Extra height step allowed between neighboring ground columns on top of the
  // slope, in meters. Bumps smaller than about this stay part of the ground.
  double step_tolerance = 0.03;
  // Patches of ground separated from the main ground by unseen space (e.g.
  // behind an obstacle) are joined to it if they line up with ground within
  // this distance, in meters.
  double max_gap = 0.5;
  // A patch with no ground within max_gap is still taken as ground if it is at
  // least this large, in square meters (e.g. a second scanned area).
  double min_isolated_area = 0.5;
  // Cell size of the ground height map, in meters.
  double cell_size = 0.05;
  // Unseen holes in the ground map are patched if they are enclosed by ground
  // and no larger than this, in square meters. Holes under segments are
  // always patched, whatever their size.
  double max_hole_area = 1.0;
};

// A connected group of non-ground voxels: something on or over the ground.
struct MapSegment {
  // Voxel indices, as from TsdfVolume::occupied_voxels().
  std::vector<Eigen::Vector3i> voxels;
  // Inclusive voxel index bounds.
  Eigen::Vector3i min = Eigen::Vector3i::Zero();
  Eigen::Vector3i max = Eigen::Vector3i::Zero();
  // Whether the segment touches the ground (an object or structure standing
  // on it). False for overhangs seen without their support, such as a canopy
  // or an eave, and for floating debris.
  bool grounded = false;
  // Height of the segment's lowest and highest points above the ground
  // beneath them, in meters (NaN where there is no ground map below).
  double bottom_above_ground = 0.0;
  double top_above_ground = 0.0;
};

struct GroundMap {
  // Ground surface height per cell, holes patched. NaN where there is no
  // ground and nothing to patch from (e.g. beyond the scanned area).
  HeightMap heights;
  // Same layout as heights: 1 where the height was interpolated rather than
  // observed.
  std::vector<std::uint8_t> filled;

  bool is_filled(int x, int y) const {
    return filled[static_cast<std::size_t>(y) * static_cast<std::size_t>(heights.width) +
                  static_cast<std::size_t>(x)] != 0;
  }
};

struct GroundSegmentation {
  std::vector<Eigen::Vector3i> ground_voxels;
  // Largest first.
  std::vector<MapSegment> segments;
  GroundMap ground;
};

// Splits occupied voxels (edge voxel_size meters) into ground and segments.
//
// The ground is grown from the largest region of column bottoms (the lowest
// voxel under each xy) that rises no faster than max_slope, then extended to
// other such regions that line up with it across unseen gaps. Tops of raised
// beds and other surfaces with no visible ground below stay out because their
// edges drop steeply to the ground. Cost is linear in the number of voxels
// plus the xy area they cover; it reprocesses the whole map on each call.
GroundSegmentation segment_ground(const std::vector<Eigen::Vector3i>& voxels, double voxel_size,
                                  const GroundConfig& config = {});

// Segments the occupied voxels of a volume.
GroundSegmentation segment_ground(const TsdfVolume& volume, const GroundConfig& config = {});

}  // namespace vg
