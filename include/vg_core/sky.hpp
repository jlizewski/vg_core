#pragma once

// The canopy and other things overhead that depth can't reach, from where the
// sky is seen and where it isn't.
//
// Phone depth stops at a few meters, but a tree canopy or a roofline 6-15 m up
// still blocks the sun. Semantic labels see them: every pixel labeled sky is a
// ray open all the way up, and every pixel looking up that isn't sky (and has
// no depth reading near by) is blocked somewhere along its ray, beyond depth's
// reach. Over many frames from many places, coarse cells that sky rays pass
// through are open, and cells that blocked rays keep passing through but sky
// rays never do are taken as occupied: a visual hull of the canopy.
//
// Evidence is gathered while a recording is replayed (build_map_package
// writes sky.vgs to the package), so it can be combined across walks in a
// site's frame and read out again with other settings. On seven walks of one
// yard, the carved cells from six walks predicted which of the seventh's
// upward pixels were sky better than a horizon profile (sky elevation per
// direction), most where it matters: under the canopy and near the house.

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <unordered_map>
#include <vector>

#include "vg_core/height_map.hpp"
#include "vg_core/registration.hpp"
#include "vg_core/sensor_data.hpp"

namespace vg {

struct SkyConfig {
  // Gather the evidence (needs semantic labels).
  bool enabled = true;
  // Edge of the coarse cells, in meters.
  double cell_size = 0.5;
  // Rays are followed this far, in meters.
  double range = 40.0;
  // A pixel that isn't sky and has no depth reading this near is blocked
  // somewhere past this distance, in meters (about depth's reach).
  double blocked_beyond = 3.5;
  // Every Nth pixel in each direction casts a ray.
  int pixel_stride = 3;
  // Blocked rays count only looking at least this far up, in degrees: lower
  // ones mostly hit the ground or things near it, which the depth maps.
  double min_elevation_deg = 3.0;
};

// Per coarse cell: how many frames saw sky through it, and how many saw it
// blocked by something beyond depth's reach along a ray through it.
class SkyEvidence {
 public:
  struct Counts {
    std::uint32_t clear = 0;
    std::uint32_t blocked = 0;
  };

  explicit SkyEvidence(double cell_size = 0.5);

  double cell_size() const { return cell_size_; }
  std::size_t frames() const { return frames_; }
  std::size_t size() const { return cells_.size(); }
  bool empty() const { return cells_.empty(); }

  // Adds a frame's rays: its depth and semantic labels (frames without labels
  // add nothing), seen from world_from_camera (camera optical frame).
  void add(const DepthFrame& frame, const Eigen::Isometry3d& world_from_camera,
           const SkyConfig& config);

  // Adds another walk's evidence, moved by t into this one's frame (each of
  // its cells into the cell its center lands in).
  void add(const SkyEvidence& other, const MapTransform& t);

  // Every cell with evidence, as (cell index, counts).
  std::vector<std::pair<Eigen::Vector3i, Counts>> cells() const;

  // For reading a saved one back.
  void set(const Eigen::Vector3i& cell, const Counts& counts);
  void set_frames(std::size_t frames) { frames_ = frames; }

 private:
  double cell_size_;
  std::size_t frames_ = 0;
  std::unordered_map<std::int64_t, Counts> cells_;
};

// Files. Throw std::runtime_error if they can't be written or read.
void save_sky(const SkyEvidence& sky, const std::filesystem::path& path);
SkyEvidence load_sky(const std::filesystem::path& path);

struct CanopyConfig {
  // A cell is occupied if at least this many frames saw it blocked...
  int min_blocked = 20;
  // ... and at most this many saw sky through it (labels err sometimes).
  int max_clear = 1;
  // Occupied cells lower than this over the terrain (m) are left out: the
  // depth maps what's that low.
  double min_height = 2.0;
};

struct Canopy {
  // The occupied cells (indices at the evidence's cell size).
  std::vector<Eigen::Vector3i> cells;
  double cell_size = 0.0;
  // Per column of cells: the height of the lowest and the highest occupied
  // cell over the terrain, in meters (NaN where none). Seen from below, the
  // bottom is what's measured; above it no sky ray passes, so the top is only
  // an upper bound.
  HeightMap bottom;
  HeightMap top;
  // Cells with enough evidence to judge (open or occupied).
  std::size_t observed = 0;
};

// The canopy from sky evidence, over terrain (in the same frame).
Canopy make_canopy(const SkyEvidence& sky, const HeightMap& terrain,
                   const CanopyConfig& config = {});

}  // namespace vg
