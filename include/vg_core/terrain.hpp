#pragma once

// The terrain: a smooth, coarse contour of the ground, fitted to two kinds of
// evidence.
//
// - Seen ground (ground.asc cells that weren't patched): precise where it's
//   really the ground, but in overgrown places what's taken for ground can be
//   the top of low foliage or dead growth.
// - The walked path: the phone is carried at a fairly steady height, so the
//   camera's height minus that height is the ground underfoot. Each point is
//   rough (about +-18 cm on a measured walk), but many points together pin
//   down the ground level where none is seen, and show up "ground" that sits
//   too high.
//
// The terrain is a grid of nodes (node_spacing apart) with heights read
// between them bilinearly, solved as one sparse weighted least-squares problem
// with a bending penalty, so it follows the general lie of the land rather
// than every bump. The phone's height is solved for too, unless given. The
// fit starts from the path alone, then brings in the seen ground with robust
// weights: ground well above the fit (foliage) counts for little where the
// path disagrees with it.
//
// What stands on the terrain (beds, mounds, plants, ...) is the surface minus
// the terrain: see MapPackage::make_terrain().

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "vg_core/height_map.hpp"

namespace vg {

struct TerrainConfig {
  // Distance between the terrain's nodes, in meters: the finest detail it has.
  double node_spacing = 0.5;
  // How much the terrain may bend: the height a node may sit off the line
  // through its two neighbors, in meters (the sigma of a second difference).
  // Smaller is smoother.
  double bend = 0.05;
  // Spread of the seen ground about the terrain, in meters, for a node's
  // area of it: the cells there count as one observation together, since
  // their errors (foliage, a missed bump) are shared.
  double ground_sigma = 0.05;
  // Robust weights: a ground cell this far above the fit counts half as much,
  // and far less the further above it is (foliage taken for ground)...
  double above_scale = 0.1;
  // ... and below it (dips and ditches are more likely real).
  double below_scale = 0.3;
  // Rounds of reweighting.
  int iterations = 8;

  // The walked path as evidence.
  bool use_path = true;
  // The phone's height above the ground while walking, in meters; unset to
  // solve for it (from where the path crosses seen ground).
  std::optional<double> phone_height;
  // The phone's height when it can't be solved for (no seen ground under the
  // path) and isn't given.
  double default_phone_height = 1.5;
  // Spread of one path point's height above the ground, in meters.
  double path_sigma = 0.18;
  // Robust weights: a path point this far off the fit counts half as much
  // (e.g. the phone raised to look at something).
  double path_scale = 0.3;
  // Path points are taken this far apart along the walk, in meters, so
  // standing still doesn't count as many points.
  double path_spacing = 0.25;

  // The terrain is mapped only within this distance of evidence (seen ground
  // or the path), in meters.
  double reach = 2.0;
};

// The fitted terrain's nodes; height anywhere by bilinear interpolation.
struct TerrainModel {
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();  // Node (0, 0).
  double spacing = 0.0;
  int width = 0;
  int height = 0;
  std::vector<double> nodes;  // Row-major (row = y).

  // Height at p, clamped to the grid's edge outside it.
  double at(const Eigen::Vector2d& p) const;
};

struct Terrain {
  // The terrain on the ground map's cell grid (extended to cover the path),
  // NaN beyond reach of any evidence.
  HeightMap heights;
  TerrainModel model;
  // The phone's height over the terrain, and whether it was solved for.
  double phone_height = 0.0;
  bool phone_height_solved = false;
  // Evidence used.
  std::size_t ground_cells = 0;
  std::size_t path_points = 0;
  // Seen ground cells left mostly out of the fit (robust weight under 0.5),
  // and how many of them were above it.
  std::size_t ground_cells_discounted = 0;
  std::size_t ground_cells_discounted_above = 0;
  // Median distance of the evidence from the fitted terrain, in meters (path
  // points less the phone height).
  double ground_residual = 0.0;
  double path_residual = 0.0;
};

// Fits the terrain to ground (patched: 1 where its cell was interpolated, not
// seen; same layout, or empty if all were seen) and path (camera positions
// along the walk, in time order). Either may be empty, not both; with no
// evidence at all the result is empty.
Terrain fit_terrain(const HeightMap& ground, const std::vector<std::uint8_t>& patched,
                    const std::vector<Eigen::Vector3d>& path, const TerrainConfig& config = {});

}  // namespace vg
