#pragma once

// Registering one map into another: finding where a new walk's map sits in an
// earlier one, from the maps alone (no cloud service, no GPS needed).
//
// Both maps are gravity aligned (z up), so the transform has four degrees of
// freedom: a turn about z, and a shift in x, y and z.
//
// 1. Coarse: for each candidate heading, the outlines of the new map's
//    features (where something stands on the terrain: beds, posts, walls,
//    plants; features.asc), all weighted alike,
//    are rasterized and cross-correlated with the reference's by FFT, giving
//    the best shift for that heading. A heading prior (from each map's GPS)
//    narrows the search; without one, all 360 degrees are searched.
// 2. Height: the terrains are lined up where they overlap.
// 3. Fine: trimmed ICP on the occupied voxels standing over the terrain,
//    turning about z and shifting, from each of the best few headings; the
//    one lining up the most points wins, weighed by how well its heading
//    agrees with GPS (which breaks near ties). A fit not clearly better than
//    its best rival is refused as ambiguous.
//
// On three real walks of one yard this found the same alignment with and
// without GPS, with about 0.6 m and 2 degrees of disagreement around a loop of
// three: the walks' own drift, which one rigid transform can't remove.

#include <Eigen/Core>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "vg_core/height_map.hpp"

namespace vg {

// A gravity-aligned rigid transform: p' = Rz(yaw) * p + (x, y, z).
struct MapTransform {
  double yaw = 0.0;  // Radians, counter-clockwise about z.
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  Eigen::Vector3d apply(const Eigen::Vector3d& p) const;
  Eigen::Vector2d apply_xy(const Eigen::Vector2d& p) const;
  // This after other: p -> this(other(p)).
  MapTransform after(const MapTransform& other) const;
  MapTransform inverse() const;
};

// What one map offers for registration.
struct RegistrationInput {
  // Height of what stands on the terrain per cell (features.asc).
  HeightMap features;
  // The terrain (terrain.asc), for lining up heights; may be empty.
  HeightMap terrain;
  // Centers of the occupied voxels, for the fine fit (those over terrain).
  std::vector<Eigen::Vector3d> points;
};

struct RegistrationConfig {
  // Coarse search grid, in meters.
  double raster = 0.3;
  // Features shorter than this (m) are left out of the coarse search: low
  // bumps and noise match anywhere.
  double min_feature_height = 0.25;
  // Use each map's GPS heading and position, when both have them (see
  // Site::add), to narrow the heading search and weigh the fits.
  bool use_gps = true;
  // 1-sigma error of a walk's GPS position relative to another's, in meters.
  // Walks minutes apart share most of their GPS error: three on one day
  // landed 2.7-4.2 m from where GPS put them.
  double gps_position_sigma = 3.0;
  // How far apart the two maps may be, in meters, beyond the reference's
  // features: the coarse search covers that much shift.
  double search_radius = 25.0;
  // Headings tried, in degrees apart.
  double yaw_step_deg = 2.0;
  // With a heading prior, headings within this many of its sigmas (and at
  // least min_yaw_window_deg) are searched; without one, all of them.
  double prior_sigmas = 3.0;
  double min_yaw_window_deg = 10.0;

  // Fine fit: only voxels at least this high over their map's terrain (m)
  // take part, in the fit and in judging it: flat ground lines up at any
  // shift, so it would hold the fit back and pass a wrong one.
  double icp_min_height = 0.15;
  // Pairs of points farther apart than this are ignored, starting
  // at icp_start_distance and shrinking each round to icp_end_distance (m).
  double icp_start_distance = 1.0;
  double icp_end_distance = 0.15;
  int icp_rounds = 30;
  // The fine fit keeps one voxel per cell this size (m): dense voxels add
  // little, and cost time.
  double icp_thin = 0.15;
  // Points of the new map used by the fine fit, at most (evenly picked).
  int max_icp_points = 60000;

  // The best this many headings of the coarse search, at least
  // candidate_separation_deg apart, go on to the fine fit, which picks among
  // them: the coarse peak can be a few degrees off, or a mirror image.
  int coarse_candidates = 12;
  double candidate_separation_deg = 5.0;

  // Acceptance. The fit must line up this many times as many points as the
  // best rival (another heading or place), each weighed by how well its
  // heading agrees with GPS when there's a heading prior...
  double min_rival_ratio = 1.3;
  // ... at least this share of the new map's points must lie within 1 m of
  // the reference (the maps overlap)...
  double min_overlap = 0.2;
  // ... and at least this share of those within 0.3 m (they line up). On
  // real walks right fits scored 0.61-0.82 (each walk drifts a little, and
  // plants change) and a wrong one 0.41.
  double min_inliers = 0.5;
};

struct RegistrationResult {
  // Takes the new map's coordinates into the reference's.
  MapTransform reference_from_map;
  // Coarse search: normalized correlation of the chosen heading's edges.
  double peak = 0.0;
  bool used_prior = false;
  // The best rival fit (another heading or place): how many times better the
  // chosen fit is (points lining up, weighed by agreement with GPS, when
  // there's a prior), and its yaw. 0 when there was no rival.
  double rival_ratio = 0.0;
  double rival_yaw = 0.0;
  // Fine fit, over the voxels standing over the terrain: share of the new
  // map's within 1 m of the reference's, and of those, within 0.1 m and
  // 0.3 m; median distance of those (m).
  double overlap = 0.0;
  double inliers_10cm = 0.0;
  double inliers_30cm = 0.0;
  double median_distance = 0.0;
  // Whether it passed the acceptance checks; why not, if it didn't.
  bool accepted = false;
  std::string problem;
};

// What GPS says about where the map sits in the reference, if anything.
struct RegistrationPrior {
  // The reference_from_map yaw and its 1-sigma, in radians (from both maps'
  // GPS headings): narrows the heading search, and weighs the fits.
  std::optional<double> yaw;
  double yaw_sigma = 0.0;
  // Where the map's origin lands in the reference, and its 1-sigma, in meters
  // (from both maps' GPS positions): weighs the fits, so of two places that
  // fit alike, the one GPS agrees with wins.
  std::optional<Eigen::Vector2d> origin;
  double origin_sigma = 0.0;
};

// Registers map into reference.
RegistrationResult register_map(const RegistrationInput& reference, const RegistrationInput& map,
                                const RegistrationConfig& config = {},
                                const RegistrationPrior& prior = {});

}  // namespace vg
