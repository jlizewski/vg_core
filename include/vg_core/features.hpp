#pragma once

// A site's feature elements (docs/design/guided-mapping.md): the things on
// the ground a person recognizes and relocalization can match (the post by
// the shed, the fence along the back, a raised bed), as elements rather
// than maps.
//
// What each walk saw standing through waist height (the ground stage's
// elements.csv: GroundConfig::element_band_low/high) becomes elements, by the
// shape of its cross-section there (an oriented box):
//
//   post   small both ways: a trunk, a post, a pole
//   line   long and thin: a fence, a wall
//   block  the rest with some area: a raised bed, a bin, a shed
//
// Placed in the site frame, each walk's elements are matched to the site's,
// oldest walk first: a match refines the element's position (the mean of
// its sightings); an element a walk covered (saw that spot) but didn't find
// is missed, and retire_after_misses misses in a row retire it (it's kept,
// marked retired). Everything is re-derived from the walks on every build,
// so the same walks give the same elements.
//
// On disk, in <site>/layers/features/ (Site::make_feature_layer()):
//
//   elements.csv      every element: kind, class, footprint box, height,
//                     sightings and misses, first and last seen, status,
//                     spread (how far its sightings lie from its position)
//   observations.csv  every sighting: walk, element, where it was seen

#include <Eigen/Core>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "vg_core/ground_layer.hpp"
#include "vg_core/registration.hpp"
#include "vg_core/semantics.hpp"

namespace vg {

namespace feature_layer_file {
inline constexpr const char* kDir = "layers/features";
inline constexpr const char* kElements = "elements.csv";
inline constexpr const char* kObservations = "observations.csv";
}  // namespace feature_layer_file

enum class FeatureKind : std::uint8_t { Post, Line, Block };

std::string_view feature_kind_name(FeatureKind kind);

// An element's footprint and what it is.
struct Feature {
  FeatureKind kind = FeatureKind::Post;
  SemanticClass label = SemanticClass::Unknown;
  Eigen::Vector2d center = Eigen::Vector2d::Zero();  // Meters.
  double length = 0.0;  // Along axis, meters.
  double width = 0.0;   // Across it.
  double axis = 0.0;    // Radians from x, in [0, pi).
  double height = 0.0;  // Top above the ground, meters.
};

struct FeatureConfig {
  // Objects lower than this (m) over the ground aren't elements.
  double min_height = 0.3;
  // A post fits in a square this size (m).
  double post_max_size = 0.6;
  // A line is at least this long (m), and no wider than this share of it.
  double line_min_length = 1.0;
  double line_max_aspect = 0.3;
  // A block's box covers at least this much ground (m^2).
  double block_min_area = 0.25;
  // Posts and blocks no longer than this (m) can be the same element: a post
  // seen whole in one walk can look like a small block in another.
  double small_block_size = 1.0;
  // Two sightings are the same element if they're this close (m): centers
  // for posts and blocks; for lines, across the line, and within this many
  // degrees of each other.
  double match_distance = 0.5;
  double match_angle_deg = 20.0;
  // Missed by this many walks in a row that covered it: retired.
  int retire_after_misses = 2;
};

// A walk's elements, in its map frame: its objects that stand on the ground
// (not overhangs or stretches labeled ground), by footprint.
std::vector<Feature> walk_features(const std::vector<WalkObject>& objects,
                                   const FeatureConfig& config = {});

// One walk's elements placed in the site frame, and whether the walk covered
// a site-frame point (saw that spot, so would have seen an element there).
struct WalkFeatures {
  std::string id;
  std::string recorded;
  std::vector<Feature> features;  // Site frame.
  std::function<bool(const Eigen::Vector2d&)> covers;
};

struct SiteFeature {
  std::string id;  // e.g. post-3, by kind in order of first sighting.
  Feature feature;  // Position and size: the mean of its sightings.
  int seen = 0;     // Walks that saw it.
  int missed = 0;   // Walks in a row since it was last seen that covered it but didn't.
  std::string first_seen;
  std::string last_seen;
  bool retired = false;
  // Farthest sighting from the position (m): across the line, for lines.
  double spread = 0.0;
};

struct FeatureSighting {
  std::string walk;
  std::string element;
  Feature feature;  // As seen by that walk, in the site frame.
};

struct FeatureLayer {
  std::vector<SiteFeature> elements;
  std::vector<FeatureSighting> sightings;
};

// Matches the walks' elements (walks oldest first) into the site's.
FeatureLayer match_features(const std::vector<WalkFeatures>& walks,
                            const FeatureConfig& config = {});

// f moved from a walk's map frame into the site frame.
Feature to_site(const Feature& f, const MapTransform& site_from_map);

}  // namespace vg
