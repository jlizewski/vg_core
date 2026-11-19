#pragma once

// A site's layers read back from disk, as Site::make_ground_layer() and
// Site::make_feature_layer() last wrote them (docs/formats.md#site): what an
// app shows over the site's map. The ground layer's grids are read as height
// maps from layers/ground/ (ground_layer_file::k...).

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "vg_core/boundary.hpp"
#include "vg_core/features.hpp"
#include "vg_core/ground_layer.hpp"
#include "vg_core/guidance.hpp"

namespace vg {

struct SiteLayers {
  bool has_ground = false;  // layers/ground/ has its grids.
  // The site's boundary, counter-clockwise; empty with no boundary walk.
  std::vector<BoundaryVertex> boundary;
  // Feature elements, retired ones included.
  std::vector<SiteFeature> features;
  // Ground-fill guidance: targets by rank; empty and done when not made.
  bool has_guidance = false;
  Guidance guidance;
  // The user's edits: the ground layer's (areas), then the boundary's (lines).
  std::vector<GroundEdit> edits;
  // Each area edit against the walks recorded after it.
  std::vector<GroundLayer::EditReview> reviews;
};

// Reads site_dir's layers. Missing files are empty layers; throws
// std::runtime_error on a malformed one.
SiteLayers load_site_layers(const std::filesystem::path& site_dir);

std::optional<BoundaryEdge> parse_boundary_edge(std::string_view name);
std::optional<FeatureKind> parse_feature_kind(std::string_view name);
std::optional<GroundEdit::Kind> parse_edit_kind(std::string_view name);

}  // namespace vg
