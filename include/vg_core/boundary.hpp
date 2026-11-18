#pragma once

// A site's boundary (docs/design/guided-mapping.md): the edge of the site,
// from a boundary walk, a walk along the edge that comes back to its start.
//
//   1. The loop: where the walk comes back closest to its start, after having
//      gone well away from it. How far apart they are, and how far the
//      walk's overlap with its own start sits from it, is the tracker's drift
//      around the loop.
//   2. The path round the loop, resampled, counter-clockwise.
//   3. Each point pushed outward, across the ground layer, to the edge the
//      walker followed: the first covered cell (a fence, a wall, a hedge), or
//      the end of the seen ground. With neither within max_push, it's put
//      assumed_offset out and marked assumed, for the user to confirm.
//      A running median smooths spikes (a gate, a gap in a hedge).
//
// The site's boundary is its first boundary walk's; other boundary walks are
// compared with it. Cells of the ground layer outside it are Outside.
//
// On disk, in <site>/layers/boundary/ (Site::make_ground_layer()):
//
//   boundary.csv  the site's boundary: one vertex per line, x, y (site frame,
//                 m), how it was found (structure, ground_end, assumed)
//   walks.csv     each boundary walk: its loop, drift, and agreement with the
//                 site's boundary

#include <Eigen/Core>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "vg_core/ground_layer.hpp"

namespace vg {

namespace boundary_layer_file {
inline constexpr const char* kDir = "layers/boundary";
inline constexpr const char* kBoundary = "boundary.csv";
inline constexpr const char* kWalks = "walks.csv";
}  // namespace boundary_layer_file

struct BoundaryConfig {
  // The walk closes its loop coming back within close_distance of its start
  // (m), having come at least min_away nearer it than it had been.
  double min_away = 5.0;
  double close_distance = 5.0;
  // The start of the walk compared with its overlap at the end, for drift (m).
  double overlap_length = 3.0;
  // The path is resampled every sample_spacing (m).
  double sample_spacing = 0.5;
  // How far out to look for the edge (m), and where to put it with none.
  double max_push = 3.0;
  double assumed_offset = 1.0;
  // Running median of the pushes over this many vertices (odd).
  int smooth_window = 5;
  // Each point's outward direction is taken across this much of the path on
  // each side (m), so it turns gradually round a corner.
  double normal_span = 1.0;
};

// How a boundary vertex was found.
enum class BoundaryEdge : std::uint8_t {
  Structure,  // A covered cell: a fence, a wall, a hedge.
  GroundEnd,  // Where the seen ground ends.
  Assumed,    // Neither within max_push: assumed_offset out.
};

std::string_view boundary_edge_name(BoundaryEdge edge);

struct BoundaryVertex {
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  // Site frame, m.
  BoundaryEdge edge = BoundaryEdge::Assumed;
  double push = 0.0;  // How far out from the path (m).
};

// Where a walk comes back to its start.
struct LoopClosure {
  bool closed = false;
  std::size_t end = 0;  // Index of the path point closest to the start.
  double length = 0.0;  // Of the loop, m.
  double gap = 0.0;     // From that point to the start, m.
  // The walk's overlap with its start, after the loop: how far it sits from
  // the start of the walk, on average (m), and that per 100 m of loop.
  double overlap_offset = 0.0;
  double drift_per_100m = 0.0;
};

LoopClosure close_loop(const std::vector<Eigen::Vector2d>& path, const BoundaryConfig& config = {});

struct Boundary {
  LoopClosure loop;
  std::vector<BoundaryVertex> vertices;  // Counter-clockwise.
  double area = 0.0;                     // m^2.
  double perimeter = 0.0;                // m.
};

// The boundary from a boundary walk's path (site frame), pushed out across
// the ground layer. Empty (no vertices) if the walk doesn't close its loop.
Boundary estimate_boundary(const std::vector<Eigen::Vector2d>& path, const GroundLayer& ground,
                           const BoundaryConfig& config = {});

// How two boundaries agree: the distance from each one's outline to the
// other's, both ways (m), and their overlap (intersection over union of the
// areas, from a raster at cell m).
struct BoundaryAgreement {
  double median_distance = 0.0;
  double p90_distance = 0.0;
  double overlap = 0.0;
};
BoundaryAgreement compare_boundaries(const Boundary& a, const Boundary& b, double cell = 0.1);

// Cells of the ground layer outside the boundary become Outside (no height).
void apply_boundary(GroundLayer& ground, const Boundary& boundary);

// A map package's camera path (trajectory.csv), x and y in its map frame.
std::vector<Eigen::Vector2d> load_path(const std::filesystem::path& package);

}  // namespace vg
