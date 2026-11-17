#pragma once

// A site's ground layer (docs/design/guided-mapping.md): the ground as an
// entity of its own, combined from every walk of the site rather than taken
// from one, with what's known of each cell and where each value came from,
// and the user's edits on top.
//
// Like every layer it has evidence (here, each walk's ground stage output,
// placed in the site frame), edits, and an estimator that turns them into
// the clean layer. The estimator is a pure function of its inputs, so the
// same evidence and edits always give the same layer.
//
// On disk, in <site>/layers/ground/ (written by Site::make_ground_layer()):
//
//   evidence.csv      the walks used and how each was placed in the site
//   edits.csv         the user's edits, oldest first (read, never rewritten)
//   ground.asc        ground height per cell (m); no data outside or unknown
//   state.asc         GroundState per cell
//   provenance.asc    Provenance per cell
//   observations.asc  how many walks measured the cell
//   source.asc        which evidence row (1-based) the height came from
//   layer.json        how it was made: estimator, settings, inputs, counts

#include <Eigen/Core>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "vg_core/height_map.hpp"
#include "vg_core/registration.hpp"

namespace vg {

namespace ground_layer_file {
inline constexpr const char* kDir = "layers/ground";
inline constexpr const char* kEvidence = "evidence.csv";
inline constexpr const char* kEdits = "edits.csv";
inline constexpr const char* kHeights = "ground.asc";
inline constexpr const char* kState = "state.asc";
inline constexpr const char* kProvenance = "provenance.asc";
inline constexpr const char* kObservations = "observations.asc";
inline constexpr const char* kSource = "source.asc";
inline constexpr const char* kInfo = "layer.json";
}  // namespace ground_layer_file

// What the site knows of the ground in one cell.
enum class GroundState : std::uint8_t {
  Outside = 0,   // No walk saw it, and nothing encloses it.
  Measured = 1,  // Seen as ground.
  Filled = 2,    // A small unseen hole, filled smoothly from around it.
  Covered = 3,   // Under something standing on the ground; the surface runs on beneath.
  Unknown = 4,   // An unseen hole too large to fill: a place to go and look.
};

// Where a cell's value came from.
enum class Provenance : std::uint8_t {
  None = 0,       // No value.
  Measured = 1,   // From walks that saw it.
  Inferred = 2,   // Filled or carried over from around it.
  Edited = 3,     // Set by the user.
  Confirmed = 4,  // Shown to the user and accepted.
};

// One walk's ground, in its own map frame (the ground stage's outputs), and
// where that map sits in the site.
struct WalkGround {
  std::string id;  // Names it in evidence.csv, e.g. the package's name.
  std::string recorded;
  MapTransform site_from_map;
  HeightMap heights;                // ground.asc: patched where not measured.
  std::vector<std::uint8_t> state;  // ground_state.asc: GroundCell, 0 where none.
  std::vector<float> weight;        // ground_weight.asc: 0 where not measured.
};

// Loads a package's ground stage outputs. Throws std::runtime_error if they're
// missing (re-run its ground stage: vg map <package> ground).
WalkGround load_walk_ground(const std::filesystem::path& package);

// A user's correction of the ground.
struct GroundEdit {
  enum class Kind : std::uint8_t {
    Ground,     // This area is ground: measured stays, the rest is filled.
    NotGround,  // Something stands here: covered, the ground runs on beneath.
  };
  std::string id;
  std::string time;  // When it was made, ISO 8601.
  Kind kind = Kind::Ground;
  std::vector<Eigen::Vector2d> polygon;  // Site frame, meters; at least 3 vertices.
  std::string note;
};

// edits.csv: `id,time,kind,polygon,note`, polygon as `x y;x y;...`, kind
// `ground` or `not_ground`. A missing file is no edits. Throws
// std::runtime_error on a malformed line.
std::vector<GroundEdit> load_ground_edits(const std::filesystem::path& path);
void save_ground_edits(const std::vector<GroundEdit>& edits, const std::filesystem::path& path);

struct GroundLayerConfig {
  // How walks that measured a cell are combined.
  enum class Combine : std::uint8_t {
    Latest,  // The latest walk that measured it (as the site's current maps).
  };
  Combine combine = Combine::Latest;
};

struct GroundLayer {
  // NaN where Outside or Unknown, or Filled or Covered with no ground around
  // to fill from.
  HeightMap heights;
  // The rest have the same layout as heights.
  std::vector<GroundState> state;
  std::vector<Provenance> provenance;
  std::vector<std::uint16_t> observations;  // Walks that measured the cell.
  std::vector<int> source;                  // Index of the walk the height came from; -1.

  std::size_t index(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(heights.width) +
           static_cast<std::size_t>(x);
  }
};

// The ground layer from the walks (oldest first) and edits (applied in
// order), on a grid in the site frame covering every walk, at the first
// walk's cell size. Cells under a NotGround edit are Covered and under a
// Ground edit Measured or Filled; cells left Filled or Covered with no
// height are filled smoothly from the ground around them.
GroundLayer estimate_ground(const std::vector<WalkGround>& walks,
                            const std::vector<GroundEdit>& edits,
                            const GroundLayerConfig& config = {});

// Whether p lies inside the polygon (even-odd rule).
bool inside_polygon(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& p);

}  // namespace vg
