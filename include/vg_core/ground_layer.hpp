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
//   source.asc        which evidence row (1-based) counts most toward the height
//   spread.asc        how far the farthest walk is from the height (m)
//   layer.json        how it was made: estimator, settings, inputs, counts

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include "vg_core/height_map.hpp"
#include "vg_core/registration.hpp"
#include "vg_core/semantics.hpp"

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
inline constexpr const char* kSpread = "spread.asc";
inline constexpr const char* kInfo = "layer.json";
}  // namespace ground_layer_file

// What the site knows of the ground in one cell.
enum class GroundState : std::uint8_t {
  Outside = 0,   // No walk saw it, and nothing encloses it.
  Measured = 1,  // Seen as ground.
  Filled = 2,    // Unseen but surely ground: a small hole, or under low plants.
  Covered = 3,   // Under a structure or something tall standing on it.
  Unknown = 4,   // An unseen hole too large to fill: a place to go and look.
  // Filled and Covered ground runs on smoothly from the measured ground
  // around it.
};

// Where a cell's value came from.
enum class Provenance : std::uint8_t {
  None = 0,       // No value.
  Measured = 1,   // From walks that saw it.
  Inferred = 2,   // Filled or carried over from around it.
  Edited = 3,     // Set by the user.
  Confirmed = 4,  // Shown to the user and accepted.
};

// Something a walk saw on or over the ground (a row of objects.csv).
struct WalkObject {
  SemanticClass label = SemanticClass::Unknown;
  bool grounded = false;          // Stands on the ground (false: an overhang).
  double top_above_ground = 0.0;  // Meters; NaN if unknown.
  double footprint_area = 0.0;    // Square meters.
  // Its footprint as an oriented box (MapSegment), in the walk's map frame;
  // NaN center if the table predates it.
  Eigen::Vector2d center = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
  double length = 0.0;
  double width = 0.0;
  double axis = 0.0;  // Radians.
};

// One walk's ground, in its own map frame (the ground stage's outputs), and
// where that map sits in the site.
struct WalkGround {
  std::string id;  // Names it in evidence.csv, e.g. the package's name.
  std::string recorded;
  // The walk whose frame is the site's (the first added, D9): its ground is
  // the height every other walk is aligned to. Without one, the oldest.
  bool sets_frame = false;
  MapTransform site_from_map;
  HeightMap heights;                // ground.asc: patched where not measured.
  std::vector<std::uint8_t> state;  // ground_state.asc: GroundCell, 0 where none.
  std::vector<float> weight;        // ground_weight.asc: 0 where not measured.
  // ground_object.asc: the index in objects of the largest object over the
  // cell, -1 for none. Empty if not known (every patched cell under an
  // object then counts as covered).
  std::vector<int> object;
  std::vector<WalkObject> objects;  // objects.csv, in id order.
  // elements.csv: what stood through the element band (all grounded; their
  // area as footprint_area). Empty if the package predates it.
  std::vector<WalkObject> elements;
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
  // How the walks that measured a cell (or patched it the same way) are
  // combined. Robustly, by default: the weighted median of their heights
  // (weighted by how much each saw the cell, WalkGround::weight), then the
  // weighted mean of the walks within agree_within of it. One walk that's off
  // (a bad reading, a misplaced walk) can't move the result, and walks that
  // agree average out their noise. With two walks it's the better-seen one,
  // or their mean where they agree.
  //
  // latest_wins instead takes the latest walk that measured the cell, as the
  // site's current maps do; kept for comparison.
  bool latest_wins = false;
  double agree_within = 0.10;  // Meters.

  // Before combining, each walk's ground (but the one that sets the site
  // frame, WalkGround::sets_frame) is moved up or down and tilted to meet the
  // walks before it, where
  // both measured the ground: phone tracking drifts in height along a walk,
  // so walks placed by a turn and a shift (registration) can still sit
  // centimeters apart, or tilted by a degree or two. The fit leaves out
  // cells far off it (something changed there). With fewer than
  // align_min_cells shared cells it's an offset only; a tilt steeper than
  // align_max_tilt_deg is refused (offset only).
  bool align = true;
  int align_min_cells = 200;
  double align_max_tilt_deg = 5.0;

  // Where no walk measured the ground: under something standing on it that's
  // a structure, a tree, or at least cover_min_height tall, it's Covered;
  // under anything lower (plants), Filled. Overhangs and objects labeled
  // ground (a stretch of ground not joined to the rest) don't count. Other
  // unseen cells are a hole: Filled up to max_fill_area, else Unknown; but
  // cells no walk knew anything of, touching the map's edge, are Outside.
  double cover_min_height = 0.5;  // Meters.
  double max_fill_area = 1.0;     // Square meters.
};

// How a walk's ground was moved to meet the walks before it: its height at p
// is lowered by correction_at(p).
struct WalkAlignment {
  double offset = 0.0;  // Meters, at center.
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  Eigen::Vector2d slope = Eigen::Vector2d::Zero();  // Rise per meter in x and y.
  bool tilted = false;
  // The cells it shared with the walks before it, and the median distance
  // between them there before and after the correction (m; NaN for none).
  std::size_t cells = 0;
  double before = std::numeric_limits<double>::quiet_NaN();
  double after = std::numeric_limits<double>::quiet_NaN();

  double correction_at(const Eigen::Vector2d& p) const { return offset + slope.dot(p - center); }
  double tilt_deg() const { return std::atan(slope.norm()) * 180.0 / 3.14159265358979323846; }
};

struct GroundLayer {
  // NaN where Outside or Unknown, or Filled or Covered with no ground around
  // to fill from.
  HeightMap heights;
  // The rest have the same layout as heights.
  std::vector<GroundState> state;
  std::vector<Provenance> provenance;
  std::vector<std::uint16_t> observations;  // Walks that measured the cell.
  // The walk counting most toward the height (the latest, with latest_wins);
  // -1 for none.
  std::vector<int> source;
  // How far the farthest walk that measured (or patched) the cell is from the
  // result, in meters: 0 for one walk, large where walks disagree (something
  // changed, or a walk is misplaced). NaN where no walk gave a height.
  std::vector<float> spread;
  // Per walk, how its ground was aligned (zero for the walk that sets the frame).
  std::vector<WalkAlignment> alignment;

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
