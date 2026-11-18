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
#include "vg_core/semantics.hpp"
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

  // Elements (GroundSegmentation::elements): what stands through this band
  // of heights above the ground, in meters, in groups of at least
  // element_min_columns voxel columns. At about waist height a fence is just
  // a fence and a trunk just a trunk, apart from the shrubs and beds at their
  // foot and the canopy over them, which whole segments join together.
  double element_band_low = 0.6;
  double element_band_high = 1.4;
  int element_min_columns = 3;
  // A column counts with at least this many voxels in the band: a stray
  // voxel or two bridging two things doesn't join them.
  int element_min_band_voxels = 2;

  // Semantic labels, when the voxels have them (see segment_ground()):
  // geometry finds the ground, labels help it. A column whose bottom voxel
  // is labeled one of not_ground_labels (e.g. a low plant read as raised
  // ground) isn't ground, and is patched like the ground under objects. The
  // ground the rest is judged against is the region with the most columns
  // labeled ground rather than simply the largest.
  //
  // With rescue_isolated_ground, a patch with nothing to join is also ground,
  // however small, if at least min_ground_share of its labeled columns are
  // labeled ground and it rises no more steeply than rescue_max_slope (rise
  // over run) toward the nearest ground. Off by default: on a real yard it
  // also took in surfaces a meter up (a deck or porch floor, labeled ground,
  // can't be told from ground rising behind an obstacle).
  bool use_labels = true;
  SemanticMask not_ground_labels = mask_of(SemanticClass::Plant) | mask_of(SemanticClass::Tree) |
                                   mask_of(SemanticClass::Structure) |
                                   mask_of(SemanticClass::Dynamic);
  bool rescue_isolated_ground = false;
  double min_ground_share = 0.5;
  double rescue_max_slope = 0.25;
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
  // Area of the ground map's cells it has voxels over, in square meters.
  double footprint_area = 0.0;
  // Its footprint as an oriented box, from the principal axes of its voxels
  // seen from above: the center (m), its extent along the major axis
  // (length) and across it (width), and the major axis' direction (radians
  // from x, in [0, pi)).
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double length = 0.0;
  double width = 0.0;
  double axis = 0.0;
  // The class most of its labeled voxels have (Unknown if none are), and the
  // share of its labeled voxels that have it.
  SemanticClass label = SemanticClass::Unknown;
  double label_share = 0.0;
};

// What a walk knows about the ground in one cell.
enum class GroundCell : std::uint8_t {
  None = 0,         // Not seen: beyond the walk, or nothing to patch from.
  Measured = 1,     // Ground seen there.
  UnderObject = 2,  // Hidden under an object standing on or over it; patched.
  SmallHole = 3,    // Unseen, enclosed and small (max_hole_area); patched.
  LargeHole = 4,    // Unseen and enclosed, but too large to patch; no height.
};

struct GroundMap {
  // Ground surface height per cell, holes patched. NaN where there is no
  // ground and nothing to patch from (e.g. beyond the scanned area).
  HeightMap heights;
  // The rest have the same layout as heights.
  // 1 where the height was interpolated rather than observed.
  std::vector<std::uint8_t> filled;
  std::vector<GroundCell> state;
  // Measured cells: the total fused weight of the ground seen there (the
  // weights given to segment_ground(), summed over the cell's columns); how
  // much to trust it against another walk's. 0 elsewhere, or without weights.
  std::vector<float> weight;
  // The largest segment (index into GroundSegmentation::segments) with voxels
  // over the cell, standing on it or overhanging; -1 for none.
  std::vector<int> object;

  std::size_t index(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(heights.width) +
           static_cast<std::size_t>(x);
  }
  bool is_filled(int x, int y) const { return filled[index(x, y)] != 0; }
  GroundCell state_at(int x, int y) const { return state[index(x, y)]; }
};

// Something standing through the element band (GroundConfig): a post, a
// trunk, a fence, a wall, as seen at that height.
struct MapElement {
  // Its cross-section in the band as an oriented box (as MapSegment's).
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double length = 0.0;
  double width = 0.0;
  double axis = 0.0;
  std::size_t columns = 0;  // Voxel columns it has in the band.
  double area = 0.0;        // Square meters.
  // The top of what stands in those columns, above the ground (m), and the
  // class most of its labeled voxels in the band have.
  double top_above_ground = 0.0;
  SemanticClass label = SemanticClass::Unknown;
};

struct GroundSegmentation {
  std::vector<Eigen::Vector3i> ground_voxels;
  // Columns left out of the ground because of their label.
  std::size_t columns_not_ground_by_label = 0;
  // Largest first.
  std::vector<MapSegment> segments;
  GroundMap ground;
  // Largest first.
  std::vector<MapElement> elements;
};

// Splits occupied voxels (edge voxel_size meters) into ground and segments.
//
// The ground is grown from the largest region of column bottoms (the lowest
// voxel under each xy) that rises no faster than max_slope, then extended to
// other such regions that line up with it across unseen gaps. Tops of raised
// beds and other surfaces with no visible ground below stay out because their
// edges drop steeply to the ground. Cost is linear in the number of voxels
// plus the xy area they cover; it reprocesses the whole map on each call.
//
// labels, if not empty, is each voxel's semantic class (as from
// TsdfVolume::label_of(), Unknown where it has none), in the same order as
// voxels; see GroundConfig::use_labels. weights, if not empty, is each
// voxel's fused weight (TsdfVolume::weight_at()), in the same order, for
// GroundMap::weight.
GroundSegmentation segment_ground(const std::vector<Eigen::Vector3i>& voxels, double voxel_size,
                                  const GroundConfig& config = {},
                                  const std::vector<SemanticClass>& labels = {},
                                  const std::vector<float>& weights = {});

// Fills the cells of map marked in fill smoothly from the cells with heights
// around them (each settles to the mean of its neighbors: Laplace's
// equation), and marks them in filled. A region that touches no cell with a
// height stays NaN. filled must have map's size.
void fill_height_holes(HeightMap& map, const std::vector<char>& fill,
                       std::vector<std::uint8_t>& filled);

// Segments the occupied voxels of a volume, with their weights.
GroundSegmentation segment_ground(const TsdfVolume& volume, const GroundConfig& config = {});

}  // namespace vg
