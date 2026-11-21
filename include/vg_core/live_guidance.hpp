#pragma once

// Ground-fill guidance while walking (docs/design/guided-mapping.md, stage
// 2.4): the site's holes (guidance.hpp) closing as the live map sees them.
//
// A walk recorded in the same tracking session as one of the site's visits
// is already in the site frame (D17): that visit's placement places the live
// map too. As the live height map (live_height_map.hpp) changes, each hole
// cell it covers with a surface no higher than max_height over the hole's
// expected ground (a plane through the measured ground around the hole) is
// seen: ground, or something standing on it, either way no longer unknown.
// A surface far above (a canopy) doesn't count: the ground under it is
// still unseen. A hole is done when done_share of it is seen.
//
// This is only a live estimate; the walk's real contribution is worked out
// when it's added to the site after the step (vg site add), which may still
// leave part of a hole unknown, or mark it filled or covered.

#include <Eigen/Geometry>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "vg_core/ground_layer.hpp"
#include "vg_core/guidance.hpp"
#include "vg_core/live_height_map.hpp"
#include "vg_core/registration.hpp"

namespace vg {

struct LiveGuidanceConfig {
  // A live surface this far (m) above a hole's expected ground or less sees
  // it; more (a canopy, an eave) doesn't. Below it by more than max_depth
  // doesn't either (noise).
  double max_height = 2.0;
  double max_depth = 0.5;
  // A hole is done when this share of it is seen.
  double done_share = 0.9;
  // The plan the spots come from (as vg site ground made it).
  GuidanceConfig guidance;
};

struct LiveTarget {
  int rank = 0;       // 1 = largest, as in layers/guidance/targets.csv.
  int order = 0;      // When to visit it, as planned.
  double area = 0.0;  // m^2.
  Eigen::Vector2d center = Eigen::Vector2d::Zero();  // Site frame.
  std::vector<LookSpot> spots;
  std::size_t cells = 0;
  std::size_t seen_cells = 0;
  double seen = 0.0;  // Share seen while walking, 0-1.
  bool done = false;
};

// Where to go next: the nearest spot (from where the camera is) of any hole
// that isn't done, and how to turn to walk toward it.
struct NextSpot {
  int rank = 0;                                        // Its hole.
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  // Site frame.
  double heading = 0.0;                                // Which way to face there, radians from x.
  double distance = 0.0;                               // From the camera, m (horizontal).
  // How far to turn to walk toward it: radians, positive to the left
  // (counter-clockwise seen from above), in (-pi, pi].
  double turn = 0.0;
};

class LiveGuidance {
 public:
  // From a ground layer (its heights and states) and the placement of the
  // walk's tracking frame in the site.
  LiveGuidance(const GroundLayer& ground, const MapTransform& site_from_world,
               const LiveGuidanceConfig& config = {});

  // For the site in site_dir and the tracking frame a recording carries.
  // Throws std::runtime_error saying why if there's no ground layer, or no
  // visit of the site in that tracking frame (the walk can't be placed
  // without one: relocalization comes later).
  static LiveGuidance for_site(const std::filesystem::path& site_dir,
                               const std::string& tracking_frame,
                               const LiveGuidanceConfig& config = {});

  // Cells of the live height map that changed (LiveHeightMap::update, or the
  // C API's vg_map_builder_update), in the walk's world frame, with its cell
  // size.
  void update(const std::vector<HeightCell>& changed, double cell_size);

  // Where the camera is (optical frame: z forward), in the walk's world frame.
  void set_camera_pose(const Eigen::Isometry3d& world_from_camera);

  const std::vector<LiveTarget>& targets() const { return targets_; }
  // The next spot to go to; nullopt when every hole is done, or before a
  // camera pose is known.
  std::optional<NextSpot> next() const;
  // Every hole done.
  bool done() const;
  // Unknown ground not yet seen, m^2.
  double unseen_area() const;
  // Where the camera is in the site frame (x, y), once known.
  std::optional<Eigen::Vector2d> position() const;

  // Every hole cell: its center (site frame), its hole's index in targets(),
  // and whether it's been seen. For checking the live estimate against what
  // the walk really did once added to the site.
  struct HoleCellState {
    Eigen::Vector2d center;
    int target = 0;
    bool seen = false;
  };
  std::vector<HoleCellState> hole_cells() const;

 private:
  void see(const Eigen::Vector2d& site_xy, double site_z);

  LiveGuidanceConfig config_;
  MapTransform site_from_world_;
  HeightMap grid_;                      // The ground layer's grid (heights unused).
  std::vector<int> hole_of_;            // Per grid cell: index into targets_, or -1.
  std::vector<char> seen_;              // Per grid cell.
  std::vector<Eigen::Vector3d> plane_;  // Per hole: z = a x + b y + c (site frame).
  std::vector<LiveTarget> targets_;
  std::optional<Eigen::Vector2d> position_;
  double facing_ = 0.0;
};

}  // namespace vg
