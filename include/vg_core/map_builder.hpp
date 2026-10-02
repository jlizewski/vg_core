#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "vg_core/height_map.hpp"
#include "vg_core/live_height_map.hpp"
#include "vg_core/sensor_data.hpp"
#include "vg_core/tsdf_volume.hpp"

namespace vg {

struct MapBuilderConfig {
  TsdfConfig tsdf;
  // Cell size of the live height map, in meters.
  double height_cell_size = 0.05;
  // How far (ns) a pose may be from a depth frame's time to be used for it.
  Timestamp max_pose_age = 50'000'000;
};

// Builds the map live from a stream of sensor data, as it arrives on device or
// from a replayed session. Not thread-safe: call it from one thread (e.g. a
// mapping worker) and hand the height map updates to the UI.
//
// Each depth frame is paired with the body pose closest in time (the last pose
// before it or the first after it), so a frame waits for the next pose before
// it's integrated; call flush() at the end of a session.
class MapBuilder {
 public:
  struct Stats {
    std::size_t integrated = 0;
    std::size_t skipped_no_pose = 0;   // No pose close enough in time.
    std::size_t skipped_tracking = 0;  // Taken while tracking was unreliable.
  };

  explicit MapBuilder(const MapBuilderConfig& config = {});

  // Fixed pose of a camera ("rgb", "left", ...) on the body. Identity if unset.
  void set_camera_extrinsics(const std::string& camera, const Eigen::Isometry3d& body_from_camera);

  // Pose of the body in the world, from the platform tracker.
  void add_pose(const PoseStamped& world_from_body);

  // Whether the tracker currently trusts its poses (ARCore/ARKit "normal").
  // Depth frames arriving while this is false are skipped.
  void set_tracking_ok(bool ok) { tracking_ok_ = ok; }

  void add_depth(const std::string& camera, const DepthFrame& frame);

  // Integrates or skips a depth frame still waiting for a pose.
  void flush();

  // What changed since the previous update(), for a live view to redraw.
  struct Update {
    // Height map cells whose height changed (NaN: cell cleared).
    std::vector<HeightCell> height_cells;
    // Voxel blocks whose contents may have changed. For a 3D view, re-read
    // each one with volume().occupied_voxels(block) and replace what was shown.
    std::vector<Eigen::Vector3i> blocks;
  };

  // Brings the live height map up to date with everything integrated so far
  // and reports what changed. Call it at the rate you want to refresh the
  // display; the cost depends on how much changed.
  Update update();

  // The live height map as of the last update() call.
  HeightMap height_map() const { return height_map_.snapshot(); }

  // Number of height map cells with a surface, as of the last update().
  std::size_t height_map_cells() const { return height_map_.size(); }

  const TsdfVolume& volume() const { return volume_; }
  const Stats& stats() const { return stats_; }

 private:
  struct PendingDepth {
    std::string camera;
    DepthFrame frame;
    bool tracking_ok = true;
  };

  // Integrates pending_ with the closer of pose_ and next_pose, then clears it.
  void resolve(const PoseStamped* next_pose);

  MapBuilderConfig config_;
  TsdfVolume volume_;
  LiveHeightMap height_map_;
  std::map<std::string, Eigen::Isometry3d> extrinsics_;
  std::optional<PoseStamped> pose_;
  std::optional<PendingDepth> pending_;
  bool tracking_ok_ = true;
  Stats stats_;
};

}  // namespace vg
