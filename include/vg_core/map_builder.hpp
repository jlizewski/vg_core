#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "vg_core/depth_spool.hpp"
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

  // Keyframes: only depth frames taken after the camera has moved at least this
  // far (meters) or turned at least this much (radians) since the last
  // integrated frame are fused live; the rest are deferred, to be fused later
  // with integrate_deferred(). Both 0 (the default) fuses every frame live.
  double keyframe_translation = 0.0;
  double keyframe_rotation = 0.0;

  // Where deferred frames are kept: a file at this path (deleted when the
  // builder is destroyed), or memory if empty.
  std::string deferred_path;
};

// Builds the map live from a stream of sensor data, as it arrives on device or
// from a replayed session. Not thread-safe: call it from one thread (e.g. a
// mapping worker) and hand the height map updates to the UI.
//
// Each depth frame is paired with the body pose closest in time (the last pose
// before it or the first after it), so a frame waits for the next pose before
// it's integrated; call flush() at the end of a session.
//
// To stay real-time on a phone, the builder can fuse only keyframes live (see
// MapBuilderConfig) and defer the rest, or defer everything while the caller is
// falling behind (set_live()). Deferred frames keep the pose they were paired
// with, so integrate_deferred() can fuse them once recording has stopped and
// the finished map holds every frame.
class MapBuilder {
 public:
  struct Stats {
    std::size_t integrated = 0;
    std::size_t skipped_no_pose = 0;   // No pose close enough in time.
    std::size_t skipped_tracking = 0;  // Taken while tracking was unreliable.
    std::size_t deferred = 0;          // Set aside for integrate_deferred().
  };

  explicit MapBuilder(const MapBuilderConfig& config = {});

  // Fixed pose of a camera ("rgb", "left", ...) on the body. Identity if unset.
  void set_camera_extrinsics(const std::string& camera, const Eigen::Isometry3d& body_from_camera);

  // Pose of the body in the world, from the platform tracker.
  void add_pose(const PoseStamped& world_from_body);

  // Whether the tracker currently trusts its poses (ARCore/ARKit "normal").
  // Depth frames arriving while this is false are skipped.
  void set_tracking_ok(bool ok) { tracking_ok_ = ok; }

  // Whether depth frames added from now on may be fused live (keyframes only,
  // if keyframing is on). While false every frame is deferred, e.g. while the
  // mapping thread has a backlog. True by default.
  void set_live(bool live) { live_ = live; }

  void add_depth(const std::string& camera, const DepthFrame& frame);

  // Integrates or skips a depth frame still waiting for a pose.
  void flush();

  // Fuses up to max_frames deferred frames, oldest first, and returns how many
  // are still deferred. Call update() afterwards to see them in the height map.
  std::size_t integrate_deferred(std::size_t max_frames);

  // Number of frames waiting for integrate_deferred().
  std::size_t deferred_frames() const { return deferred_ ? deferred_->size() : 0; }

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
    bool live = true;
  };

  // Integrates pending_ with the closer of pose_ and next_pose, then clears it.
  void resolve(const PoseStamped* next_pose);

  // Whether a frame from camera at world_from_camera is far enough from the
  // last one integrated to be a keyframe.
  bool is_keyframe(const std::string& camera, const Eigen::Isometry3d& world_from_camera) const;

  void integrate(const std::string& camera, const DepthFrame& frame,
                 const Eigen::Isometry3d& world_from_camera);

  MapBuilderConfig config_;
  TsdfVolume volume_;
  LiveHeightMap height_map_;
  std::map<std::string, Eigen::Isometry3d> extrinsics_;
  std::optional<PoseStamped> pose_;
  std::optional<PendingDepth> pending_;
  bool tracking_ok_ = true;
  bool live_ = true;
  // Pose of each camera's last integrated frame, for keyframe selection.
  std::map<std::string, Eigen::Isometry3d> last_integrated_;
  std::unique_ptr<DepthSpool> deferred_;  // Created on first use.
  Stats stats_;
};

}  // namespace vg
