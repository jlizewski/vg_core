#pragma once

// Removing the jumps a platform tracker (ARCore, ARKit) makes when it
// corrects its own drift. The tracker snaps its pose, and with it the world
// frame, by several cm in one frame; everything mapped after the snap then
// sits offset from what was mapped before it, as a ghost copy of the same
// ground or wall. Undoing each snap keeps the whole walk in one consistent
// frame (that of its start).

#include <cstddef>
#include <optional>

#include "vg_core/sensor_data.hpp"

namespace vg {

struct PoseJumpConfig {
  // Motion between consecutive poses faster than a hand-held phone moves.
  double max_speed = 2.0;                             // m/s
  double max_angular_speed = 3.14159265358979323846;  // rad/s (180 deg/s)
  // Jumps smaller than these are ignored, however short the interval.
  double min_translation = 0.03;                              // m
  double min_rotation = 3.0 * 3.14159265358979323846 / 180.0;  // rad (3 deg)
  // Consecutive poses further apart than this are a gap, not a jump.
  Timestamp max_gap = 500'000'000;
};

// Whether the move from pose a to the next pose b is a jump.
bool is_pose_jump(const PoseStamped& a, const PoseStamped& b, const PoseJumpConfig& config);

// Takes a tracker's poses in time order and returns them with every jump so
// far undone: across a jump, the motion is taken to continue as it was just
// before, and the rest of the track is moved to follow on from there.
class PoseJumpFilter {
 public:
  explicit PoseJumpFilter(const PoseJumpConfig& config = {}) : config_(config) {}

  PoseStamped correct(const PoseStamped& pose);

  // Jumps undone so far.
  std::size_t jumps() const { return jumps_; }

  // What correct() currently applies: corrected = correction() * tracker pose.
  const Eigen::Isometry3d& correction() const { return correction_; }

 private:
  PoseJumpConfig config_;
  // The last two poses as the tracker gave them.
  std::optional<PoseStamped> last_;
  std::optional<PoseStamped> before_last_;
  Eigen::Isometry3d correction_ = Eigen::Isometry3d::Identity();
  std::size_t jumps_ = 0;
};

}  // namespace vg
