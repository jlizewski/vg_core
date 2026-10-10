#include "vg_core/pose_jumps.hpp"

#include <cmath>

namespace vg {

bool is_pose_jump(const PoseStamped& a, const PoseStamped& b, const PoseJumpConfig& config) {
  const Timestamp elapsed = b.timestamp - a.timestamp;
  if (elapsed > config.max_gap) {
    return false;
  }
  const double dt = static_cast<double>(elapsed) * 1e-9;
  const Eigen::Isometry3d delta = a.world_from_frame.inverse() * b.world_from_frame;
  const double translation = delta.translation().norm();
  const double rotation = std::abs(Eigen::AngleAxisd(delta.linear()).angle());
  return (translation > config.min_translation && translation > config.max_speed * dt) ||
         (rotation > config.min_rotation && rotation > config.max_angular_speed * dt);
}

PoseStamped PoseJumpFilter::correct(const PoseStamped& pose) {
  if (last_ && is_pose_jump(*last_, pose, config_)) {
    // Where the tracker would have put this pose had it kept moving as it
    // was: the previous step's motion, repeated (or none if there is none).
    Eigen::Isometry3d expected = last_->world_from_frame;
    if (before_last_ && last_->timestamp > before_last_->timestamp) {
      const Eigen::Isometry3d step =
          before_last_->world_from_frame.inverse() * last_->world_from_frame;
      if (!is_pose_jump(*before_last_, *last_, config_)) {
        expected = expected * step;
      }
    }
    // The tracker moved its world by snap = pose * expected^-1; undo it from
    // here on.
    const Eigen::Isometry3d snap = pose.world_from_frame * expected.inverse();
    correction_ = correction_ * snap.inverse();
    ++jumps_;
  }
  before_last_ = last_;
  last_ = pose;
  PoseStamped corrected = pose;
  corrected.world_from_frame = correction_ * pose.world_from_frame;
  return corrected;
}

}  // namespace vg
