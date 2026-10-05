#include "vg_core/map_builder.hpp"

#include <cmath>
#include <cstdlib>

namespace vg {

MapBuilder::MapBuilder(const MapBuilderConfig& config)
    : config_(config), volume_(config.tsdf), height_map_(config.height_cell_size) {}

void MapBuilder::set_camera_extrinsics(const std::string& camera,
                                       const Eigen::Isometry3d& body_from_camera) {
  extrinsics_[camera] = body_from_camera;
}

void MapBuilder::add_pose(const PoseStamped& world_from_body) {
  if (pending_) {
    resolve(&world_from_body);
  }
  pose_ = world_from_body;
}

void MapBuilder::add_depth(const std::string& camera, const DepthFrame& frame) {
  if (pending_) {
    resolve(nullptr);
  }
  pending_ = PendingDepth{camera, frame, tracking_ok_, live_};
}

void MapBuilder::flush() {
  if (pending_) {
    resolve(nullptr);
  }
}

void MapBuilder::resolve(const PoseStamped* next_pose) {
  const PendingDepth pending = std::move(*pending_);
  pending_.reset();
  if (!pending.tracking_ok) {
    ++stats_.skipped_tracking;
    return;
  }

  const Timestamp t = pending.frame.timestamp;
  const PoseStamped* best = nullptr;
  Timestamp best_age = config_.max_pose_age;
  const PoseStamped* previous = pose_ ? &*pose_ : nullptr;
  for (const PoseStamped* candidate : {previous, next_pose}) {
    if (candidate == nullptr) {
      continue;
    }
    const Timestamp age = std::llabs(candidate->timestamp - t);
    if (age <= best_age) {
      best = candidate;
      best_age = age;
    }
  }
  if (best == nullptr) {
    ++stats_.skipped_no_pose;
    return;
  }

  Eigen::Isometry3d body_from_camera = Eigen::Isometry3d::Identity();
  if (const auto it = extrinsics_.find(pending.camera); it != extrinsics_.end()) {
    body_from_camera = it->second;
  }
  const Eigen::Isometry3d world_from_camera = best->world_from_frame * body_from_camera;
  if (pending.live && is_keyframe(pending.camera, world_from_camera)) {
    integrate(pending.camera, pending.frame, world_from_camera);
    return;
  }
  if (!deferred_) {
    deferred_ = config_.deferred_path.empty() ? std::make_unique<DepthSpool>()
                                              : std::make_unique<DepthSpool>(config_.deferred_path);
  }
  deferred_->push(pending.camera, pending.frame, world_from_camera);
  ++stats_.deferred;
}

bool MapBuilder::is_keyframe(const std::string& camera,
                             const Eigen::Isometry3d& world_from_camera) const {
  if (config_.keyframe_translation <= 0.0 && config_.keyframe_rotation <= 0.0) {
    return true;
  }
  const auto it = last_integrated_.find(camera);
  if (it == last_integrated_.end()) {
    return true;
  }
  const Eigen::Isometry3d delta = it->second.inverse() * world_from_camera;
  const double angle = Eigen::AngleAxisd(delta.linear()).angle();
  return (config_.keyframe_translation > 0.0 &&
          delta.translation().norm() >= config_.keyframe_translation) ||
         (config_.keyframe_rotation > 0.0 && std::abs(angle) >= config_.keyframe_rotation);
}

void MapBuilder::integrate(const std::string& camera, const DepthFrame& frame,
                           const Eigen::Isometry3d& world_from_camera) {
  volume_.integrate(frame, world_from_camera);
  last_integrated_[camera] = world_from_camera;
  ++stats_.integrated;
}

std::size_t MapBuilder::integrate_deferred(std::size_t max_frames) {
  for (std::size_t i = 0; i < max_frames && deferred_frames() > 0; ++i) {
    const auto entry = deferred_->pop();
    volume_.integrate(entry->frame, entry->world_from_camera);
    ++stats_.integrated;
  }
  return deferred_frames();
}

MapBuilder::Update MapBuilder::update() {
  Update result;
  result.blocks = volume_.take_changed_blocks();
  result.height_cells = height_map_.update(volume_, result.blocks);
  return result;
}

}  // namespace vg
