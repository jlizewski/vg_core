#include "vg_core/map_builder.hpp"

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
  pending_ = PendingDepth{camera, frame, tracking_ok_};
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
  volume_.integrate(pending.frame, best->world_from_frame * body_from_camera);
  ++stats_.integrated;
}

MapBuilder::Update MapBuilder::update() {
  Update result;
  result.blocks = volume_.take_changed_blocks();
  result.height_cells = height_map_.update(volume_, result.blocks);
  return result;
}

}  // namespace vg
