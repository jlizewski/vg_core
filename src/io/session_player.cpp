#include "vg_core/io/session_player.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

namespace vg::io {

std::size_t play(SessionReader& reader,
                 const std::function<bool(const SessionMessage&)>& on_message,
                 const PlaybackOptions& options) {
  using Clock = std::chrono::steady_clock;
  std::size_t count = 0;
  std::optional<Timestamp> first;
  const Clock::time_point start = Clock::now();

  while (auto message = reader.next()) {
    const Timestamp t = timestamp_of(*message);
    if (options.rate > 0.0) {
      if (!first) {
        first = t;
      }
      const double offset_ns = static_cast<double>(t - *first) / options.rate;
      std::this_thread::sleep_until(start +
                                    std::chrono::duration_cast<Clock::duration>(
                                        std::chrono::duration<double, std::nano>(offset_ns)));
    }
    ++count;
    if (!on_message(*message)) {
      break;
    }
  }
  return count;
}

SessionMapper::SessionMapper(TsdfVolume& volume, Timestamp max_pose_age)
    : volume_(volume), max_pose_age_(max_pose_age) {}

void SessionMapper::handle(const SessionMessage& message) {
  if (const auto* pose = std::get_if<PoseStamped>(&message)) {
    if (pending_) {
      resolve(pose);
    }
    pose_ = *pose;
  } else if (const auto* status = std::get_if<TrackingStatus>(&message)) {
    tracking_ = status->state;
  } else if (const auto* transforms = std::get_if<StaticTransforms>(&message)) {
    extrinsics_.insert(extrinsics_.end(), transforms->transforms.begin(),
                       transforms->transforms.end());
  } else if (const auto* depth = std::get_if<DepthImage>(&message)) {
    if (pending_) {
      resolve(nullptr);
    }
    const bool tracking_ok =
        tracking_ != TrackingState::Limited && tracking_ != TrackingState::Lost;
    pending_ = PendingDepth{*depth, tracking_ok};
  }
}

void SessionMapper::flush() {
  if (pending_) {
    resolve(nullptr);
  }
}

void SessionMapper::resolve(const PoseStamped* next_pose) {
  const PendingDepth pending = std::move(*pending_);
  pending_.reset();
  if (!pending.tracking_ok) {
    ++stats_.skipped_tracking;
    return;
  }

  const Timestamp t = pending.depth.frame.timestamp;
  const PoseStamped* best = nullptr;
  Timestamp best_age = max_pose_age_;
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
  const std::string frame = "cam/" + pending.depth.camera;
  for (const StaticTransform& transform : extrinsics_) {
    if (transform.parent_frame == "body" && transform.child_frame == frame) {
      body_from_camera = transform.parent_from_child;
    }
  }
  volume_.integrate(pending.depth.frame, best->world_from_frame * body_from_camera);
  ++stats_.integrated;
}

}  // namespace vg::io
