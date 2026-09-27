#include "vg_core/io/session_player.hpp"

#include <chrono>
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

void SessionMapper::handle(const SessionMessage& message) {
  if (const auto* pose = std::get_if<PoseStamped>(&message)) {
    builder_.add_pose(*pose);
  } else if (const auto* depth = std::get_if<DepthImage>(&message)) {
    builder_.add_depth(depth->camera, depth->frame);
  } else if (const auto* status = std::get_if<TrackingStatus>(&message)) {
    builder_.set_tracking_ok(status->state != TrackingState::Limited &&
                             status->state != TrackingState::Lost);
  } else if (const auto* transforms = std::get_if<StaticTransforms>(&message)) {
    const std::string prefix = "cam/";
    for (const StaticTransform& t : transforms->transforms) {
      if (t.parent_frame == "body" && t.child_frame.compare(0, prefix.size(), prefix) == 0) {
        builder_.set_camera_extrinsics(t.child_frame.substr(prefix.size()), t.parent_from_child);
      }
    }
  }
}

}  // namespace vg::io
