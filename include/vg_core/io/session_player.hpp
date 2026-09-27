#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

#include "vg_core/io/session_reader.hpp"
#include "vg_core/tsdf_volume.hpp"

namespace vg::io {

struct PlaybackOptions {
  // Playback speed relative to recording: 1 is real time, 2 twice as fast.
  // 0 or less plays as fast as possible.
  double rate = 1.0;
};

// Sends every remaining message of reader to on_message, pacing them to their
// recorded timing scaled by options.rate. on_message returns false to stop.
// Returns the number of messages delivered.
std::size_t play(SessionReader& reader,
                 const std::function<bool(const SessionMessage&)>& on_message,
                 const PlaybackOptions& options = {});

// Feeds a session's depth frames into a TSDF volume. Each depth frame is paired
// with the body pose closest in time (the last pose before it or the first
// after it), combined with the body-to-camera transform from /tf_static
// (identity if absent). A frame waits for the next pose, so call flush() at the
// end of the session.
class SessionMapper {
 public:
  struct Stats {
    std::size_t integrated = 0;
    std::size_t skipped_no_pose = 0;   // No pose close enough in time.
    std::size_t skipped_tracking = 0;  // Tracker reported limited or lost.
  };

  // max_pose_age: how far (ns) a pose may be from the depth frame's time.
  explicit SessionMapper(TsdfVolume& volume, Timestamp max_pose_age = 50'000'000);

  // Handles one message; ignores types it doesn't use.
  void handle(const SessionMessage& message);

  // Integrates or skips a depth frame still waiting for a pose.
  void flush();

  const Stats& stats() const { return stats_; }

 private:
  struct PendingDepth {
    DepthImage depth;
    bool tracking_ok = true;
  };

  // Integrates pending_ with the closer of pose_ and next_pose, then clears it.
  void resolve(const PoseStamped* next_pose);

  TsdfVolume& volume_;
  Timestamp max_pose_age_;
  std::optional<PoseStamped> pose_;
  std::optional<PendingDepth> pending_;
  TrackingState tracking_ = TrackingState::Unknown;
  std::vector<StaticTransform> extrinsics_;
  Stats stats_;
};

}  // namespace vg::io
