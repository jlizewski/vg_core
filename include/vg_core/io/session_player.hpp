#pragma once

#include <cstddef>
#include <functional>

#include "vg_core/io/session_reader.hpp"
#include "vg_core/map_builder.hpp"

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

// Feeds a session into a MapBuilder: poses, depth frames, camera extrinsics
// from /tf_static (body to cam/<name>) and tracking status. Call flush() on the
// builder at the end of the session.
class SessionMapper {
 public:
  explicit SessionMapper(MapBuilder& builder) : builder_(builder) {}

  // Handles one message; ignores types the map builder doesn't use.
  void handle(const SessionMessage& message);

 private:
  MapBuilder& builder_;
};

}  // namespace vg::io
