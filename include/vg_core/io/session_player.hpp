#pragma once

#include <cstddef>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>

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

// Plays a session back as if it were happening live, so a host app can drive
// its live pipeline from a recording, e.g. to work on the live map and the
// scanning guidance without going outside. Unlike play(), the host pulls the
// messages, and playback can be paused and sped up or slowed down while it runs.
//
// A host that falls more than a second behind (stopped in a debugger, say)
// carries on in real time from where it is instead of racing to catch up.
//
// Call next() from one playback thread; the other methods from any thread.
class SessionPlayer {
 public:
  // Throws std::runtime_error if the file can't be read.
  explicit SessionPlayer(const std::filesystem::path& path, double rate = 1.0);
  ~SessionPlayer();
  SessionPlayer(const SessionPlayer&) = delete;
  SessionPlayer& operator=(const SessionPlayer&) = delete;

  // Waits until the next message is due and returns it, or nullopt at the end
  // of the session or once stop() has been called.
  std::optional<SessionMessage> next();

  // Playback speed relative to recording: 1 is real time, 2 twice as fast.
  // 0 or less plays as fast as the host takes the messages.
  void set_rate(double rate);
  double rate() const;

  // While paused, next() waits.
  void set_paused(bool paused);
  bool paused() const;

  // Ends playback: next() returns nullopt, straight away if it is waiting.
  void stop();

  // Session time of the last message next() returned, in ns from the start of
  // the session.
  Timestamp position() const;

  // Length of the session in ns, or 0 if the file doesn't say.
  Timestamp duration() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

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
