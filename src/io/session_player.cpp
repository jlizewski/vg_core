#include "vg_core/io/session_player.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
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

struct SessionPlayer::Impl {
  using Clock = std::chrono::steady_clock;

  // How far behind schedule the host may fall before playback moves on in real
  // time from where it is.
  static constexpr Clock::duration kMaxLag = std::chrono::seconds(1);

  explicit Impl(const std::filesystem::path& path) : reader(path) {}

  SessionReader reader;  // Used by next() only.

  mutable std::mutex mutex;
  std::condition_variable changed;
  double rate = 1.0;
  bool paused = false;
  bool stopped = false;
  std::optional<Timestamp> start;
  Timestamp end = 0;
  Timestamp last = 0;  // Time of the last message delivered.

  // The playhead was at session time anchor_time at anchor_wall, and has been
  // moving at rate since unless paused.
  bool anchored = false;
  Timestamp anchor_time = 0;
  Clock::time_point anchor_wall;

  bool real_time() const { return rate > 0.0; }

  // When a message at session time t is due.
  Clock::time_point due(Timestamp t) const {
    const double offset_ns = static_cast<double>(t - anchor_time) / rate;
    return anchor_wall + std::chrono::duration_cast<Clock::duration>(
                             std::chrono::duration<double, std::nano>(offset_ns));
  }

  // Moves the anchor to now, keeping the playhead where it is; call before
  // changing rate or paused.
  void reanchor(Clock::time_point now) {
    if (!real_time()) {
      anchor_time = last;  // Playing flat out: the playhead is the last message.
    } else if (anchored && !paused) {
      anchor_time += static_cast<Timestamp>(
          std::chrono::duration<double, std::nano>(now - anchor_wall).count() * rate);
    }
    anchor_wall = now;
  }
};

SessionPlayer::SessionPlayer(const std::filesystem::path& path, double rate)
    : impl_(std::make_unique<Impl>(path)) {
  impl_->rate = rate;
  if (const auto range = impl_->reader.time_range()) {
    impl_->start = range->first;
    impl_->end = range->second;
  }
}

SessionPlayer::~SessionPlayer() = default;

std::optional<SessionMessage> SessionPlayer::next() {
  Impl& d = *impl_;
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    if (d.stopped) {
      return std::nullopt;
    }
  }
  std::optional<SessionMessage> message = d.reader.next();
  if (!message) {
    return std::nullopt;
  }
  const Timestamp t = timestamp_of(*message);

  std::unique_lock<std::mutex> lock(d.mutex);
  if (!d.anchored) {
    d.anchored = true;
    d.anchor_time = t;
    d.anchor_wall = Impl::Clock::now();
    d.last = t;
    if (!d.start) {
      d.start = t;
    }
  }
  for (;;) {
    if (d.stopped) {
      return std::nullopt;
    }
    if (d.paused) {
      d.changed.wait(lock);
      continue;
    }
    const auto now = Impl::Clock::now();
    if (!d.real_time()) {
      break;
    }
    const auto due = d.due(t);
    if (now >= due) {
      if (now - due > Impl::kMaxLag) {
        d.anchor_time = t;
        d.anchor_wall = now;
      }
      break;
    }
    d.changed.wait_until(lock, due);
  }
  d.last = std::max(d.last, t);
  return message;
}

void SessionPlayer::set_rate(double rate) {
  {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->reanchor(Impl::Clock::now());
    impl_->rate = rate;
  }
  impl_->changed.notify_all();
}

double SessionPlayer::rate() const {
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->rate;
}

void SessionPlayer::set_paused(bool paused) {
  {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->paused == paused) {
      return;
    }
    impl_->reanchor(Impl::Clock::now());
    impl_->paused = paused;
  }
  impl_->changed.notify_all();
}

bool SessionPlayer::paused() const {
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->paused;
}

void SessionPlayer::stop() {
  {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stopped = true;
  }
  impl_->changed.notify_all();
}

Timestamp SessionPlayer::position() const {
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->start ? impl_->last - *impl_->start : 0;
}

Timestamp SessionPlayer::duration() const {
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->start ? std::max<Timestamp>(impl_->end - *impl_->start, 0) : 0;
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
