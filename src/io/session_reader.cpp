#include "vg_core/io/session_reader.hpp"

#include <algorithm>
#include <map>
#include <mcap/reader.hpp>
#include <stdexcept>
#include <string>
#include <string_view>

#include "messages.hpp"

namespace vg::io {
namespace {

// Depth confidence, kept internal until it's attached to its depth image.
struct Confidence {
  std::string camera;
  Timestamp timestamp = 0;
  std::vector<std::uint8_t> values;
};

using Decoded = std::variant<SessionMessage, Confidence>;

// How far out of log-time order messages may be in a file read without its
// index. Writers buffer sensors separately (vg_android's IMU arrives in
// batches up to ~0.5 s behind the camera), so file order is only roughly
// time order.
constexpr mcap::Timestamp kReorderWindow = 2'000'000'000;

// True if the file was closed cleanly, so it can be read in time order from
// its index. A recording cut off before close() (app killed, crash, phone
// out of space) has its chunks but no summary or footer.
bool has_message_index(mcap::McapReader& reader) {
  if (!reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok()) {
    return false;
  }
  const auto& chunks = reader.chunkIndexes();
  return std::any_of(chunks.begin(), chunks.end(),
                     [](const mcap::ChunkIndex& c) { return c.messageIndexLength > 0; });
}

bool starts_with(std::string_view s, std::string_view prefix) {
  return s.substr(0, prefix.size()) == prefix;
}

}  // namespace

struct SessionReader::Impl {
  mcap::McapReader reader;
  std::optional<mcap::LinearMessageView> view;
  std::optional<mcap::LinearMessageView::Iterator> it;
  std::optional<Decoded> pending;
  bool indexed = true;
  // Without an index: decoded messages waiting to be sorted into time order,
  // and the newest log time seen so far.
  std::multimap<mcap::Timestamp, Decoded> reorder;
  mcap::Timestamp newest = 0;
  std::map<std::string, CameraIntrinsics> camera_calibrations;
  std::map<std::string, CameraIntrinsics> depth_calibrations;

  // Next decoded message in time order, or nullopt at the end.
  std::optional<Decoded> next_raw() {
    if (indexed) {
      return next_in_file();
    }
    // Hold messages until nothing older can still turn up, then release the
    // oldest. Equal log times keep their file order.
    while (reorder.empty() || newest - reorder.begin()->first < kReorderWindow) {
      mcap::Timestamp log_time = 0;
      std::optional<Decoded> decoded = next_in_file(&log_time);
      if (!decoded) {
        break;
      }
      newest = std::max(newest, log_time);
      reorder.emplace(log_time, std::move(*decoded));
    }
    if (reorder.empty()) {
      return std::nullopt;
    }
    Decoded oldest = std::move(reorder.begin()->second);
    reorder.erase(reorder.begin());
    return oldest;
  }

  // Next decoded message in the order the view yields them.
  std::optional<Decoded> next_in_file(mcap::Timestamp* log_time = nullptr) {
    while (*it != view->end()) {
      const mcap::MessageView& mv = **it;
      std::optional<Decoded> decoded = decode(
          mv.channel->topic, {reinterpret_cast<const char*>(mv.message.data), mv.message.dataSize});
      if (log_time != nullptr) {
        *log_time = mv.message.logTime;
      }
      ++*it;
      if (decoded) {
        return decoded;
      }
    }
    return std::nullopt;
  }

  std::optional<Decoded> decode(const std::string& topic, std::string_view data) {
    using namespace detail;
    if (topic == "/vg/session") {
      return SessionMessage{decode_session_info(data)};
    }
    if (topic == "/tf_static") {
      return SessionMessage{decode_static_transforms(data)};
    }
    if (topic == "/pose") {
      return SessionMessage{decode_pose(data)};
    }
    if (topic == "/pose/status") {
      return SessionMessage{decode_tracking_status(data)};
    }
    if (topic == "/imu") {
      return SessionMessage{decode_imu(data)};
    }
    if (topic == "/gps") {
      return SessionMessage{decode_gps(data)};
    }
    if (!starts_with(topic, "/cam/")) {
      return std::nullopt;  // Unknown topic.
    }
    const std::string_view rest = std::string_view(topic).substr(5);
    const std::string camera(rest.substr(0, rest.find('/')));
    const std::string_view kind = rest.substr(std::min(rest.size(), camera.size() + 1));

    if (kind == "calibration") {
      CameraCalibration c;
      c.camera = camera;
      c.intrinsics = decode_calibration(data);
      camera_calibrations[camera] = c.intrinsics;
      return SessionMessage{std::move(c)};
    }
    if (kind == "image") {
      CameraImage image = decode_image(data);
      image.camera = camera;
      return SessionMessage{std::move(image)};
    }
    if (kind == "depth/calibration") {
      depth_calibrations[camera] = decode_calibration(data);
      return std::nullopt;  // Attached to the depth images that follow.
    }
    if (kind == "depth") {
      const RawImage raw = decode_raw_image(data);
      DepthImage depth;
      depth.camera = camera;
      depth.frame.timestamp = raw.timestamp;
      depth.frame.intrinsics = depth_intrinsics(camera, raw);
      depth.frame.depth = depth_from_raw(raw);
      return SessionMessage{std::move(depth)};
    }
    if (kind == "depth/confidence") {
      const RawImage raw = decode_raw_image(data);
      return Confidence{camera, raw.timestamp, confidence_from_raw(raw)};
    }
    return std::nullopt;
  }

  CameraIntrinsics depth_intrinsics(const std::string& camera, const detail::RawImage& raw) {
    for (const auto* calibrations : {&depth_calibrations, &camera_calibrations}) {
      auto it_cal = calibrations->find(camera);
      if (it_cal != calibrations->end() && it_cal->second.width == raw.width &&
          it_cal->second.height == raw.height) {
        return it_cal->second;
      }
    }
    throw std::runtime_error("session: no calibration matching depth image size for camera " +
                             camera);
  }
};

SessionReader::SessionReader(const std::filesystem::path& path) : impl_(std::make_unique<Impl>()) {
  const mcap::Status status = impl_->reader.open(path.string());
  if (!status.ok()) {
    throw std::runtime_error("session: cannot open " + path.string() + ": " + status.message);
  }
  mcap::ReadMessageOptions options;
  // A file without an index can only be read front to back; next_raw() puts
  // it back into time order.
  impl_->indexed = has_message_index(impl_->reader);
  options.readOrder = impl_->indexed ? mcap::ReadMessageOptions::ReadOrder::LogTimeOrder
                                     : mcap::ReadMessageOptions::ReadOrder::FileOrder;
  // Problems (e.g. a file truncated by a crash mid-recording) end reading at
  // the last good message rather than failing the whole session.
  impl_->view.emplace(impl_->reader.readMessages([](const mcap::Status&) {}, options));
  impl_->it.emplace(impl_->view->begin());
}

SessionReader::~SessionReader() = default;

std::optional<std::pair<Timestamp, Timestamp>> SessionReader::time_range() const {
  // Reading in time order has already loaded the summary, if the file has one.
  const mcap::McapReader& reader = impl_->reader;
  if (const auto& stats = reader.statistics(); stats && stats->messageCount > 0) {
    return std::pair{static_cast<Timestamp>(stats->messageStartTime),
                     static_cast<Timestamp>(stats->messageEndTime)};
  }
  const auto& chunks = reader.chunkIndexes();
  if (chunks.empty()) {
    return std::nullopt;
  }
  mcap::Timestamp first = chunks.front().messageStartTime;
  mcap::Timestamp last = chunks.front().messageEndTime;
  for (const mcap::ChunkIndex& chunk : chunks) {
    first = std::min(first, chunk.messageStartTime);
    last = std::max(last, chunk.messageEndTime);
  }
  return std::pair{static_cast<Timestamp>(first), static_cast<Timestamp>(last)};
}

bool SessionReader::indexed() const { return impl_->indexed; }

std::optional<SessionMessage> SessionReader::next() {
  Impl& d = *impl_;
  for (;;) {
    std::optional<Decoded> m;
    if (d.pending) {
      m = std::move(d.pending);
      d.pending.reset();
    } else {
      m = d.next_raw();
    }
    if (!m) {
      return std::nullopt;
    }
    if (std::holds_alternative<Confidence>(*m)) {
      continue;  // Confidence without a matching depth image.
    }
    SessionMessage message = std::get<SessionMessage>(std::move(*m));
    if (auto* depth = std::get_if<DepthImage>(&message)) {
      // The writer puts confidence right after its depth image.
      d.pending = d.next_raw();
      if (d.pending) {
        if (auto* c = std::get_if<Confidence>(&*d.pending);
            c != nullptr && c->camera == depth->camera && c->timestamp == depth->frame.timestamp &&
            c->values.size() == depth->frame.depth.size()) {
          depth->frame.confidence = std::move(c->values);
          d.pending.reset();
        }
      }
    }
    return message;
  }
}

}  // namespace vg::io
