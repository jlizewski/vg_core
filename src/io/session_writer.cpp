#include "vg_core/io/session_writer.hpp"

#include <map>
#include <mcap/writer.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "messages.hpp"
#include "schema_descriptors.hpp"

namespace vg::io {

struct SessionWriter::Impl {
  mcap::McapWriter writer;
  bool open = false;
  std::unordered_map<std::string, mcap::SchemaId> schemas;
  std::unordered_map<std::string, mcap::ChannelId> channels;
  // Last depth calibration written per camera, to write it only on change.
  std::map<std::string, CameraIntrinsics> depth_calibrations;

  mcap::ChannelId channel(const std::string& topic, const std::string& schema_name) {
    if (auto it = channels.find(topic); it != channels.end()) {
      return it->second;
    }
    auto schema_it = schemas.find(schema_name);
    if (schema_it == schemas.end()) {
      const std::string_view descriptor = detail::schema_descriptor(schema_name);
      if (descriptor.empty()) {
        throw std::logic_error("session: no descriptor for " + schema_name);
      }
      mcap::Schema schema(schema_name, "protobuf", descriptor);
      writer.addSchema(schema);
      schema_it = schemas.emplace(schema_name, schema.id).first;
    }
    mcap::Channel ch(topic, "protobuf", schema_it->second);
    writer.addChannel(ch);
    return channels.emplace(topic, ch.id).first->second;
  }

  void write(const std::string& topic, const std::string& schema_name, Timestamp timestamp,
             const std::string& data) {
    if (!open) {
      throw std::logic_error("session: write after close");
    }
    if (timestamp < 0) {
      throw std::invalid_argument("session: negative timestamp on " + topic);
    }
    mcap::Message message;
    message.channelId = channel(topic, schema_name);
    message.sequence = 0;
    message.logTime = static_cast<mcap::Timestamp>(timestamp);
    message.publishTime = message.logTime;
    message.data = reinterpret_cast<const std::byte*>(data.data());
    message.dataSize = data.size();
    const mcap::Status status = writer.write(message);
    if (!status.ok()) {
      throw std::runtime_error("session: writing " + topic + ": " + status.message);
    }
  }
};

namespace {

bool same(const CameraIntrinsics& a, const CameraIntrinsics& b) {
  return a.width == b.width && a.height == b.height && a.fx == b.fx && a.fy == b.fy &&
         a.cx == b.cx && a.cy == b.cy;
}

std::string camera_topic(const std::string& camera, const char* suffix) {
  return "/cam/" + camera + suffix;
}

}  // namespace

SessionWriter::SessionWriter(const std::filesystem::path& path, const SessionInfo& info)
    : impl_(std::make_unique<Impl>()) {
  mcap::McapWriterOptions options("");
  options.compression = mcap::Compression::Zstd;
  const mcap::Status status = impl_->writer.open(path.string(), options);
  if (!status.ok()) {
    throw std::runtime_error("session: cannot open " + path.string() + ": " + status.message);
  }
  impl_->open = true;

  mcap::Metadata metadata;
  metadata.name = "vg_capture";
  metadata.metadata = {{"format_version", info.format_version}, {"producer", info.producer}};
  if (const mcap::Status meta_status = impl_->writer.write(metadata); !meta_status.ok()) {
    throw std::runtime_error("session: writing metadata: " + meta_status.message);
  }
  impl_->write("/vg/session", "vg.SessionInfo", info.start_time, detail::encode(info));
}

SessionWriter::~SessionWriter() {
  if (impl_) {
    close();
  }
}

void SessionWriter::close() {
  if (impl_->open) {
    impl_->writer.close();
    impl_->open = false;
  }
}

void SessionWriter::write(const CameraCalibration& c) {
  impl_->write(camera_topic(c.camera, "/calibration"), "foxglove.CameraCalibration", c.timestamp,
               detail::encode_calibration(c.timestamp, "cam/" + c.camera, c.intrinsics));
}

void SessionWriter::write(const StaticTransforms& t) {
  impl_->write("/tf_static", "foxglove.FrameTransforms", t.timestamp, detail::encode(t));
}

void SessionWriter::write(const CameraImage& image) {
  impl_->write(camera_topic(image.camera, "/image"), "foxglove.CompressedImage", image.timestamp,
               detail::encode(image));
}

void SessionWriter::write(const DepthImage& depth) {
  const DepthFrame& frame = depth.frame;
  const std::size_t pixels = static_cast<std::size_t>(frame.intrinsics.width) *
                             static_cast<std::size_t>(frame.intrinsics.height);
  if (frame.depth.size() != pixels ||
      (!frame.confidence.empty() && frame.confidence.size() != pixels)) {
    throw std::invalid_argument("session: depth or confidence size doesn't match intrinsics");
  }

  auto last = impl_->depth_calibrations.find(depth.camera);
  if (last == impl_->depth_calibrations.end() || !same(last->second, frame.intrinsics)) {
    impl_->write(
        camera_topic(depth.camera, "/depth/calibration"), "foxglove.CameraCalibration",
        frame.timestamp,
        detail::encode_calibration(frame.timestamp, "cam/" + depth.camera, frame.intrinsics));
    impl_->depth_calibrations[depth.camera] = frame.intrinsics;
  }
  impl_->write(camera_topic(depth.camera, "/depth"), "foxglove.RawImage", frame.timestamp,
               detail::encode_depth(depth));
  if (!frame.confidence.empty()) {
    impl_->write(camera_topic(depth.camera, "/depth/confidence"), "foxglove.RawImage",
                 frame.timestamp, detail::encode_confidence(depth));
  }
}

void SessionWriter::write(const PoseStamped& pose) {
  impl_->write("/pose", "foxglove.PoseInFrame", pose.timestamp, detail::encode(pose));
}

void SessionWriter::write(const TrackingStatus& status) {
  impl_->write("/pose/status", "vg.TrackingStatus", status.timestamp, detail::encode(status));
}

void SessionWriter::write(const ImuSample& imu) {
  impl_->write("/imu", "vg.Imu", imu.timestamp, detail::encode(imu));
}

void SessionWriter::write(const GpsFix& fix) {
  impl_->write("/gps", "foxglove.LocationFix", fix.timestamp, detail::encode(fix));
}

void SessionWriter::write(const SessionMessage& message) {
  std::visit(
      [this](const auto& m) {
        using T = std::decay_t<decltype(m)>;
        if constexpr (std::is_same_v<T, SessionInfo>) {
          throw std::invalid_argument("session: SessionInfo is written by the constructor");
        } else {
          write(m);
        }
      },
      message);
}

}  // namespace vg::io
