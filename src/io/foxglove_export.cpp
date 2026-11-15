#include "vg_core/io/foxglove_export.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <map>
#include <mcap/writer.hpp>
#include <stdexcept>
#include <string>
#include <variant>

#include "mcap_channels.hpp"
#include "messages.hpp"
#include "protobuf.hpp"
#include "vg_core/depth_filter.hpp"
#include "vg_core/io/session_reader.hpp"
#include "vg_core/semantics.hpp"

namespace vg::io {
namespace {

using detail::ProtoWriter;

constexpr const char* kWhat = "foxglove export";

ProtoWriter vector3(const Eigen::Vector3d& v) {
  ProtoWriter w;
  w.double_field(1, v.x());
  w.double_field(2, v.y());
  w.double_field(3, v.z());
  return w;
}

ProtoWriter quaternion(const Eigen::Quaterniond& q) {
  ProtoWriter w;
  w.double_field(1, q.x());
  w.double_field(2, q.y());
  w.double_field(3, q.z());
  w.double_field(4, q.w());
  return w;
}

// foxglove.Pose.
ProtoWriter pose(const Eigen::Isometry3d& p) {
  ProtoWriter w;
  w.message_field(1, vector3(p.translation()));
  w.message_field(2, quaternion(Eigen::Quaterniond(p.rotation())));
  return w;
}

// foxglove.Color, from 0-255 components and an opacity.
ProtoWriter color(const Rgba& c, double alpha) {
  ProtoWriter w;
  w.double_field(1, c[0] / 255.0);
  w.double_field(2, c[1] / 255.0);
  w.double_field(3, c[2] / 255.0);
  w.double_field(4, alpha);
  return w;
}

// foxglove.FrameTransform.
ProtoWriter frame_transform(Timestamp t, const std::string& parent, const std::string& child,
                            const Eigen::Isometry3d& parent_from_child) {
  ProtoWriter w;
  w.timestamp_field(1, t);
  w.bytes_field(2, parent);
  w.bytes_field(3, child);
  w.message_field(4, vector3(parent_from_child.translation()));
  w.message_field(5, quaternion(Eigen::Quaterniond(parent_from_child.rotation())));
  return w;
}

std::string frame_transforms(const ProtoWriter& one) {
  ProtoWriter w;
  w.message_field(1, one);
  return w.data();
}

// A foxglove.SceneEntity that stays until replaced, following its frame.
ProtoWriter entity(Timestamp t, const std::string& frame, const std::string& id) {
  ProtoWriter w;
  w.timestamp_field(1, t);
  w.bytes_field(2, frame);
  w.bytes_field(3, id);
  w.varint_field(5, 1);  // frame_locked
  return w;
}

std::string scene_update(const std::vector<ProtoWriter>& entities) {
  ProtoWriter w;
  for (const ProtoWriter& e : entities) {
    w.message_field(2, e);
  }
  return w.data();
}

// A mesh as a foxglove.TriangleListPrimitive.
ProtoWriter triangle_list(const MeshPart& part) {
  ProtoWriter w;
  w.message_field(1, pose(Eigen::Isometry3d::Identity()));
  for (const Eigen::Vector3f& p : part.mesh.positions) {
    w.message_field(2, vector3(p.cast<double>()));
  }
  for (const Rgba& c : part.mesh.colors) {
    w.message_field(4, color(c, part.opacity * c[3] / 255.0));
  }
  w.bytes_field(5, std::string_view(reinterpret_cast<const char*>(part.mesh.indices.data()),
                                    part.mesh.indices.size() * sizeof(std::uint32_t)));
  return w;
}

// The walk as a foxglove.LinePrimitive (a strip).
ProtoWriter line_strip(const std::vector<Eigen::Vector3d>& points, const Rgba& c) {
  ProtoWriter w;
  w.message_field(2, pose(Eigen::Isometry3d::Identity()));
  w.double_field(3, 3.0);  // Thickness in pixels...
  w.varint_field(4, 1);    // ... as scale_invariant.
  for (const Eigen::Vector3d& p : points) {
    w.message_field(5, vector3(p));
  }
  w.message_field(6, color(c, 1.0));
  return w;
}

// foxglove.PackedElementField.
ProtoWriter packed_field(const char* name, std::uint32_t offset, int type) {
  ProtoWriter w;
  w.bytes_field(1, name);
  w.fixed32_field(2, offset);
  w.varint_field(3, static_cast<std::uint64_t>(type));
  return w;
}

// One point of a cloud: x, y, z, red, green, blue, alpha, confidence, label.
struct CloudPoint {
  float x, y, z;
  std::uint8_t r, g, b, a;
  std::uint8_t confidence;
  std::uint8_t label;
  std::uint8_t pad[2];
};
static_assert(sizeof(CloudPoint) == 20);
constexpr int kUint8 = 1;
constexpr int kFloat32 = 7;

// Color of points without a label.
constexpr Rgba kUnlabeled{255, 0, 255, 255};

// A depth image as a foxglove.PointCloud in its camera's frame; points counts
// the points kept.
std::string point_cloud(const DepthImage& depth, const FoxgloveExportOptions& options,
                        std::size_t& points) {
  const DepthFrame& f = depth.frame;
  const CameraIntrinsics& k = f.intrinsics;
  const int stride = std::max(1, options.pixel_stride);
  std::string data;
  data.reserve(static_cast<std::size_t>(k.width / stride + 1) *
               static_cast<std::size_t>(k.height / stride + 1) * sizeof(CloudPoint));
  for (int v = 0; v < k.height; v += stride) {
    for (int u = 0; u < k.width; u += stride) {
      const std::size_t i = f.index(u, v);
      const float d = f.depth[i];
      if (!std::isfinite(d) || d <= 0.0f || d > options.max_depth) {
        continue;
      }
      const std::uint8_t confidence = f.confidence.empty() ? 255 : f.confidence[i];
      if (confidence < options.min_confidence ||
          (options.edge_ratio > 0.0 && is_depth_edge(f, u, v, options.edge_ratio))) {
        continue;
      }
      CloudPoint p{};
      p.x = static_cast<float>((u - k.cx) / k.fx * d);
      p.y = static_cast<float>((v - k.cy) / k.fy * d);
      p.z = d;
      Rgba c = kUnlabeled;
      if (!f.labels.empty()) {
        p.label = f.labels[i];
        const auto rgb =
            semantic_class_color(p.label < kSemanticClasses ? static_cast<SemanticClass>(p.label)
                                                            : SemanticClass::Unknown);
        c = {rgb[0], rgb[1], rgb[2], 255};
      }
      p.r = c[0];
      p.g = c[1];
      p.b = c[2];
      p.a = c[3];
      p.confidence = confidence;
      data.append(reinterpret_cast<const char*>(&p), sizeof(p));
    }
  }
  points += data.size() / sizeof(CloudPoint);

  ProtoWriter w;
  w.timestamp_field(1, f.timestamp);
  w.bytes_field(2, "cam/" + depth.camera);
  w.message_field(3, pose(Eigen::Isometry3d::Identity()));
  w.fixed32_field(4, sizeof(CloudPoint));
  w.message_field(5, packed_field("x", offsetof(CloudPoint, x), kFloat32));
  w.message_field(5, packed_field("y", offsetof(CloudPoint, y), kFloat32));
  w.message_field(5, packed_field("z", offsetof(CloudPoint, z), kFloat32));
  w.message_field(5, packed_field("red", offsetof(CloudPoint, r), kUint8));
  w.message_field(5, packed_field("green", offsetof(CloudPoint, g), kUint8));
  w.message_field(5, packed_field("blue", offsetof(CloudPoint, b), kUint8));
  w.message_field(5, packed_field("alpha", offsetof(CloudPoint, a), kUint8));
  w.message_field(5, packed_field("confidence", offsetof(CloudPoint, confidence), kUint8));
  w.message_field(5, packed_field("label", offsetof(CloudPoint, label), kUint8));
  w.bytes_field(6, data);
  return w.data();
}

Eigen::Isometry3d isometry(const MapTransform& t) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = Eigen::AngleAxisd(t.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p.translation() = t.translation;
  return p;
}

}  // namespace

FoxgloveExportSummary export_foxglove(const std::filesystem::path& session,
                                      const std::filesystem::path& out,
                                      const FoxgloveExportOptions& options, std::ostream& log) {
  SessionReader reader(session);
  const auto range = reader.time_range();

  mcap::McapWriter writer;
  mcap::McapWriterOptions writer_options("");
  writer_options.compression = mcap::Compression::Zstd;
  if (const mcap::Status status = writer.open(out.string(), writer_options); !status.ok()) {
    throw std::runtime_error(std::string(kWhat) + ": cannot open " + out.string() + ": " +
                             status.message);
  }
  detail::McapChannels channels(writer);
  const auto write = [&](const std::string& topic, const char* schema, Timestamp t,
                         const std::string& data) {
    channels.write(topic, schema, t, data, kWhat);
  };

  FoxgloveExportSummary summary;
  PoseJumpFilter jumps(options.pose_jumps);
  std::vector<Eigen::Vector3d> path;
  std::optional<Timestamp> start;
  std::map<std::string, Timestamp> last_image;
  const auto image_gap = options.image_rate > 0.0
                             ? static_cast<Timestamp>(1e9 / options.image_rate)
                             : Timestamp{0};
  while (const auto message = reader.next()) {
    if (std::holds_alternative<SessionInfo>(*message)) {
      continue;
    }
    // Nothing goes before the walk's start: a message stamped earlier (or 0)
    // would stretch Foxglove's timeline back to it, with nothing to show.
    const Timestamp t = start ? std::max(*start, timestamp_of(*message)) : timestamp_of(*message);
    if (!start) {
      // Things that hold for the whole walk, at its start.
      start = t;
      if (options.site_from_world) {
        write("/tf_static", "foxglove.FrameTransforms", t,
              frame_transforms(
                  frame_transform(t, "site", "world", isometry(*options.site_from_world))));
        std::vector<ProtoWriter> entities;
        for (const MeshPart& part : options.site_model) {
          if (!part.mesh.empty()) {
            ProtoWriter e = entity(t, "site", part.name);
            e.message_field(12, triangle_list(part));
            entities.push_back(e);
          }
        }
        if (!entities.empty()) {
          write("/site/model", "foxglove.SceneUpdate", t, scene_update(entities));
        }
      }
    }

    if (const auto* p = std::get_if<PoseStamped>(&*message)) {
      const PoseStamped corrected = options.remove_pose_jumps ? jumps.correct(*p) : *p;
      write("/tf", "foxglove.FrameTransforms", t,
            frame_transforms(frame_transform(t, "world", "body", corrected.world_from_frame)));
      path.push_back(corrected.world_from_frame.translation());
      ++summary.poses;
    } else if (const auto* tf = std::get_if<StaticTransforms>(&*message)) {
      write("/tf_static", "foxglove.FrameTransforms", t, detail::encode(*tf));
    } else if (const auto* calibration = std::get_if<CameraCalibration>(&*message)) {
      write("/cam/" + calibration->camera + "/calibration", "foxglove.CameraCalibration", t,
            detail::encode_calibration(t, "cam/" + calibration->camera, calibration->intrinsics));
    } else if (const auto* image = std::get_if<CameraImage>(&*message)) {
      const auto last = last_image.find(image->camera);
      if (options.images && (last == last_image.end() || t - last->second >= image_gap)) {
        last_image[image->camera] = t;
        write("/cam/" + image->camera + "/image", "foxglove.CompressedImage", t,
              detail::encode(*image));
        ++summary.images;
      }
    } else if (const auto* depth = std::get_if<DepthImage>(&*message)) {
      write("/cam/" + depth->camera + "/points", "foxglove.PointCloud", t,
            point_cloud(*depth, options, summary.points));
      if (++summary.clouds % 500 == 0) {
        log << "foxglove: " << summary.clouds << " depth frames";
        if (range && range->second > range->first) {
          log << " ("
              << static_cast<int>(100.0 * static_cast<double>(t - range->first) /
                                  static_cast<double>(range->second - range->first))
              << "%)";
        }
        log << std::endl;
      }
    }
  }

  // The whole walk as a line, from its start (written last: the path is only
  // known at the end; MCAP readers sort by time).
  if (start && path.size() > 1) {
    ProtoWriter e = entity(*start, "world", "walk");
    e.message_field(11, line_strip(path, {255, 210, 0, 255}));
    write("/walk/path", "foxglove.SceneUpdate", *start, scene_update({e}));
  }
  writer.close();
  summary.pose_jumps = jumps.jumps();
  return summary;
}

}  // namespace vg::io
