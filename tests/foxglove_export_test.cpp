#include "vg_core/io/foxglove_export.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <map>
#include <mcap/reader.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "../src/io/protobuf.hpp"
#include "vg_core/io/session_writer.hpp"

namespace {

constexpr vg::Timestamp kMs = 1'000'000;

class TempDir {
 public:
  TempDir()
      : path_(std::filesystem::temp_directory_path() /
              ("vg_foxglove_" +
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()))) {
    std::filesystem::create_directories(path_);
  }
  ~TempDir() {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

struct Message {
  std::string schema;
  vg::Timestamp time = 0;
  std::string data;
};

std::map<std::string, std::vector<Message>> read_topics(const std::filesystem::path& path) {
  mcap::McapReader reader;
  EXPECT_TRUE(reader.open(path.string()).ok());
  std::map<std::string, std::vector<Message>> topics;
  for (const auto& view : reader.readMessages()) {
    topics[view.channel->topic].push_back(
        {view.schema->name, static_cast<vg::Timestamp>(view.message.logTime),
         std::string(reinterpret_cast<const char*>(view.message.data), view.message.dataSize)});
  }
  reader.close();
  return topics;
}

// The fields of an encoded message, by number (last one wins).
std::map<std::uint32_t, vg::io::detail::ProtoField> fields(std::string_view data) {
  std::map<std::uint32_t, vg::io::detail::ProtoField> out;
  vg::io::detail::ProtoReader reader(data);
  vg::io::detail::ProtoField f;
  while (reader.next(f)) {
    out[f.number] = f;
  }
  return out;
}

// A three-pose walk with a 4x3 depth image (one pixel empty) at each pose.
void write_walk(const std::filesystem::path& path) {
  vg::io::SessionInfo info;
  info.producer = "vg_core tests";
  info.start_time = 1 * kMs;
  vg::io::SessionWriter writer(path, info);
  vg::io::StaticTransforms tf;
  tf.timestamp = 2 * kMs;
  vg::io::StaticTransform cam;
  cam.parent_frame = "body";
  cam.child_frame = "cam/rgb";
  tf.transforms.push_back(cam);
  writer.write(tf);
  vg::io::CameraCalibration calibration;
  calibration.timestamp = 2 * kMs;
  calibration.camera = "rgb";
  calibration.intrinsics = {4, 3, 2.0, 2.0, 1.5, 1.0};
  writer.write(calibration);
  for (int i = 0; i < 3; ++i) {
    const vg::Timestamp t = (100 + 100 * i) * kMs;
    vg::PoseStamped pose;
    pose.timestamp = t;
    pose.world_from_frame.translation() = Eigen::Vector3d(0.05 * i, 0.0, 1.2);
    writer.write(pose);
    vg::io::CameraImage image;
    image.timestamp = t;
    image.camera = "rgb";
    image.format = "jpeg";
    image.data = {1, 2, 3};
    writer.write(image);
    vg::io::DepthImage depth;
    depth.camera = "rgb";
    depth.frame.timestamp = t;
    depth.frame.intrinsics = {4, 3, 2.0, 2.0, 1.5, 1.0};
    depth.frame.depth.assign(12, 1.0f);
    depth.frame.depth[0] = 0.0f;
    depth.frame.labels.assign(12, 1);  // Ground.
    writer.write(depth);
  }
}

TEST(FoxgloveExport, WritesTransformsCloudsAndTheSiteModel) {
  TempDir dir;
  const auto session = dir.path() / "walk.vgcap.mcap";
  const auto out = dir.path() / "walk.foxglove.mcap";
  write_walk(session);

  vg::io::FoxgloveExportOptions options;
  options.pixel_stride = 1;
  options.edge_ratio = 0.0;
  vg::MapTransform site_from_world;
  site_from_world.yaw = 0.5;
  site_from_world.translation = {3.0, 4.0, 0.0};
  options.site_from_world = site_from_world;
  vg::MeshPart surface;
  surface.name = "surface";
  surface.mesh.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  surface.mesh.normals.assign(3, Eigen::Vector3f::UnitZ());
  surface.mesh.colors.assign(3, vg::Rgba{10, 20, 30, 255});
  surface.mesh.indices = {0, 1, 2};
  options.site_model = {surface};

  std::ostringstream log;
  const vg::io::FoxgloveExportSummary summary = vg::io::export_foxglove(session, out, options, log);
  EXPECT_EQ(summary.poses, 3u);
  EXPECT_EQ(summary.clouds, 3u);
  EXPECT_EQ(summary.points, 3u * 11u);
  EXPECT_EQ(summary.images, 3u);

  auto topics = read_topics(out);
  ASSERT_EQ(topics["/tf"].size(), 3u);
  EXPECT_EQ(topics["/tf"][0].schema, "foxglove.FrameTransforms");
  ASSERT_EQ(topics["/tf_static"].size(), 2u);  // site -> world, and the recording's.
  ASSERT_EQ(topics["/cam/rgb/points"].size(), 3u);
  EXPECT_EQ(topics["/cam/rgb/points"][0].schema, "foxglove.PointCloud");
  EXPECT_EQ(topics["/cam/rgb/image"].size(), 3u);
  ASSERT_EQ(topics["/site/model"].size(), 1u);
  EXPECT_EQ(topics["/site/model"][0].schema, "foxglove.SceneUpdate");
  ASSERT_EQ(topics["/cam/rgb/calibration"].size(), 1u);
  EXPECT_EQ(topics["/cam/rgb/calibration"][0].time, 2 * kMs);  // Its own time, not 0.
  ASSERT_EQ(topics["/walk/path"].size(), 1u);
  EXPECT_EQ(topics["/walk/path"][0].time, 2 * kMs);  // Shown from the walk's start.

  // world -> body at the second pose.
  const auto tf = fields(fields(topics["/tf"][1].data).at(1).bytes);
  EXPECT_EQ(tf.at(2).bytes, "world");
  EXPECT_EQ(tf.at(3).bytes, "body");
  EXPECT_DOUBLE_EQ(fields(tf.at(4).bytes).at(1).as_double(), 0.05);

  // site -> world comes first.
  const auto site = fields(fields(topics["/tf_static"][0].data).at(1).bytes);
  EXPECT_EQ(site.at(2).bytes, "site");
  EXPECT_EQ(site.at(3).bytes, "world");

  // The cloud: in the camera frame, 20-byte points, the empty pixel left out,
  // colored as ground.
  const auto cloud = fields(topics["/cam/rgb/points"][0].data);
  EXPECT_EQ(cloud.at(2).bytes, "cam/rgb");
  EXPECT_EQ(cloud.at(4).as_uint32(), 20u);
  const std::string_view points = cloud.at(6).bytes;
  ASSERT_EQ(points.size(), 11u * 20u);
  float xyz[3];
  std::memcpy(xyz, points.data(), sizeof(xyz));  // Pixel (1, 0).
  EXPECT_FLOAT_EQ(xyz[0], (1.0f - 1.5f) / 2.0f);
  EXPECT_FLOAT_EQ(xyz[1], (0.0f - 1.0f) / 2.0f);
  EXPECT_FLOAT_EQ(xyz[2], 1.0f);
  EXPECT_EQ(static_cast<std::uint8_t>(points[12]), 150);  // Ground's red.
  EXPECT_EQ(static_cast<std::uint8_t>(points[17]), 1);    // Its label.
}

TEST(FoxgloveExport, WithoutASiteHasNoModel) {
  TempDir dir;
  const auto session = dir.path() / "walk.vgcap.mcap";
  const auto out = dir.path() / "walk.foxglove.mcap";
  write_walk(session);
  vg::io::FoxgloveExportOptions options;
  options.images = false;
  std::ostringstream log;
  vg::io::export_foxglove(session, out, options, log);
  auto topics = read_topics(out);
  EXPECT_EQ(topics.count("/site/model"), 0u);
  EXPECT_EQ(topics.count("/cam/rgb/image"), 0u);
  EXPECT_EQ(topics["/tf_static"].size(), 1u);
  EXPECT_EQ(topics["/tf"].size(), 3u);
}

}  // namespace
