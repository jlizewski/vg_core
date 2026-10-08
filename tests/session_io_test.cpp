#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <future>
#include <string>
#include <vector>

#include "test_scene.hpp"
#include "vg_core/io/session_player.hpp"
#include "vg_core/io/session_reader.hpp"
#include "vg_core/io/session_writer.hpp"

namespace {

using vg::io::SessionMessage;
using vg::test::look_at;
using vg::test::render_depth;

constexpr vg::Timestamp kMs = 1'000'000;

// Unique temp file, removed when the test ends.
class TempFile {
 public:
  explicit TempFile(const std::string& name)
      : path_(std::filesystem::temp_directory_path() /
              (name + "_" +
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
               ".mcap")) {}
  ~TempFile() {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

vg::io::SessionInfo test_info() {
  vg::io::SessionInfo info;
  info.producer = "vg_core tests";
  info.platform = "synthetic";
  info.device_model = "none";
  info.start_time = 5 * kMs;
  info.utc_offset_ns = 1'700'000'000'000'000'000;
  info.notes = "raised bed";
  return info;
}

std::vector<SessionMessage> read_all(const std::filesystem::path& path) {
  vg::io::SessionReader reader(path);
  std::vector<SessionMessage> out;
  while (auto m = reader.next()) {
    out.push_back(std::move(*m));
  }
  return out;
}

void expect_isometry_near(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  EXPECT_TRUE(a.isApprox(b, 1e-9)) << a.matrix() << "\nvs\n" << b.matrix();
}

TEST(SessionIo, RoundTripsEveryMessageType) {
  TempFile file("vg_session_roundtrip");

  vg::io::CameraCalibration calibration;
  calibration.timestamp = 10 * kMs;
  calibration.camera = "rgb";
  calibration.intrinsics = {640, 480, 500.0, 501.0, 319.5, 239.5};

  vg::io::StaticTransforms transforms;
  transforms.timestamp = 10 * kMs;
  vg::io::StaticTransform t;
  t.parent_frame = "body";
  t.child_frame = "cam/rgb";
  t.parent_from_child.translate(Eigen::Vector3d(0.01, -0.02, 0.03));
  t.parent_from_child.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized()));
  transforms.transforms.push_back(t);

  vg::io::CameraImage image;
  image.timestamp = 20 * kMs;
  image.camera = "rgb";
  image.format = "jpeg";
  image.data = {0xff, 0xd8, 0x00, 0x01, 0xff, 0xd9};

  vg::io::DepthImage depth;
  depth.camera = "rgb";
  depth.frame.timestamp = 20 * kMs;
  depth.frame.intrinsics = {4, 3, 3.0, 3.0, 1.5, 1.0};
  for (int i = 0; i < 12; ++i) {
    depth.frame.depth.push_back(0.5f + 0.1f * static_cast<float>(i));
    depth.frame.confidence.push_back(static_cast<std::uint8_t>(i * 20));
  }

  vg::PoseStamped pose;
  pose.timestamp = 20 * kMs;
  pose.world_from_frame.translate(Eigen::Vector3d(1.0, 2.0, 1.5));
  pose.world_from_frame.rotate(Eigen::AngleAxisd(-1.2, Eigen::Vector3d::UnitZ()));

  vg::io::TrackingStatus status;
  status.timestamp = 25 * kMs;
  status.state = vg::io::TrackingState::Limited;
  status.reason = "excessive_motion";

  vg::ImuSample imu;
  imu.timestamp = 30 * kMs;
  imu.angular_velocity = {0.1, -0.2, 0.3};
  imu.linear_acceleration = {0.0, 0.1, 9.81};

  vg::GpsFix fix;
  fix.timestamp = 40 * kMs;
  fix.latitude = 45.5231;
  fix.longitude = -122.6765;
  fix.altitude = 15.25;
  fix.position_covariance.diagonal() << 4.0, 4.0, 9.0;

  {
    vg::io::SessionWriter writer(file.path(), test_info());
    writer.write(calibration);
    writer.write(transforms);
    writer.write(image);
    writer.write(pose);
    writer.write(depth);
    writer.write(status);
    writer.write(imu);
    writer.write(SessionMessage{fix});
  }

  const auto messages = read_all(file.path());
  ASSERT_EQ(messages.size(), 9u);

  const auto& info = std::get<vg::io::SessionInfo>(messages[0]);
  EXPECT_EQ(info.format_version, "0.1");
  EXPECT_EQ(info.producer, "vg_core tests");
  EXPECT_EQ(info.platform, "synthetic");
  EXPECT_EQ(info.start_time, 5 * kMs);
  EXPECT_EQ(info.utc_offset_ns, 1'700'000'000'000'000'000);
  EXPECT_EQ(info.notes, "raised bed");

  const auto& cal = std::get<vg::io::CameraCalibration>(messages[1]);
  EXPECT_EQ(cal.camera, "rgb");
  EXPECT_EQ(cal.intrinsics.width, 640);
  EXPECT_DOUBLE_EQ(cal.intrinsics.fy, 501.0);
  EXPECT_DOUBLE_EQ(cal.intrinsics.cx, 319.5);

  const auto& tf = std::get<vg::io::StaticTransforms>(messages[2]);
  ASSERT_EQ(tf.transforms.size(), 1u);
  EXPECT_EQ(tf.transforms[0].child_frame, "cam/rgb");
  expect_isometry_near(tf.transforms[0].parent_from_child, t.parent_from_child);

  // Messages with the same time come back in the order written.
  const auto& img = std::get<vg::io::CameraImage>(messages[3]);
  EXPECT_EQ(img.format, "jpeg");
  EXPECT_EQ(img.data, image.data);

  const auto& p = std::get<vg::PoseStamped>(messages[4]);
  EXPECT_EQ(p.timestamp, 20 * kMs);
  expect_isometry_near(p.world_from_frame, pose.world_from_frame);

  const auto& d = std::get<vg::io::DepthImage>(messages[5]);
  EXPECT_EQ(d.camera, "rgb");
  EXPECT_EQ(d.frame.intrinsics.width, 4);
  EXPECT_DOUBLE_EQ(d.frame.intrinsics.cx, 1.5);
  EXPECT_EQ(d.frame.depth, depth.frame.depth);
  EXPECT_EQ(d.frame.confidence, depth.frame.confidence);

  const auto& s = std::get<vg::io::TrackingStatus>(messages[6]);
  EXPECT_EQ(s.state, vg::io::TrackingState::Limited);
  EXPECT_EQ(s.reason, "excessive_motion");

  const auto& i = std::get<vg::ImuSample>(messages[7]);
  EXPECT_EQ(i.linear_acceleration, imu.linear_acceleration);
  EXPECT_EQ(i.angular_velocity, imu.angular_velocity);

  const auto& g = std::get<vg::GpsFix>(messages[8]);
  EXPECT_DOUBLE_EQ(g.latitude, fix.latitude);
  EXPECT_DOUBLE_EQ(g.longitude, fix.longitude);
  EXPECT_DOUBLE_EQ(g.altitude, fix.altitude);
  EXPECT_EQ(g.position_covariance, fix.position_covariance);
}

TEST(SessionIo, ReadsDepthWithoutConfidence) {
  TempFile file("vg_session_depth");
  vg::io::DepthImage depth;
  depth.camera = "rgb";
  depth.frame.timestamp = 7 * kMs;
  depth.frame.intrinsics = {2, 1, 1.0, 1.0, 0.5, 0.0};
  depth.frame.depth = {1.0f, 2.0f};
  {
    vg::io::SessionWriter writer(file.path(), test_info());
    writer.write(depth);
    vg::ImuSample imu;
    imu.timestamp = 7 * kMs;
    writer.write(imu);
  }
  const auto messages = read_all(file.path());
  ASSERT_EQ(messages.size(), 3u);
  const auto& d = std::get<vg::io::DepthImage>(messages[1]);
  EXPECT_TRUE(d.frame.confidence.empty());
  EXPECT_EQ(d.frame.depth, depth.frame.depth);
  EXPECT_TRUE(std::holds_alternative<vg::ImuSample>(messages[2]));
}

TEST(SessionIo, RejectsMismatchedDepthSize) {
  TempFile file("vg_session_bad");
  vg::io::SessionWriter writer(file.path(), test_info());
  vg::io::DepthImage depth;
  depth.camera = "rgb";
  depth.frame.intrinsics = {2, 2, 1.0, 1.0, 0.5, 0.5};
  depth.frame.depth = {1.0f};
  EXPECT_THROW(writer.write(depth), std::invalid_argument);
}

TEST(SessionIo, ThrowsOnMissingFile) {
  EXPECT_THROW(vg::io::SessionReader("does/not/exist.mcap"), std::runtime_error);
}

TEST(SessionPlayer, PacesToRecordedTiming) {
  TempFile file("vg_session_timing");
  {
    vg::io::SessionWriter writer(file.path(), test_info());
    for (int i = 0; i <= 4; ++i) {
      vg::ImuSample imu;
      imu.timestamp = 5 * kMs + i * 25 * kMs;  // 100 ms of data after the session info.
      writer.write(imu);
    }
  }
  auto elapsed = [&file](double rate) {
    vg::io::SessionReader reader(file.path());
    vg::io::PlaybackOptions options;
    options.rate = rate;
    const auto start = std::chrono::steady_clock::now();
    const auto count = vg::io::play(reader, [](const SessionMessage&) { return true; }, options);
    EXPECT_EQ(count, 6u);
    return std::chrono::steady_clock::now() - start;
  };
  EXPECT_GE(elapsed(1.0), std::chrono::milliseconds(100));
  EXPECT_GE(elapsed(4.0), std::chrono::milliseconds(25));
  EXPECT_LT(elapsed(0.0), std::chrono::milliseconds(100));
}

TEST(SessionPlayer, StopsWhenCallbackReturnsFalse) {
  TempFile file("vg_session_stop");
  {
    vg::io::SessionWriter writer(file.path(), test_info());
    for (int i = 0; i < 10; ++i) {
      vg::ImuSample imu;
      imu.timestamp = 10 * kMs + i * kMs;
      writer.write(imu);
    }
  }
  vg::io::SessionReader reader(file.path());
  vg::io::PlaybackOptions options;
  options.rate = 0.0;
  std::size_t seen = 0;
  const auto count =
      vg::io::play(reader, [&seen](const SessionMessage&) { return ++seen < 3; }, options);
  EXPECT_EQ(count, 3u);
}

// IMU samples every 25 ms from 5 ms to 105 ms, after the session info at 5 ms.
void write_imu_session(const std::filesystem::path& path) {
  vg::io::SessionWriter writer(path, test_info());
  for (int i = 0; i <= 4; ++i) {
    vg::ImuSample imu;
    imu.timestamp = 5 * kMs + i * 25 * kMs;
    writer.write(imu);
  }
}

TEST(SessionIo, ReportsTimeRange) {
  TempFile file("vg_session_range");
  write_imu_session(file.path());
  vg::io::SessionReader reader(file.path());
  const auto range = reader.time_range();
  ASSERT_TRUE(range.has_value());
  EXPECT_EQ(range->first, 5 * kMs);
  EXPECT_EQ(range->second, 105 * kMs);
}

TEST(SessionPlayer, PlaysLiveAndReportsPosition) {
  TempFile file("vg_session_live");
  write_imu_session(file.path());
  vg::io::SessionPlayer player(file.path());
  EXPECT_EQ(player.duration(), 100 * kMs);
  const auto start = std::chrono::steady_clock::now();
  std::size_t count = 0;
  while (player.next()) {
    ++count;
  }
  EXPECT_EQ(count, 6u);
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
  EXPECT_EQ(player.position(), 100 * kMs);
}

TEST(SessionPlayer, ChangesSpeedWhilePlaying) {
  TempFile file("vg_session_speed");
  write_imu_session(file.path());
  vg::io::SessionPlayer player(file.path());
  const auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(player.next());
  player.set_rate(10.0);
  while (player.next()) {
  }
  // 100 ms of session at 10x takes 10 ms; at 1x it would take 100 ms.
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(80));
  EXPECT_DOUBLE_EQ(player.rate(), 10.0);
}

TEST(SessionPlayer, WaitsWhilePausedAndStops) {
  TempFile file("vg_session_pause");
  write_imu_session(file.path());
  vg::io::SessionPlayer player(file.path(), 0.0);
  ASSERT_TRUE(player.next());
  player.set_paused(true);
  EXPECT_TRUE(player.paused());

  auto waiting = std::async(std::launch::async, [&player] { return player.next().has_value(); });
  EXPECT_EQ(waiting.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  player.set_paused(false);
  ASSERT_EQ(waiting.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_TRUE(waiting.get());

  player.set_paused(true);
  auto stopped = std::async(std::launch::async, [&player] { return player.next().has_value(); });
  EXPECT_EQ(stopped.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  player.stop();
  ASSERT_EQ(stopped.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_FALSE(stopped.get());
  EXPECT_FALSE(player.next());
}

TEST(SessionMapper, ReplayedSessionBuildsHeightMap) {
  TempFile file("vg_session_replay");
  vg::test::Scene scene;
  scene.box = vg::test::Box{{0.2, -0.3, 0.0}, {0.8, 0.3, 0.3}};

  // Camera mounted on the body with a small offset, as on a phone.
  Eigen::Isometry3d body_from_camera = Eigen::Isometry3d::Identity();
  body_from_camera.translation() = Eigen::Vector3d(0.05, 0.0, 0.0);

  {
    vg::io::SessionWriter writer(file.path(), test_info());
    vg::io::StaticTransforms tf;
    tf.timestamp = 5 * kMs;
    tf.transforms.push_back({"body", "cam/rgb", body_from_camera});
    writer.write(tf);

    const Eigen::Vector3d target(0.3, 0.0, 0.0);
    vg::Timestamp t = 100 * kMs;
    int frame = 0;
    for (const Eigen::Vector3d& eye :
         {Eigen::Vector3d(0.0, 0.0, 1.5), Eigen::Vector3d(-1.0, 0.5, 1.4),
          Eigen::Vector3d(1.5, -0.5, 1.2)}) {
      const Eigen::Isometry3d world_from_camera = look_at(eye, target, Eigen::Vector3d::UnitZ());
      vg::PoseStamped pose;
      pose.timestamp = t;
      pose.world_from_frame = world_from_camera * body_from_camera.inverse();
      vg::io::DepthImage depth;
      depth.camera = "rgb";
      depth.frame = render_depth(scene, world_from_camera);
      depth.frame.timestamp = t;
      // Producers may write the pose before or after its depth frame.
      if (frame++ % 2 == 0) {
        writer.write(pose);
        writer.write(depth);
      } else {
        writer.write(depth);
        writer.write(pose);
      }
      t += 33 * kMs;
    }

    // A frame with no recent pose is skipped.
    vg::io::DepthImage stale;
    stale.camera = "rgb";
    stale.frame = render_depth(scene, look_at({0, 0, 1.5}, target, Eigen::Vector3d::UnitY()));
    stale.frame.timestamp = t + 500 * kMs;
    writer.write(stale);
  }

  vg::MapBuilderConfig config;
  config.tsdf.max_depth = 2.5;
  vg::MapBuilder builder(config);
  vg::io::SessionMapper mapper(builder);
  vg::io::SessionReader reader(file.path());
  vg::io::PlaybackOptions options;
  options.rate = 0.0;
  vg::io::play(
      reader,
      [&mapper](const SessionMessage& m) {
        mapper.handle(m);
        return true;
      },
      options);
  builder.flush();

  EXPECT_EQ(builder.stats().integrated, 3u);
  EXPECT_EQ(builder.stats().skipped_no_pose, 1u);

  builder.update();
  const auto map = builder.height_map();
  auto height_at = [&map](double x, double y) {
    const int cx = static_cast<int>(std::floor((x - map.origin.x()) / map.cell_size));
    const int cy = static_cast<int>(std::floor((y - map.origin.y()) / map.cell_size));
    return map.at(cx, cy);
  };
  EXPECT_NEAR(height_at(0.5, 0.0), 0.3, 0.03);
  EXPECT_NEAR(height_at(-0.4, 0.0), 0.0, 0.03);
}

}  // namespace
