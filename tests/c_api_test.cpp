#include "vg_core_c/vg_core_c.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <variant>
#include <vector>

#include "test_scene.hpp"
#include "vg_core/io/session_reader.hpp"
#include "vg_core/version.hpp"

extern "C" int vg_c_header_check(void);

namespace {

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kSecond = 1'000 * kMs;

// A directory removed when the test ends.
class TempDir {
 public:
  explicit TempDir(const std::string& name)
      : path_(std::filesystem::temp_directory_path() /
              (name + "_" +
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

std::string utf8(const std::filesystem::path& p) { return p.u8string(); }

vg_pose to_c(const Eigen::Isometry3d& t) {
  const Eigen::Quaterniond q(t.rotation());
  return vg_pose{{q.x(), q.y(), q.z(), q.w()},
                 {t.translation().x(), t.translation().y(), t.translation().z()}};
}

vg_intrinsics to_c(const vg::CameraIntrinsics& k) {
  return vg_intrinsics{k.width, k.height, k.fx, k.fy, k.cx, k.cy};
}

// A raised bed 0.6 m square and 0.3 m tall, centered at (0.5, 0).
vg::test::Scene bed() {
  vg::test::Scene scene;
  scene.box = vg::test::Box{{0.2, -0.3, 0.0}, {0.8, 0.3, 0.3}};
  return scene;
}

// Camera poses on a circle around the bed, looking at it, one a second (a
// slow walk, so the pose jump filter leaves it alone).
std::vector<Eigen::Isometry3d> walk(int n) {
  std::vector<Eigen::Isometry3d> poses;
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * 3.14159265358979323846 * i / n;
    const Eigen::Vector3d eye(0.5 + 1.8 * std::cos(a), 1.8 * std::sin(a), 1.4);
    poses.push_back(
        vg::test::look_at(eye, Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::UnitZ()));
  }
  return poses;
}

vg_depth_frame depth_frame(std::int64_t t, const vg::DepthFrame& rendered) {
  vg_depth_frame frame{};
  frame.timestamp_ns = t;
  frame.intrinsics = to_c(rendered.intrinsics);
  frame.depth_m = rendered.depth.data();
  return frame;
}

// The value of grid at world point (x, y), NaN outside it.
float grid_at(const vg_grid* grid, double x, double y) {
  vg_grid_info info{};
  if (vg_grid_get_info(grid, &info) != VG_OK) {
    return NAN;
  }
  const int cx = static_cast<int>(std::floor((x - info.origin_x) / info.cell_size));
  const int cy = static_cast<int>(std::floor((y - info.origin_y) / info.cell_size));
  if (cx < 0 || cy < 0 || cx >= info.width || cy >= info.height) {
    return NAN;
  }
  return vg_grid_values(grid)[static_cast<std::size_t>(cy) * static_cast<std::size_t>(info.width) +
                              static_cast<std::size_t>(cx)];
}

// Records the walk around the bed: a pose and a depth frame each second.
void record_walk(const std::filesystem::path& file, int frames) {
  vg_session_info info{};
  info.producer = "vg_core tests";
  info.platform = "synthetic";
  vg_recorder* recorder = nullptr;
  ASSERT_EQ(vg_recorder_open(utf8(file).c_str(), &info, &recorder), VG_OK) << vg_last_error();
  const vg_intrinsics k = to_c(vg::test::test_intrinsics());
  ASSERT_EQ(vg_recorder_write_calibration(recorder, 0, "rgb", &k), VG_OK) << vg_last_error();
  const auto poses = walk(frames);
  for (int i = 0; i < frames; ++i) {
    const std::int64_t t = (i + 1) * kSecond;
    const vg_pose pose = to_c(poses[static_cast<std::size_t>(i)]);
    ASSERT_EQ(vg_recorder_write_pose(recorder, t, &pose), VG_OK) << vg_last_error();
    const vg::DepthFrame rendered =
        vg::test::render_depth(bed(), poses[static_cast<std::size_t>(i)]);
    const vg_depth_frame frame = depth_frame(t, rendered);
    ASSERT_EQ(vg_recorder_write_depth(recorder, "rgb", &frame), VG_OK) << vg_last_error();
  }
  ASSERT_EQ(vg_recorder_close(recorder), VG_OK) << vg_last_error();
  vg_recorder_free(recorder);
}

void collect(void* user_data, const char* line) {
  static_cast<std::vector<std::string>*>(user_data)->emplace_back(line);
}

bool any_line_has(const std::vector<std::string>& lines, const std::string& text) {
  for (const std::string& line : lines) {
    if (line.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(CApi, ReportsItsVersion) {
  EXPECT_EQ(vg_api_version(), VG_C_API_VERSION);
  EXPECT_EQ(std::string(vg_version_string()), std::string(vg::version_string()));
  EXPECT_EQ(vg_c_header_check(), 1);
}

TEST(CApi, ReportsErrorsWithAMessage) {
  vg_grid* grid = nullptr;
  EXPECT_EQ(vg_grid_load("no/such/grid.asc", &grid), VG_ERROR_FAILED);
  EXPECT_EQ(grid, nullptr);
  EXPECT_STRNE(vg_last_error(), "");

  EXPECT_EQ(vg_grid_load(nullptr, &grid), VG_ERROR_INVALID_ARGUMENT);
  EXPECT_NE(std::string(vg_last_error()).find("path"), std::string::npos);

  // The next call that succeeds clears it.
  vg_map_builder_config config;
  EXPECT_EQ(vg_map_builder_config_defaults(&config), VG_OK);
  EXPECT_STREQ(vg_last_error(), "");

  // Freeing NULL is fine.
  vg_grid_free(nullptr);
  vg_recorder_free(nullptr);
  vg_map_builder_free(nullptr);
  vg_package_free(nullptr);
  vg_site_free(nullptr);
}

TEST(CApi, BadSettingsAreConfigErrors) {
  const char* settings[] = {"--set", "ground.no_such_setting=1"};
  EXPECT_EQ(vg_replay("unused.mcap", "unused", settings, 2, nullptr, nullptr, nullptr),
            VG_ERROR_CONFIG);
  EXPECT_NE(std::string(vg_last_error()).find("no_such_setting"), std::string::npos);
}

TEST(CApi, RecordsASessionTheReaderReadsBack) {
  TempDir dir("vg_c_api_recorder");
  const auto file = dir.path() / "walk.vgcap.mcap";
  vg_session_info info{};
  info.producer = "vg_core tests";
  info.platform = "synthetic";
  info.start_time_ns = 5 * kMs;
  info.utc_offset_ns = 1'700'000'000'000'000'000;
  vg_recorder* recorder = nullptr;
  ASSERT_EQ(vg_recorder_open(utf8(file).c_str(), &info, &recorder), VG_OK) << vg_last_error();

  const vg_intrinsics k = to_c(vg::test::test_intrinsics());
  EXPECT_EQ(vg_recorder_write_calibration(recorder, 6 * kMs, "rgb", &k), VG_OK);

  Eigen::Isometry3d body_from_camera(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  body_from_camera.translation() = Eigen::Vector3d(0.01, 0.02, 0.03);
  const vg_static_transform tf{"body", "cam/rgb", to_c(body_from_camera)};
  EXPECT_EQ(vg_recorder_write_static_transforms(recorder, 6 * kMs, &tf, 1), VG_OK);

  const Eigen::Isometry3d world_from_body = walk(8)[1];
  const vg_pose pose = to_c(world_from_body);
  EXPECT_EQ(vg_recorder_write_pose(recorder, 10 * kMs, &pose), VG_OK);

  // Depth in millimeters, as ARCore gives it, with confidence and labels.
  const vg::DepthFrame rendered = vg::test::render_depth(bed(), world_from_body);
  const std::size_t n = rendered.depth.size();
  std::vector<std::uint16_t> mm(n);
  for (std::size_t i = 0; i < n; ++i) {
    mm[i] = static_cast<std::uint16_t>(std::lround(rendered.depth[i] * 1000.0f));
  }
  const std::vector<std::uint8_t> confidence(n, 200);
  const std::vector<std::uint8_t> labels(n, VG_CLASS_GROUND);
  vg_depth_frame frame{};
  frame.timestamp_ns = 11 * kMs;
  frame.intrinsics = k;
  frame.depth_mm = mm.data();
  frame.confidence = confidence.data();
  frame.labels = labels.data();
  EXPECT_EQ(vg_recorder_write_depth(recorder, "rgb", &frame), VG_OK) << vg_last_error();
  frame.depth_m = rendered.depth.data();  // Both: refused.
  EXPECT_EQ(vg_recorder_write_depth(recorder, "rgb", &frame), VG_ERROR_INVALID_ARGUMENT);

  EXPECT_EQ(vg_recorder_write_tracking(recorder, 12 * kMs, VG_TRACKING_LIMITED, "fast motion"),
            VG_OK);
  EXPECT_EQ(vg_recorder_write_tracking(recorder, 12 * kMs, 9, nullptr),
            VG_ERROR_INVALID_ARGUMENT);
  const vg_imu_sample imu{13 * kMs, {0.1, 0.2, 0.3}, {0.0, 0.0, 9.81}};
  EXPECT_EQ(vg_recorder_write_imu(recorder, &imu), VG_OK);
  vg_gps_fix fix{};
  fix.timestamp_ns = 14 * kMs;
  fix.latitude = 41.88;
  fix.longitude = -87.63;
  fix.altitude = 180.0;
  fix.position_covariance[0] = 4.0;
  fix.position_covariance[4] = 4.0;
  fix.position_covariance[8] = 9.0;
  EXPECT_EQ(vg_recorder_write_gps(recorder, &fix), VG_OK);

  EXPECT_EQ(vg_recorder_close(recorder), VG_OK);
  EXPECT_EQ(vg_recorder_write_pose(recorder, 15 * kMs, &pose), VG_ERROR_FAILED);
  vg_recorder_free(recorder);

  vg::io::SessionReader reader(file);
  int seen = 0;
  while (const auto message = reader.next()) {
    if (const auto* s = std::get_if<vg::io::SessionInfo>(&*message)) {
      EXPECT_EQ(s->producer, "vg_core tests");
      EXPECT_EQ(s->utc_offset_ns, info.utc_offset_ns);
      seen |= 1;
    } else if (const auto* c = std::get_if<vg::io::CameraCalibration>(&*message)) {
      EXPECT_EQ(c->camera, "rgb");
      EXPECT_EQ(c->intrinsics.width, k.width);
      EXPECT_DOUBLE_EQ(c->intrinsics.fx, k.fx);
      seen |= 2;
    } else if (const auto* t = std::get_if<vg::io::StaticTransforms>(&*message)) {
      ASSERT_EQ(t->transforms.size(), 1u);
      EXPECT_EQ(t->transforms[0].child_frame, "cam/rgb");
      EXPECT_TRUE(t->transforms[0].parent_from_child.isApprox(body_from_camera, 1e-9));
      seen |= 4;
    } else if (const auto* p = std::get_if<vg::PoseStamped>(&*message)) {
      EXPECT_TRUE(p->world_from_frame.isApprox(world_from_body, 1e-9));
      seen |= 8;
    } else if (const auto* d = std::get_if<vg::io::DepthImage>(&*message)) {
      ASSERT_EQ(d->frame.depth.size(), n);
      for (std::size_t i = 0; i < n; ++i) {
        ASSERT_NEAR(d->frame.depth[i], rendered.depth[i], 1e-3) << i;
      }
      EXPECT_EQ(d->frame.confidence, confidence);
      EXPECT_EQ(d->frame.labels, labels);
      seen |= 16;
    } else if (const auto* status = std::get_if<vg::io::TrackingStatus>(&*message)) {
      EXPECT_EQ(status->state, vg::io::TrackingState::Limited);
      EXPECT_EQ(status->reason, "fast motion");
      seen |= 32;
    } else if (const auto* sample = std::get_if<vg::ImuSample>(&*message)) {
      EXPECT_DOUBLE_EQ(sample->angular_velocity.y(), 0.2);
      EXPECT_DOUBLE_EQ(sample->linear_acceleration.z(), 9.81);
      seen |= 64;
    } else if (const auto* g = std::get_if<vg::GpsFix>(&*message)) {
      EXPECT_DOUBLE_EQ(g->latitude, 41.88);
      EXPECT_DOUBLE_EQ(g->position_covariance(2, 2), 9.0);
      seen |= 128;
    }
  }
  EXPECT_EQ(seen, 255);
}

TEST(CApi, BuildsALiveMap) {
  vg_map_builder_config config;
  ASSERT_EQ(vg_map_builder_config_defaults(&config), VG_OK);
  EXPECT_GT(config.voxel_size, 0.0);
  EXPECT_GT(config.height_cell_size, 0.0);
  vg_map_builder* builder = nullptr;
  ASSERT_EQ(vg_map_builder_new(&config, &builder), VG_OK) << vg_last_error();

  const int frames = 24;
  const auto poses = walk(frames);
  for (int i = 0; i < frames; ++i) {
    const std::int64_t t = (i + 1) * kSecond;
    const vg_pose pose = to_c(poses[static_cast<std::size_t>(i)]);
    ASSERT_EQ(vg_map_builder_add_pose(builder, t, &pose), VG_OK) << vg_last_error();
    const vg::DepthFrame rendered =
        vg::test::render_depth(bed(), poses[static_cast<std::size_t>(i)]);
    const vg_depth_frame frame = depth_frame(t, rendered);
    ASSERT_EQ(vg_map_builder_add_depth(builder, "rgb", &frame), VG_OK) << vg_last_error();
  }
  ASSERT_EQ(vg_map_builder_flush(builder), VG_OK);

  vg_map_update update{};
  ASSERT_EQ(vg_map_builder_update(builder, &update), VG_OK);
  EXPECT_GT(update.cell_count, 0u);
  ASSERT_GT(update.block_count, 0u);
  // The builder owns update's arrays only until the next update: copy.
  const std::vector<std::int32_t> blocks(update.blocks, update.blocks + 3 * update.block_count);

  // Each block's voxels lie inside it.
  std::size_t voxels_seen = 0;
  for (std::size_t b = 0; b < blocks.size(); b += 3) {
    const std::int32_t* voxels = nullptr;
    std::size_t count = 0;
    ASSERT_EQ(vg_map_builder_block_voxels(builder, &blocks[b], &voxels, &count), VG_OK);
    for (std::size_t v = 0; v < count; ++v) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const std::int32_t first = blocks[b + axis] * 8;  // TsdfVolume::kBlockSize
        EXPECT_GE(voxels[3 * v + axis], first);
        EXPECT_LT(voxels[3 * v + axis], first + 8);
      }
    }
    voxels_seen += count;
  }
  EXPECT_GT(voxels_seen, 0u);

  // Nothing new since: the next update reports nothing.
  vg_map_update again{};
  ASSERT_EQ(vg_map_builder_update(builder, &again), VG_OK);
  EXPECT_EQ(again.cell_count, 0u);
  EXPECT_EQ(again.block_count, 0u);

  vg_map_builder_stats stats{};
  ASSERT_EQ(vg_map_builder_get_stats(builder, &stats), VG_OK);
  EXPECT_EQ(stats.integrated, static_cast<std::uint64_t>(frames));
  EXPECT_GT(stats.height_cells, 0u);

  vg_grid* height = nullptr;
  ASSERT_EQ(vg_map_builder_height_map(builder, &height), VG_OK);
  vg_grid_info info{};
  ASSERT_EQ(vg_grid_get_info(height, &info), VG_OK);
  EXPECT_DOUBLE_EQ(info.cell_size, config.height_cell_size);
  EXPECT_NEAR(grid_at(height, 0.5, 0.0), 0.3, 0.08);  // The bed's top.
  EXPECT_NEAR(grid_at(height, 0.5, 1.2), 0.0, 0.08);  // The ground beside it.
  vg_grid_free(height);
  vg_map_builder_free(builder);
}

TEST(CApi, DefersFramesAndFusesThemLater) {
  vg_map_builder_config config;
  ASSERT_EQ(vg_map_builder_config_defaults(&config), VG_OK);
  config.keyframe_translation = 100.0;  // Only the first frame is a keyframe.
  config.keyframe_rotation = 100.0;
  vg_map_builder* builder = nullptr;
  ASSERT_EQ(vg_map_builder_new(&config, &builder), VG_OK) << vg_last_error();

  const int frames = 8;
  const auto poses = walk(frames);
  for (int i = 0; i < frames; ++i) {
    const std::int64_t t = (i + 1) * kSecond;
    const vg_pose pose = to_c(poses[static_cast<std::size_t>(i)]);
    ASSERT_EQ(vg_map_builder_add_pose(builder, t, &pose), VG_OK);
    const vg::DepthFrame rendered =
        vg::test::render_depth(bed(), poses[static_cast<std::size_t>(i)]);
    const vg_depth_frame frame = depth_frame(t, rendered);
    ASSERT_EQ(vg_map_builder_add_depth(builder, "rgb", &frame), VG_OK);
  }
  ASSERT_EQ(vg_map_builder_flush(builder), VG_OK);
  vg_map_builder_stats stats{};
  ASSERT_EQ(vg_map_builder_get_stats(builder, &stats), VG_OK);
  EXPECT_EQ(stats.integrated, 1u);
  EXPECT_EQ(stats.waiting, static_cast<std::uint64_t>(frames - 1));

  std::size_t remaining = 99;
  ASSERT_EQ(vg_map_builder_integrate_deferred(builder, 3, &remaining), VG_OK);
  EXPECT_EQ(remaining, static_cast<std::size_t>(frames - 4));
  ASSERT_EQ(vg_map_builder_integrate_deferred(builder, 100, &remaining), VG_OK);
  EXPECT_EQ(remaining, 0u);
  ASSERT_EQ(vg_map_builder_get_stats(builder, &stats), VG_OK);
  EXPECT_EQ(stats.integrated, static_cast<std::uint64_t>(frames));
  vg_map_builder_free(builder);
}

TEST(CApi, RunsThePipelineFromARecording) {
  TempDir dir("vg_c_api_pipeline");
  const auto session = dir.path() / "walk.vgcap.mcap";
  record_walk(session, 24);
  const std::string package = utf8(dir.path() / "walk");

  // Replay. The recording has no GPS, so the sun stage can't run: the job
  // completes without it.
  std::vector<std::string> lines;
  int32_t complete = -1;
  const char* replay_settings[] = {"--set", "pipeline=voxels,surface,ground,terrain,sun"};
  ASSERT_EQ(vg_replay(utf8(session).c_str(), package.c_str(), replay_settings, 2, collect, &lines,
                      &complete),
            VG_OK)
      << vg_last_error();
  EXPECT_EQ(complete, 0);
  EXPECT_TRUE(any_line_has(lines, "map package written to"));

  // The ground map: flat ground at 0, the bed patched over.
  vg_grid* ground = nullptr;
  ASSERT_EQ(vg_grid_load(utf8(dir.path() / "walk" / "ground.asc").c_str(), &ground), VG_OK)
      << vg_last_error();
  float lo = 0.0f;
  float hi = 0.0f;
  std::size_t cells = 0;
  ASSERT_EQ(vg_grid_range(ground, &lo, &hi, &cells), VG_OK);
  EXPECT_GT(cells, 0u);
  EXPECT_NEAR(lo, 0.0f, 0.1f);
  EXPECT_NEAR(hi, 0.0f, 0.1f);
  vg_grid_free(ground);

  // The 3D map.
  vg_package* map = nullptr;
  ASSERT_EQ(vg_package_open(package.c_str(), &map), VG_OK) << vg_last_error();
  const std::int32_t* voxels = nullptr;
  const std::uint8_t* classes = nullptr;
  std::size_t count = 0;
  double voxel_size = 0.0;
  ASSERT_EQ(vg_package_voxels(map, &voxels, &classes, &count, &voxel_size), VG_OK);
  EXPECT_GT(count, 0u);
  EXPECT_GT(voxel_size, 0.0);
  EXPECT_EQ(classes[0], VG_CLASS_UNKNOWN);  // The recording has no labels.
  vg_geo_reference geo{};
  int32_t has_geo = -1;
  ASSERT_EQ(vg_package_geo_reference(map, &geo, &has_geo), VG_OK);
  EXPECT_EQ(has_geo, 0);
  vg_package_free(map);

  // Re-run the sun stage with a location given.
  const char* stages[] = {"sun"};
  const char* sun_settings[] = {"--set", "sun.latitude=41.88", "--set", "sun.longitude=-87.63",
                                "--set", "sun.x_bearing_deg=0",  "--set", "sun.step_minutes=240"};
  ASSERT_EQ(vg_map_stages(package.c_str(), stages, 1, sun_settings, 8, nullptr, nullptr,
                          &complete),
            VG_OK)
      << vg_last_error();
  EXPECT_EQ(complete, 1);
  EXPECT_TRUE(std::filesystem::exists(dir.path() / "walk" / "sun_hours.asc"));

  // Build settings can't change without a rebuild.
  const char* rebuild[] = {"--set", "build.voxel_size=0.05"};
  EXPECT_EQ(vg_map_stages(package.c_str(), stages, 1, rebuild, 2, nullptr, nullptr, nullptr),
            VG_ERROR_CONFIG);

  // A site of one walk.
  const std::string site_dir = utf8(dir.path() / "site");
  const char* packages[] = {package.c_str()};
  int32_t all_added = -1;
  ASSERT_EQ(vg_site_add(site_dir.c_str(), packages, 1, 0, nullptr, 0, nullptr, nullptr,
                        &all_added),
            VG_OK)
      << vg_last_error();
  EXPECT_EQ(all_added, 1);
  ASSERT_EQ(vg_site_make_maps(site_dir.c_str(), nullptr, 0, nullptr, nullptr), VG_OK);
  EXPECT_TRUE(std::filesystem::exists(dir.path() / "site" / "current" / "ground.asc"));

  vg_site* site = nullptr;
  ASSERT_EQ(vg_site_open(site_dir.c_str(), &site), VG_OK) << vg_last_error();
  std::size_t visits = 0;
  ASSERT_EQ(vg_site_visit_count(site, &visits), VG_OK);
  ASSERT_EQ(visits, 1u);
  vg_site_visit visit{};
  ASSERT_EQ(vg_site_get_visit(site, 0, &visit), VG_OK);
  EXPECT_EQ(visit.has_fit, 0);  // The first visit sets the site's frame.
  EXPECT_TRUE(std::filesystem::equivalent(std::filesystem::u8path(visit.package_dir),
                                          std::filesystem::u8path(package)));
  EXPECT_EQ(vg_site_get_visit(site, 1, &visit), VG_ERROR_INVALID_ARGUMENT);
  vg_site_free(site);
}
