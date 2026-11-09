#include "vg_core/site.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <sstream>

#include "test_scene.hpp"
#include "vg_core/height_map_io.hpp"
#include "vg_core/map_package.hpp"

namespace {

namespace fs = std::filesystem;

// A site in a fresh temporary directory, and a small package (a raised bed)
// to add to it, twice over (as two walks).
class SiteTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() / "vg_site_test";
    fs::remove_all(root_);
    vg::test::Scene scene;
    scene.box = vg::test::Box{{0.2, -0.3, 0.0}, {0.9, 0.5, 0.4}};
    vg::TsdfConfig config;
    config.max_depth = 2.5;
    config.voxel_size = 0.0762;
    config.truncation_distance = 3 * config.voxel_size;
    vg::TsdfVolume volume(config);
    for (const Eigen::Vector3d& eye :
         {Eigen::Vector3d(0.5, 0.1, 1.5), Eigen::Vector3d(-0.7, 0.5, 1.2),
          Eigen::Vector3d(1.7, -0.5, 1.2)}) {
      const auto pose = vg::test::look_at(eye, {0.5, 0.0, 0.0}, Eigen::Vector3d::UnitZ());
      volume.integrate(vg::test::render_depth(scene, pose), pose);
    }
    for (const char* name : {"recording-20261009-175006", "recording-20261016-115329"}) {
      vg::MapMetadata metadata;
      metadata.session = std::string(name) + ".vgcap.mcap";
      metadata.cell_size = 0.1524;
      const fs::path dir = root_ / "walks" / name;
      vg::create_map_package(dir, volume, metadata, vg::StageSettings(), log_);
      vg::MapPackage package(dir, log_);
      package.make_surface();
      package.make_ground();
      package.make_terrain();
    }
  }

  void TearDown() override { fs::remove_all(root_); }

  fs::path root_;
  std::ostringstream log_;
};

TEST(SiteRecordedTimeTest, FindsTheStampInARecordingsName) {
  EXPECT_EQ(vg::recorded_time("recording-20261016-115329.vgcap.mcap"), "20261016-115329");
  EXPECT_FALSE(vg::recorded_time("garden.mcap").has_value());
}

TEST_F(SiteTest, AddsVisitsOldestFirstAndMakesCurrentMaps) {
  const fs::path site_dir = root_ / "site";
  {
    vg::Site site(site_dir);
    // The later walk first: it sets the site frame.
    EXPECT_TRUE(site.add(root_ / "walks" / "recording-20261016-115329", {}, log_).accepted);
    // One small bed matches itself turned by 90 degrees too: forced in.
    site.add(root_ / "walks" / "recording-20261009-175006", {}, log_, true);
    EXPECT_THROW(site.add(root_ / "walks" / "recording-20261009-175006", {}, log_, true),
                 std::runtime_error);
    site.make_maps(log_);
  }
  vg::Site site(site_dir);  // Read back from site.csv.
  ASSERT_EQ(site.visits().size(), 2u);
  EXPECT_EQ(site.visits()[0].recorded, "20261009-175006");
  EXPECT_EQ(site.visits()[1].recorded, "20261016-115329");
  EXPECT_TRUE(site.visits()[0].fit.has_value());
  EXPECT_FALSE(site.visits()[1].fit.has_value());
  EXPECT_DOUBLE_EQ(site.visits()[1].site_from_map.yaw, 0.0);
  EXPECT_TRUE(fs::exists(site.package_dir(site.visits()[0]) / vg::package_file::kMap));

  for (const char* file : {"surface.asc", "ground.asc", "terrain.asc", "features.asc",
                           vg::site_file::kVisit, vg::site_file::kChanges}) {
    EXPECT_TRUE(fs::is_regular_file(site_dir / vg::site_file::kCurrent / file)) << file;
  }
  // Cells come from the latest visit that saw them: mostly the 16th's.
  const auto visit = vg::load_height_map(site_dir / vg::site_file::kCurrent / vg::site_file::kVisit);
  int latest = 0;
  int earlier = 0;
  for (const float v : visit.heights) {
    latest += v == 2.0f ? 1 : 0;
    earlier += v == 1.0f ? 1 : 0;
  }
  EXPECT_GT(latest, earlier);
}

}  // namespace
