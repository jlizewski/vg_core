#include "vg_core/site.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <sstream>

#include "test_scene.hpp"
#include "vg_core/height_map_io.hpp"
#include "vg_core/map_package.hpp"
#include "vg_core/semantics.hpp"

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
  EXPECT_GT(fs::file_size(site_dir / vg::site_file::kCurrent / vg::site_file::kModel), 1000u);
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

TEST(SiteModelTest, ColorsTheSurfaceAndHangsTheCanopyOverTheTerrain) {
  const fs::path root = fs::temp_directory_path() / "vg_site_model_test";
  fs::remove_all(root);
  fs::create_directories(root / vg::site_file::kCurrent);
  const auto grid = [](float value, double origin_x) {
    vg::HeightMap map;
    map.origin = {origin_x, 0.0};
    map.cell_size = 0.5;
    map.width = 2;
    map.height = 2;
    map.heights.assign(4, value);
    return map;
  };
  const fs::path current = root / vg::site_file::kCurrent;
  vg::save_height_map(grid(-1.5f, 0.0), current / vg::package_file::kSurface);
  vg::save_height_map(grid(-1.7f, 0.0), current / vg::package_file::kTerrain);
  vg::save_height_map(grid(static_cast<float>(vg::SemanticClass::Plant), 0.0),
                      current / vg::package_file::kSemantic);
  // The canopy's grid is shifted half a cell: still sampled at its centers.
  vg::save_height_map(grid(3.0f, 0.25), current / vg::package_file::kCanopy);
  vg::save_height_map(grid(30.0f, 0.25), current / vg::package_file::kCanopyTop);

  const std::vector<vg::MeshPart> parts = vg::Site(root).model();
  ASSERT_EQ(parts.size(), 2u);
  EXPECT_EQ(parts[0].name, "surface");
  EXPECT_EQ(parts[0].mesh.triangles(), 2u);
  const auto plant = vg::semantic_class_color(vg::SemanticClass::Plant);
  EXPECT_EQ(parts[0].mesh.colors[0][1], plant[1]);
  EXPECT_FLOAT_EQ(parts[0].mesh.positions[0].z(), -1.5f);

  // Underside 3 m over the terrain, where the terrain covers it (the shifted
  // canopy's second column is beyond it).
  EXPECT_EQ(parts[1].name, "canopy");
  EXPECT_LT(parts[1].opacity, 1.0f);
  ASSERT_EQ(parts[1].mesh.positions.size(), 2u);
  for (const Eigen::Vector3f& p : parts[1].mesh.positions) {
    EXPECT_FLOAT_EQ(p.z(), 1.3f);
  }
  fs::remove_all(root);
}

}  // namespace
