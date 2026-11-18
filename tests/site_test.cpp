#include "vg_core/site.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
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
    // One directory per test: ctest runs them in parallel.
    root_ = fs::temp_directory_path() /
            ("vg_site_test_" +
             std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
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

std::string read_all(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

TEST_F(SiteTest, MakesTheGroundLayerFromEveryVisit) {
  const fs::path site_dir = root_ / "site";
  vg::Site site(site_dir);
  site.add(root_ / "walks" / "recording-20261009-175006", {}, log_);
  site.add(root_ / "walks" / "recording-20261016-115329", {}, log_, true);

  const vg::GroundLayer layer = site.make_ground_layer(log_);
  const fs::path dir = site_dir / vg::ground_layer_file::kDir;
  for (const char* name :
       {vg::ground_layer_file::kEvidence, vg::ground_layer_file::kHeights,
        vg::ground_layer_file::kState, vg::ground_layer_file::kProvenance,
        vg::ground_layer_file::kObservations, vg::ground_layer_file::kSource,
        vg::ground_layer_file::kSpread,
        vg::ground_layer_file::kInfo}) {
    EXPECT_TRUE(fs::is_regular_file(dir / name)) << name;
  }
  std::size_t measured = 0;
  for (const vg::GroundState s : layer.state) {
    measured += s == vg::GroundState::Measured ? 1 : 0;
  }
  EXPECT_GT(measured, 0u);
  EXPECT_NE(read_all(dir / vg::ground_layer_file::kEvidence).find("recording-20261016-115329"),
            std::string::npos);

  // The same inputs give the same files.
  const std::string heights = read_all(dir / vg::ground_layer_file::kHeights);
  const std::string state = read_all(dir / vg::ground_layer_file::kState);
  const std::string info = read_all(dir / vg::ground_layer_file::kInfo);
  site.make_ground_layer(log_);
  EXPECT_EQ(read_all(dir / vg::ground_layer_file::kHeights), heights);
  EXPECT_EQ(read_all(dir / vg::ground_layer_file::kState), state);
  EXPECT_EQ(read_all(dir / vg::ground_layer_file::kInfo), info);

  // An edit is picked up from edits.csv: the bed marked as standing on the
  // ground, which then runs on beneath it.
  vg::GroundEdit bed;
  bed.id = "bed";
  bed.time = "2026-11-16T12:00:00Z";
  bed.kind = vg::GroundEdit::Kind::NotGround;
  bed.polygon = {{0.2, -0.3}, {0.9, -0.3}, {0.9, 0.5}, {0.2, 0.5}};
  vg::save_ground_edits({bed}, dir / vg::ground_layer_file::kEdits);
  const vg::GroundLayer edited = site.make_ground_layer(log_);
  EXPECT_NE(read_all(dir / vg::ground_layer_file::kInfo).find("\"edits\": [\"bed\"]"),
            std::string::npos);
  std::size_t covered = 0;
  for (std::size_t i = 0; i < edited.state.size(); ++i) {
    if (edited.state[i] == vg::GroundState::Covered) {
      ++covered;
      EXPECT_EQ(edited.provenance[i], vg::Provenance::Edited);
      EXPECT_FALSE(std::isnan(edited.heights.heights[i]));
    }
  }
  EXPECT_GT(covered, 10u);  // 0.7 x 0.8 m at 0.15 m cells: about 20.
}

TEST_F(SiteTest, MakesTheFeatureLayerFromEveryVisit) {
  // Both walks saw the same bed: one block, seen twice, in the same place.
  // It's 0.4 m tall: the element band is lowered to it.
  for (const char* name : {"recording-20261009-175006", "recording-20261016-115329"}) {
    vg::MapPackage package(root_ / "walks" / name, log_);
    vg::GroundOptions options;
    options.config.element_band_low = 0.15;
    options.config.element_band_high = 0.35;
    package.make_ground(options);
  }
  const fs::path site_dir = root_ / "site";
  vg::Site site(site_dir);
  site.add(root_ / "walks" / "recording-20261009-175006", {}, log_);
  site.add(root_ / "walks" / "recording-20261016-115329", {}, log_, true);
  const vg::FeatureLayer layer = site.make_feature_layer(log_);
  // The bed's walls, as fused from a few views (one piece or a few): each
  // seen by both walks, in the same place, within the bed.
  ASSERT_FALSE(layer.elements.empty()) << log_.str();
  for (const vg::SiteFeature& e : layer.elements) {
    EXPECT_EQ(e.seen, 2) << e.id;
    EXPECT_FALSE(e.retired) << e.id;
    EXPECT_NEAR(e.spread, 0.0, 1e-9) << e.id;
    EXPECT_GT(e.feature.center.x(), 0.1);  // The scene's bed: 0.2-0.9 by -0.3-0.5 m.
    EXPECT_LT(e.feature.center.x(), 1.0);
    EXPECT_GT(e.feature.center.y(), -0.4);
    EXPECT_LT(e.feature.center.y(), 0.6);
    EXPECT_NEAR(e.feature.height, 0.4, 0.1);
  }
  EXPECT_EQ(layer.sightings.size(), 2 * layer.elements.size());
  const fs::path dir = site_dir / vg::feature_layer_file::kDir;
  EXPECT_NE(read_all(dir / vg::feature_layer_file::kElements).find(layer.elements[0].id + ","),
            std::string::npos);
  EXPECT_TRUE(fs::is_regular_file(dir / vg::feature_layer_file::kObservations));
}

TEST_F(SiteTest, PlacesAWalkFromTheSameTrackingSessionInItsFrame) {
  // Both walks recorded in one tracking session: the second is placed as the
  // first, with no search (searching, the bed matches itself turned too).
  for (const char* name : {"recording-20261009-175006", "recording-20261016-115329"}) {
    const fs::path manifest = root_ / "walks" / name / vg::package_file::kManifest;
    std::string text = read_all(manifest);
    const std::string empty = "\"tracking_frame\": \"\"";
    ASSERT_NE(text.find(empty), std::string::npos) << text;
    text.replace(text.find(empty), empty.size(), "\"tracking_frame\": \"run-1\"");
    std::ofstream(manifest, std::ios::binary) << text;
  }
  vg::Site site(root_ / "site");
  site.add(root_ / "walks" / "recording-20261009-175006", {}, log_);
  const vg::RegistrationResult second =
      site.add(root_ / "walks" / "recording-20261016-115329", {}, log_);
  ASSERT_TRUE(second.accepted) << log_.str();
  EXPECT_NE(log_.str().find("same tracking session"), std::string::npos) << log_.str();
  ASSERT_EQ(site.visits().size(), 2u);
  EXPECT_DOUBLE_EQ(site.visits()[1].site_from_map.yaw, 0.0);
  EXPECT_EQ(site.visits()[1].site_from_map.translation, Eigen::Vector3d::Zero());
  // (The test bed is too small to judge a fit: placed on the session alone.)

  // Switched off, it's searched for as before.
  vg::Site other(root_ / "other");
  other.add(root_ / "walks" / "recording-20261009-175006", {}, log_);
  vg::RegistrationConfig config;
  config.use_tracking_frame = false;
  std::ostringstream log;
  other.add(root_ / "walks" / "recording-20261016-115329", config, log, true);
  EXPECT_EQ(log.str().find("same tracking session"), std::string::npos);
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
