#include "vg_core/map_package.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>

#include "test_scene.hpp"
#include "vg_core/height_map_io.hpp"

namespace {

namespace fs = std::filesystem;
namespace file = vg::package_file;

std::string read_file(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// A package in a fresh temporary directory, built from the raised bed at the
// package's default 3 inch voxels.
class MapPackageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("vg_map_package_" + std::string(::testing::UnitTest::GetInstance()
                                                ->current_test_info()
                                                ->name()));
    fs::remove_all(dir_);

    vg::test::Scene scene;
    scene.box = vg::test::Box{{0.2, -0.3, 0.0}, {0.8, 0.3, 0.3}};
    vg::TsdfConfig config;
    config.max_depth = 2.5;
    config.voxel_size = 0.0762;
    config.truncation_distance = 3 * config.voxel_size;
    vg::TsdfVolume volume(config);
    const Eigen::Vector3d target(0.5, 0.0, 0.0);
    for (const Eigen::Vector3d& eye :
         {Eigen::Vector3d(0.5, 0.0, 1.5), Eigen::Vector3d(-0.7, 0.5, 1.2),
          Eigen::Vector3d(1.7, -0.5, 1.2)}) {
      const auto pose = vg::test::look_at(eye, target, Eigen::Vector3d::UnitZ());
      volume.integrate(vg::test::render_depth(scene, pose), pose);
    }
    metadata_.session = "bed.mcap";
    metadata_.cell_size = 0.1524;
    vg::create_map_package(dir_, volume, metadata_, vg::StageSettings().add("test", true), log_);
  }

  void TearDown() override { fs::remove_all(dir_); }

  fs::path dir_;
  vg::MapMetadata metadata_;
  std::ostringstream log_;
};

vg::SunOptions quick_sun() {
  vg::SunOptions sun;
  sun.year = 2026;
  sun.step_minutes = 24 * 60;  // One sample a day keeps the test fast.
  sun.latitude = 42.36;
  sun.longitude = -71.06;
  sun.x_bearing_deg = 90.0;
  return sun;
}

TEST_F(MapPackageTest, WritesEveryMap) {
  vg::MapPackage package(dir_, log_);
  package.make_surface();
  package.make_ground();
  ASSERT_TRUE(package.make_sun(quick_sun()));

  for (const char* name : {file::kMap, file::kVoxels, file::kSurface, file::kGround,
                           file::kObjects, file::kSunHours, file::kSunEnergy, file::kManifest}) {
    EXPECT_TRUE(fs::is_regular_file(dir_ / name)) << name;
  }
  const auto ground = vg::load_height_map(dir_ / file::kGround);
  EXPECT_DOUBLE_EQ(ground.cell_size, 0.1524);
  EXPECT_DOUBLE_EQ(vg::load_height_map(dir_ / file::kSurface).cell_size, 0.1524);
  EXPECT_EQ(vg::load_height_map(dir_ / file::kSunHours).width, ground.width);

  const std::string manifest = read_file(dir_ / file::kManifest);
  for (const char* expected :
       {"\"session\": \"bed.mcap\"", "\"build\": {\"test\": true}", "\"surface\": {",
        "\"ground\": {", "\"sun\": {\"year\": 2026", "\"name\": \"sun_energy.asc\"",
        "\"geo_reference\": null"}) {
    EXPECT_NE(manifest.find(expected), std::string::npos) << expected << "\n" << manifest;
  }
}

TEST_F(MapPackageTest, RerunsOneStageFromTheSavedMap) {
  {
    vg::MapPackage package(dir_, log_);
    package.make_surface();
    package.make_ground();
  }
  const std::string map = read_file(dir_ / file::kMap);
  const std::string surface = read_file(dir_ / file::kSurface);
  const std::string ground = read_file(dir_ / file::kGround);

  // Same settings, fresh process: same ground.
  {
    vg::MapPackage package(dir_, log_);
    package.make_ground();
  }
  EXPECT_EQ(read_file(dir_ / file::kGround), ground);

  // New ground settings change only the ground.
  vg::GroundOptions finer;
  finer.cell_size = 0.1;
  finer.config.max_hole_area = 0.0;
  {
    vg::MapPackage package(dir_, log_);
    package.make_ground(finer);
  }
  EXPECT_NE(read_file(dir_ / file::kGround), ground);
  EXPECT_DOUBLE_EQ(vg::load_height_map(dir_ / file::kGround).cell_size, 0.1);
  EXPECT_EQ(read_file(dir_ / file::kMap), map);
  EXPECT_EQ(read_file(dir_ / file::kSurface), surface);

  // The manifest keeps the surface stage and records the new ground settings.
  const std::string manifest = read_file(dir_ / file::kManifest);
  EXPECT_NE(manifest.find("\"surface\": {\"cell_size_m\": 0.1524}"), std::string::npos)
      << manifest;
  EXPECT_NE(manifest.find("\"ground\": {\"cell_size_m\": 0.1, "), std::string::npos) << manifest;
  EXPECT_NE(manifest.find("\"max_hole_area_m2\": 0,"), std::string::npos) << manifest;
}

TEST_F(MapPackageTest, SunNeedsALocationAndGround) {
  vg::MapPackage package(dir_, log_);
  EXPECT_FALSE(package.make_sun(quick_sun()));  // No ground yet.
  package.make_ground();
  EXPECT_FALSE(package.make_sun(vg::SunOptions{}));  // No GPS in the map, none given.
  EXPECT_FALSE(fs::exists(dir_ / file::kSunHours));
  EXPECT_TRUE(package.make_sun(quick_sun()));
}

TEST_F(MapPackageTest, NewPackageClearsTheOldOne) {
  {
    vg::MapPackage package(dir_, log_);
    package.make_ground();
  }
  ASSERT_TRUE(fs::exists(dir_ / file::kGround));
  vg::create_map_package(dir_, vg::TsdfVolume(), metadata_, vg::StageSettings(), log_);
  EXPECT_FALSE(fs::exists(dir_ / file::kGround));
  EXPECT_TRUE(fs::exists(dir_ / file::kMap));
  EXPECT_EQ(read_file(dir_ / file::kManifest).find("\"ground\""), std::string::npos);
}

TEST(MapPackage, MissingMapThrows) {
  std::ostringstream log;
  EXPECT_THROW(vg::MapPackage(fs::temp_directory_path() / "vg_no_such_package", log),
               std::runtime_error);
}

}  // namespace
