#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "vg_core/config/mapping_config.hpp"
#include "vg_core/config/params.hpp"
#include "vg_core/semantics.hpp"

namespace {

using vg::config::ConfigError;
using vg::config::ParamSet;

struct Settings {
  double slope = 1.0;
  float weight = 0.5f;
  int count = 2;
  bool carve = true;
  std::uint8_t confidence = 0;
  std::optional<double> voxel;
  std::vector<std::string> stages = {"a", "b"};
  double turn = 0.0;  // radians
};

ParamSet bind(Settings& s) {
  ParamSet p;
  p.add("stages", s.stages, "Stages to run.");
  p.section("ground", "The ground.");
  p.add("ground.max_slope", s.slope, "Steepest ground.", "--max-slope");
  p.add("ground.count", s.count, "A count.");
  p.section("fusion", "Fusion.");
  p.add("fusion.min_weight", s.weight, "Minimum weight.");
  p.add("fusion.carve", s.carve, "Carve.", "--carve");
  p.add("fusion.min_confidence", s.confidence, "Confidence.");
  p.add("fusion.voxel_size", s.voxel, "Voxel size, or auto.", "--voxel");
  p.add_degrees("fusion.turn", s.turn, "A turn, in degrees.");
  return p;
}

std::string error_of(const std::function<void()>& f) {
  try {
    f();
  } catch (const ConfigError& e) {
    return e.what();
  }
  return "";
}

TEST(Params, LoadsYamlSettings) {
  Settings s;
  ParamSet p = bind(s);
  p.load_yaml(
      "stages: [c, d, e]\n"
      "ground:\n"
      "  max_slope: 0.6\n"
      "fusion:\n"
      "  min_weight: 0.05\n"
      "  carve: no\n"
      "  min_confidence: 128\n"
      "  voxel_size: 0.1\n"
      "  turn: 90\n");
  EXPECT_EQ(s.stages, (std::vector<std::string>{"c", "d", "e"}));
  EXPECT_DOUBLE_EQ(s.slope, 0.6);
  EXPECT_FLOAT_EQ(s.weight, 0.05f);
  EXPECT_FALSE(s.carve);
  EXPECT_EQ(s.confidence, 128);
  ASSERT_TRUE(s.voxel);
  EXPECT_DOUBLE_EQ(*s.voxel, 0.1);
  EXPECT_NEAR(s.turn, 3.14159265358979323846 / 2.0, 1e-12);
  EXPECT_EQ(s.count, 2);  // Untouched.

  p.load_yaml("fusion:\n  voxel_size: auto\n");
  EXPECT_FALSE(s.voxel);
  p.load_yaml("");  // An empty file changes nothing.
  EXPECT_DOUBLE_EQ(s.slope, 0.6);
}

TEST(Params, RejectsUnknownKeysAndBadValues) {
  Settings s;
  ParamSet p = bind(s);
  const auto typo = error_of([&] { p.load_yaml("ground:\n  max_slop: 0.6\n", "my.yaml"); });
  EXPECT_NE(typo.find("my.yaml:2"), std::string::npos) << typo;
  EXPECT_NE(typo.find("did you mean ground.max_slope?"), std::string::npos) << typo;
  // The right key in the wrong section.
  const auto section = error_of([&] { p.load_yaml("fusion:\n  max_slope: 0.6\n"); });
  EXPECT_NE(section.find("did you mean ground.max_slope?"), std::string::npos) << section;
  EXPECT_NE(error_of([&] { p.load_yaml("grund:\n  max_slope: 0.6\n"); }).find("unknown section"),
            std::string::npos);
  EXPECT_NE(error_of([&] { p.load_yaml("ground:\n  max_slope: steep\n"); }).find("not a number"),
            std::string::npos);
  EXPECT_NE(error_of([&] { p.load_yaml("fusion:\n  min_confidence: 300\n"); }).find("range"),
            std::string::npos);
  EXPECT_NE(error_of([&] { p.load_yaml("ground:\n  max_slope: auto\n"); }).find("not a number"),
            std::string::npos);
  EXPECT_NE(error_of([&] { p.load_yaml("ground: [1, 2\n"); }), "");
  EXPECT_DOUBLE_EQ(s.slope, 1.0);
}

TEST(Params, CommandLineAppliesInOrder) {
  const auto dir = std::filesystem::temp_directory_path() / "vg_params_test";
  std::filesystem::create_directories(dir);
  const auto file = dir / "settings.yaml";
  std::ofstream(file) << "ground:\n  max_slope: 0.3\n  count: 7\n";

  Settings s;
  ParamSet p = bind(s);
  p.apply_args({"--max-slope", "0.9", "--config", file.string(), "--set", "ground.count=9",
                "--carve", "0", "--set", "stages=x,y"});
  EXPECT_DOUBLE_EQ(s.slope, 0.3);  // The file came after the flag.
  EXPECT_EQ(s.count, 9);
  EXPECT_FALSE(s.carve);
  EXPECT_EQ(s.stages, (std::vector<std::string>{"x", "y"}));

  EXPECT_NE(error_of([&] { p.apply_args({"--max-slop", "1"}); }).find("unknown option"),
            std::string::npos);
  EXPECT_NE(error_of([&] { p.apply_args({"--set", "ground.max_slope"}); }), "");
  EXPECT_NE(error_of([&] { p.apply_args({"--max-slope"}); }).find("needs a value"),
            std::string::npos);
  std::filesystem::remove_all(dir);
}

TEST(Params, WrittenYamlReadsBackTheSame) {
  Settings a;
  a.slope = 0.123456789;
  a.weight = 0.05f;
  a.voxel = 0.0762;
  a.stages = {"p", "q"};
  a.turn = 0.3;
  std::ostringstream yaml;
  bind(a).write_yaml(yaml, "A header.");
  EXPECT_EQ(yaml.str().rfind("# A header.\n", 0), 0u);
  EXPECT_NE(yaml.str().find("# Steepest ground.\n  max_slope: 0.123456789\n"), std::string::npos)
      << yaml.str();

  Settings b;
  b.voxel.reset();
  ParamSet p = bind(b);
  p.load_yaml(yaml.str());
  EXPECT_DOUBLE_EQ(b.slope, a.slope);
  EXPECT_FLOAT_EQ(b.weight, a.weight);
  EXPECT_EQ(b.voxel, a.voxel);
  EXPECT_EQ(b.stages, a.stages);
  EXPECT_NEAR(b.turn, a.turn, 1e-12);
}

TEST(MappingConfig, CheckedInDefaultsAreCurrent) {
  // config/mapping.yaml is what `vg config defaults` writes; regenerate it
  // after adding or changing a setting.
  vg::config::MappingConfig config;
  std::ostringstream expected;
  vg::config::bind(config).write_yaml(expected, vg::config::kDefaultsHeader);
  std::ifstream in(std::filesystem::path(VG_CORE_SOURCE_DIR) / "config" / "mapping.yaml",
                   std::ios::binary);
  ASSERT_TRUE(in) << "config/mapping.yaml is missing";
  std::string actual{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
  actual.erase(std::remove(actual.begin(), actual.end(), '\r'), actual.end());
  EXPECT_EQ(actual, expected.str())
      << "config/mapping.yaml is stale: run `vg config defaults > config/mapping.yaml`";
}

TEST(MappingConfig, WorksOutAutomaticValues) {
  vg::config::MappingConfig config;
  auto build = config.build_options();
  EXPECT_DOUBLE_EQ(build.cell_size, 0.1524);
  EXPECT_DOUBLE_EQ(build.builder.tsdf.voxel_size, 0.0762);
  EXPECT_DOUBLE_EQ(build.builder.tsdf.truncation_distance, 3 * 0.0762);
  EXPECT_TRUE(std::isinf(build.to));
  EXPECT_FALSE(build.min_weight);

  ParamSet p = vg::config::bind(config);
  p.apply_args({"--resolution", "0.2", "--trunc", "0.5", "--to", "30", "--year", "2030"});
  build = config.build_options();
  EXPECT_DOUBLE_EQ(build.builder.tsdf.voxel_size, 0.1);
  EXPECT_DOUBLE_EQ(build.builder.tsdf.truncation_distance, 0.5);
  EXPECT_DOUBLE_EQ(build.to, 30.0);
  EXPECT_EQ(config.sun_options().year, 2030);
}

TEST(MappingConfig, StagesAndBuildSettingsAreKnown) {
  EXPECT_NO_THROW(vg::config::check_stages({"voxels", "ground", "layers"}));
  EXPECT_THROW(vg::config::check_stages({"grond"}), ConfigError);
  EXPECT_TRUE(vg::config::is_build_setting("build.depth.max_depth"));
  EXPECT_TRUE(vg::config::is_build_setting("map.resolution"));
  EXPECT_FALSE(vg::config::is_build_setting("voxels.min_weight"));
  EXPECT_FALSE(vg::config::is_build_setting("ground.max_slope"));
}

TEST(MappingConfig, SemanticClassesByName) {
  vg::config::MappingConfig config;
  EXPECT_NO_THROW(config.validate());
  EXPECT_EQ(config.build_options().builder.tsdf.drop_labels,
            vg::mask_of(vg::SemanticClass::Dynamic) | vg::mask_of(vg::SemanticClass::Sky));

  ParamSet p = vg::config::bind(config);
  p.apply_args({"--set", "build.semantics.drop_classes=[]"});
  EXPECT_EQ(config.build_options().builder.tsdf.drop_labels, 0u);
  p.apply_args({"--set", "build.semantics.drop_classes=peeple"});
  const auto error = error_of([&] { config.validate(); });
  EXPECT_NE(error.find("unknown semantic class peeple"), std::string::npos) << error;
}

}  // namespace
