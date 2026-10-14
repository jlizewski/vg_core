// vg_replay: build the full map package (3D map, surface, ground, objects, sun) from a recording.
//
//   vg_replay <session.mcap> <package_dir> [--config FILE] [--set PATH=VALUE] [flags]
//
// Replays the session into the 3D voxel map, then runs the derived stages
// listed in the `pipeline` setting, writing the package described in
// vg_core/map_package.hpp. Any earlier package in package_dir is replaced.
//
// Settings come from config/mapping.yaml's defaults, then each --config file,
// --set and flag in order (vg_core/config/mapping_config.hpp). The package
// keeps the settings it was made with in config.yaml; to re-run a derived
// stage with new settings without replaying, use vg_map.
//
// The sun stage needs to know where the map is: from the session's GPS, or
// sun.latitude/longitude. Without either it's skipped and the rest is still
// written.

#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "vg_core/config/mapping_config.hpp"
#include "vg_core/io/map_build.hpp"
#include "vg_core/map_package.hpp"

namespace {

int usage(const vg::config::ParamSet& params) {
  std::cerr << "usage: vg_replay <session.mcap> <package_dir> [settings]\n"
            << params.flags_help();
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  vg::config::MappingConfig config;
  vg::config::ParamSet params = vg::config::bind(config);
  const std::vector<std::string> args(argv + 1, argv + argc);
  if (args.size() < 2 || args[0] == "--help" || args[0] == "-h") {
    return usage(params);
  }
  const std::string session_path = args[0];
  const std::string package_dir = args[1];

  try {
    params.apply_args({args.begin() + 2, args.end()});
    vg::config::check_stages(config.pipeline);
  } catch (const vg::config::ConfigError& e) {
    std::cerr << "vg_replay: " << e.what() << "\n";
    return 2;
  }

  try {
    const vg::io::BuildOptions build = config.build_options();
    const vg::io::BuildSummary summary =
        vg::io::build_map_package(session_path, package_dir, build, std::cout);
    // Record what was worked out, so config.yaml says what the map was made with.
    config.voxel_size = build.builder.tsdf.voxel_size;
    config.truncation = build.builder.tsdf.truncation_distance;
    config.build.min_weight = summary.min_weight;
    config.year = config.sun_options().year;

    vg::MapPackage package(package_dir, std::cout);
    vg::config::run_stages(package, config, config.pipeline);
    vg::config::save_settings(params, package.dir() / vg::package_file::kConfig,
                              vg::config::kPackageHeader);
    std::cout << "map package written to " << package_dir << "\n";
  } catch (const std::exception& e) {
    std::cerr << "vg_replay: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
