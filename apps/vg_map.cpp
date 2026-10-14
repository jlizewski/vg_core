// vg_map: re-run derived stages (voxels, surface, ground, sun, layers) of a map package from its saved 3D map.
//
//   vg_map <package_dir> <stage>... [--config FILE] [--set PATH=VALUE] [flags]
//
// Stages: voxels, surface, ground, sun, layers (a ghost surface diagnostic),
// or all (the `pipeline` setting). Each reads the package's map.vgm (and sun
// reads ground.asc), so tweaking how a map is derived doesn't need the
// recording, e.g.
//
//   vg_map garden ground sun --max-slope 0.6
//
// Settings start from the package's config.yaml (what it was made with),
// then each --config file, --set and flag in order, and config.yaml is
// updated with what ran. map and build settings can't change here: they
// need a rebuild with vg_replay.

#include <algorithm>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "vg_core/config/mapping_config.hpp"
#include "vg_core/map_package.hpp"

namespace {

int usage(const vg::config::ParamSet& params) {
  std::cerr << "usage: vg_map <package_dir> <stage>... [settings]\n"
               "stages: voxels, surface, ground, sun, semantic, layers, or all (the pipeline setting)\n"
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
  const std::filesystem::path package_dir = args[0];
  std::vector<std::string> stages;
  std::size_t i = 1;
  for (; i < args.size() && args[i].rfind("--", 0) != 0; ++i) {
    stages.push_back(args[i]);
  }
  if (stages.empty()) {
    return usage(params);
  }

  try {
    const auto saved = package_dir / vg::package_file::kConfig;
    if (std::filesystem::exists(saved)) {
      params.load_yaml_file(saved.string());
    } else {
      std::cout << "note: " << saved.string()
                << " not found (an older package); starting from the defaults\n";
    }
    // map and build settings describe the saved 3D map; they can't change here.
    std::map<std::string, std::string> built;
    for (const std::string& path : params.paths()) {
      if (vg::config::is_build_setting(path)) {
        built[path] = params.get(path);
      }
    }
    params.apply_args({args.begin() + static_cast<std::ptrdiff_t>(i), args.end()});
    for (const auto& [path, value] : built) {
      if (params.get(path) != value) {
        throw vg::config::ConfigError(path + " is a build setting: rebuild with vg replay to "
                                      "change it");
      }
    }
    if (std::find(stages.begin(), stages.end(), "all") != stages.end()) {
      stages = config.pipeline;
    }
    config.validate();
    vg::config::check_stages(stages);
  } catch (const vg::config::ConfigError& e) {
    std::cerr << "vg_map: " << e.what() << "\n";
    return 2;
  }
  const auto wants = [&](const std::string& stage) {
    return std::find(stages.begin(), stages.end(), stage) != stages.end();
  };

  try {
    vg::MapPackage package(package_dir, std::cout);
    config.year = config.sun_options().year;
    const bool all_ran = vg::config::run_stages(package, config, stages);
    if (wants("ground") && !wants("sun") &&
        std::filesystem::exists(package.dir() / vg::package_file::kSunHours)) {
      std::cout << "note: the sun maps were made over the previous ground; run the sun stage "
                   "to update them\n";
    }
    vg::config::save_settings(params, package.dir() / vg::package_file::kConfig,
                              vg::config::kPackageHeader);
    return all_ran ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "vg_map: " << e.what() << "\n";
    return 1;
  }
}
