// vg_map: re-run derived stages (voxels, surface, ground, terrain, sky, sun, layers) of a map package from its saved 3D map.
//
//   vg_map <package_dir> <stage>... [--config FILE] [--set PATH=VALUE] [flags]
//
// Stages: voxels, surface, ground, terrain, sky, sun, layers (a ghost surface diagnostic),
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

#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "vg_core/config/jobs.hpp"
#include "vg_core/config/mapping_config.hpp"

namespace {

int usage(const vg::config::ParamSet& params) {
  std::cerr << "usage: vg_map <package_dir> <stage>... [settings]\n"
               "stages: voxels, surface, ground, terrain, sky, sun, semantic, layers, or all (the pipeline setting)\n"
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
    const bool all_ran = vg::config::rerun_stages(
        package_dir, stages, {args.begin() + static_cast<std::ptrdiff_t>(i), args.end()},
        std::cout);
    return all_ran ? 0 : 1;
  } catch (const vg::config::ConfigError& e) {
    std::cerr << "vg_map: " << e.what() << "\n";
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "vg_map: " << e.what() << "\n";
    return 1;
  }
}
