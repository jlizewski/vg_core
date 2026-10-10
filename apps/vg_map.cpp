// vg_map: re-run derived stages (surface, ground, sun) of a map package from its saved 3D map.
//
//   vg_map <package_dir> <stage>... [options]
//
// Stages: surface, ground, sun, or all (the three, in that order). Each reads
// the package's map.vgm (and sun reads ground.asc), so tweaking how a map is
// derived doesn't need the recording, e.g.
//
//   vg_map garden ground sun --max-slope 0.6
//
// Settings not given are the defaults, not what the stage last ran with;
// manifest.json records what each stage last ran with.

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "package_options.hpp"
#include "vg_core/map_package.hpp"

namespace {

int usage() {
  std::cerr << "usage: vg_map <package_dir> <stage>... [options]\n"
               "stages: surface, ground, sun, all\n"
               "  --cell M               cell size of the 2D maps (default the package's)\n"
            << vg::app::kStageOptionsHelp;
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    return usage();
  }
  const std::string package_dir = argv[1];
  const std::vector<std::string> known = {"surface", "ground", "sun"};
  std::vector<std::string> requested;
  vg::app::StageOptions stages;

  int i = 2;
  for (; i < argc && std::string(argv[i]).rfind("--", 0) != 0; ++i) {
    const std::string stage = argv[i];
    if (stage == "all") {
      requested.insert(requested.end(), known.begin(), known.end());
    } else if (std::find(known.begin(), known.end(), stage) != known.end()) {
      requested.push_back(stage);
    } else {
      std::cerr << "vg_map: unknown stage " << stage << "\n";
      return usage();
    }
  }
  if (requested.empty()) {
    return usage();
  }
  for (; i < argc; ++i) {
    const std::string arg = argv[i];
    if (i + 1 >= argc) {
      return usage();
    }
    const char* text = argv[++i];
    if (arg == "--cell") {
      stages.set_cell_size(std::atof(text));
    } else if (!stages.parse(arg, text)) {
      return usage();
    }
  }
  const auto wants = [&](const std::string& stage) {
    return std::find(requested.begin(), requested.end(), stage) != requested.end();
  };

  try {
    vg::MapPackage package(package_dir, std::cout);
    if (wants("surface")) {
      package.make_surface(stages.surface);
    }
    if (wants("ground")) {
      package.make_ground(stages.ground);
      if (!wants("sun") &&
          std::filesystem::exists(package.dir() / vg::package_file::kSunHours)) {
        std::cout << "note: the sun maps were made over the previous ground; run the sun stage "
                     "to update them\n";
      }
    }
    if (wants("sun") && !package.make_sun(stages.sun)) {
      return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << "vg_map: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
