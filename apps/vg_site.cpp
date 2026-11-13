// vg_site: line up walks of one place (map packages) in a shared site frame, and make its current maps.
//
//   vg_site add <site_dir> <package>... [--force] [--config FILE] [--set PATH=VALUE] [flags]
//   vg_site maps <site_dir>
//   vg_site list <site_dir>
//
// add registers each package against the visits so far (the first one sets
// the site's frame) and remakes current/. A package missing its terrain maps
// gets them first (surface, ground and terrain stages, with its config.yaml).
// A walk that doesn't line up convincingly isn't added; --force adds it anyway.
// Settings are the site section of config/mapping.yaml, e.g.
//
//   vg_site add garden-site garden-2026-11-09 --set site.min_inliers=0.5

#include <cmath>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "vg_core/config/mapping_config.hpp"
#include "vg_core/map_package.hpp"
#include "vg_core/site.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

int usage(const vg::config::ParamSet& params) {
  std::cerr << "usage: vg_site add <site_dir> <package>... [--force] [settings]\n"
               "       vg_site maps <site_dir>\n"
               "       vg_site list <site_dir>\n"
            << params.flags_help();
  return 2;
}

// Runs the stages a package needs for registration that it hasn't run yet.
void prepare(const std::filesystem::path& dir) {
  std::vector<std::string> stages;
  for (const auto& [file, stage] :
       {std::pair{vg::package_file::kSurface, "surface"},
        std::pair{vg::package_file::kGround, "ground"},
        std::pair{vg::package_file::kTerrain, "terrain"}}) {
    if (!stages.empty() || !std::filesystem::is_regular_file(dir / file)) {
      stages.push_back(stage);  // And everything after it.
    }
  }
  if (stages.empty()) {
    return;
  }
  vg::config::MappingConfig config;
  vg::config::ParamSet params = vg::config::bind(config);
  if (std::filesystem::exists(dir / vg::package_file::kConfig)) {
    params.load_yaml_file((dir / vg::package_file::kConfig).string());
  }
  vg::MapPackage package(dir, std::cout);
  vg::config::run_stages(package, config, stages);
  vg::config::save_settings(params, dir / vg::package_file::kConfig,
                            vg::config::kPackageHeader);
}

void list(const vg::Site& site) {
  std::cout << site.visits().size() << " visits, oldest first:\n";
  int n = 0;
  for (const vg::SiteVisit& v : site.visits()) {
    const auto& t = v.site_from_map;
    std::cout << std::setw(3) << ++n << "  " << (v.recorded.empty() ? "(time unknown)" : v.recorded)
              << "  " << v.package.generic_string() << "\n     yaw " << std::fixed
              << std::setprecision(1) << t.yaw * 180.0 / kPi << " deg, shift ("
              << std::setprecision(2) << t.translation.x() << ", " << t.translation.y() << ", "
              << t.translation.z() << ") m";
    if (v.fit) {
      std::cout << "; within 30 cm " << std::setprecision(0) << 100.0 * v.fit->inliers_30cm
                << "% of " << 100.0 * v.fit->overlap << "% overlap";
    } else {
      std::cout << "; sets the site frame";
    }
    std::cout << std::defaultfloat << std::setprecision(6) << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  vg::config::MappingConfig config;
  vg::config::ParamSet params = vg::config::bind(config);
  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.size() < 2 || args[0] == "--help" || args[0] == "-h") {
    return usage(params);
  }
  const std::string command = args[0];
  const std::filesystem::path site_dir = args[1];
  std::vector<std::filesystem::path> packages;
  std::vector<std::string> settings;
  bool force = false;
  for (std::size_t i = 2; i < args.size(); ++i) {
    if (args[i] == "--force") {
      force = true;
    } else if (args[i].rfind("--", 0) == 0) {
      settings.assign(args.begin() + static_cast<std::ptrdiff_t>(i), args.end());
      break;
    } else {
      packages.emplace_back(args[i]);
    }
  }
  if ((command == "add") == packages.empty() || (command != "add" && command != "maps" &&
                                                 command != "list")) {
    return usage(params);
  }
  try {
    params.apply_args(settings);
    config.validate();
  } catch (const vg::config::ConfigError& e) {
    std::cerr << "vg_site: " << e.what() << "\n";
    return 2;
  }

  try {
    vg::Site site(site_dir);
    if (command == "list") {
      list(site);
      return 0;
    }
    bool all_added = true;
    for (const auto& package : packages) {
      prepare(package);
      const vg::RegistrationResult result = site.add(package, config.registration, std::cout, force);
      all_added = all_added && (result.accepted || force);
    }
    site.make_maps(std::cout, config.sky.config);
    list(site);
    return all_added ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "vg_site: " << e.what() << "\n";
    return 1;
  }
}
