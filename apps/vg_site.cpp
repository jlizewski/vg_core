// vg_site: line up walks of one place (map packages) in a shared site frame, and make its current maps.
//
//   vg_site add <site_dir> <package>... [--force] [--config FILE] [--set PATH=VALUE] [flags]
//   vg_site maps <site_dir>
//   vg_site ground <site_dir>
//   vg_site edit <site_dir> ground|not_ground|boundary "x y;x y;..." [note]
//   vg_site list <site_dir>
//
// edit adds a correction (an area that is or isn't ground, or a line the
// boundary runs along instead, in the site frame) and remakes the layers. Edits
// are kept in layers/ground/edits.csv and layers/boundary/edits.csv and win
// over what walks measure; walks recorded later that disagree are reported.
//
// ground builds the site's ground layer (layers/ground/, see
// vg_core/ground_layer.hpp): every visit's ground combined, with the edits in
// layers/ground/edits.csv applied; and its feature elements
// (layers/features/, vg_core/features.hpp): posts, lines and blocks matched
// across visits. Settings: the ground_layer and features sections.
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

#include "vg_core/config/jobs.hpp"
#include "vg_core/config/mapping_config.hpp"
#include "vg_core/site.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

int usage(const vg::config::ParamSet& params) {
  std::cerr << "usage: vg_site add <site_dir> <package>... [--force] [settings]\n"
               "       vg_site maps <site_dir>\n"
               "       vg_site ground <site_dir>\n"
               "       vg_site edit <site_dir> ground|not_ground|boundary \"x y;x y;...\" [note]\n"
               "       vg_site delete-edit <site_dir> <edit_id>\n"
               "       vg_site list <site_dir>\n"
            << params.flags_help();
  return 2;
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
  if (command == "delete-edit") {
    if (args.size() != 3) {
      return usage(params);
    }
    try {
      vg::config::delete_site_edit(site_dir, args[2], std::cout);
      return 0;
    } catch (const vg::config::ConfigError& e) {
      std::cerr << "vg_site: " << e.what() << "\n";
      return 2;
    } catch (const std::exception& e) {
      std::cerr << "vg_site: " << e.what() << "\n";
      return 1;
    }
  }
  if (command == "edit") {
    if (args.size() < 4) {
      return usage(params);
    }
    std::string note;
    for (std::size_t i = 4; i < args.size(); ++i) {
      note += (i > 4 ? " " : "") + args[i];
    }
    try {
      vg::config::add_site_edit(site_dir, args[2], args[3], note, std::cout);
      return 0;
    } catch (const vg::config::ConfigError& e) {
      std::cerr << "vg_site: " << e.what() << "\n";
      return 2;
    } catch (const std::exception& e) {
      std::cerr << "vg_site: " << e.what() << "\n";
      return 1;
    }
  }
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
  if ((command == "add") == packages.empty() ||
      (command != "add" && command != "maps" && command != "ground" && command != "list")) {
    return usage(params);
  }
  try {
    bool all_added = true;
    if (command == "add") {
      all_added = vg::config::add_to_site(site_dir, packages, force, settings, std::cout);
    } else if (command == "maps") {
      vg::config::make_site_maps(site_dir, settings, std::cout);
    } else if (command == "ground") {
      vg::config::make_site_ground(site_dir, settings, std::cout);
    }
    list(vg::Site(site_dir));
    return all_added ? 0 : 1;
  } catch (const vg::config::ConfigError& e) {
    std::cerr << "vg_site: " << e.what() << "\n";
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "vg_site: " << e.what() << "\n";
    return 1;
  }
}
