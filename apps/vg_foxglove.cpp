// vg_foxglove: make a recording ready to watch in Foxglove, optionally over its site's 3D model.
//
//   vg_foxglove <session.mcap> <out.mcap> [--site DIR --walk PACKAGE]
//               [--stride N] [--max-depth M] [--image-rate HZ] [--no-images]
//               [--keep-jumps]
//
// Writes the walk's poses as transforms and each depth image as a point cloud
// (see include/vg_core/io/foxglove_export.hpp). With --site and --walk (the
// map package made from this recording), the walk is placed in the site
// frame over the site's surface and canopy: from site.csv if the walk is one
// of its visits, else by registering it against the site there and then
// (shown even when the fit isn't convincing, so a bad one can be seen).
//
//   --stride N      every Nth depth pixel each way (default 2)
//   --max-depth M   leave out readings beyond M meters (default 3.5)
//   --image-rate HZ camera images a second, at most (default 10; 0: all)
//   --no-images     leave out the camera images (most of the file)
//   --keep-jumps    keep the tracker's pose jumps instead of undoing them

#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "vg_core/config/mapping_config.hpp"
#include "vg_core/io/foxglove_export.hpp"
#include "vg_core/site.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

int usage() {
  std::cerr << "usage: vg_foxglove <session.mcap> <out.mcap> [--site DIR --walk PACKAGE]\n"
               "                   [--stride N] [--max-depth M] [--image-rate HZ] [--no-images]\n"
               "                   [--keep-jumps]\n";
  return 2;
}

bool same_dir(const std::filesystem::path& a, const std::filesystem::path& b) {
  std::error_code error;
  return std::filesystem::equivalent(a, b, error);
}

// Where the walk's map sits in the site.
vg::MapTransform place_in_site(const vg::Site& site, const std::filesystem::path& walk) {
  for (const vg::SiteVisit& visit : site.visits()) {
    if (same_dir(site.package_dir(visit), walk)) {
      std::cout << "foxglove: " << walk.string() << " is a visit of the site; placed as in "
                << vg::site_file::kVisits << std::endl;
      return visit.site_from_map;
    }
  }
  std::cout << "foxglove: " << walk.string()
            << " isn't a visit of the site; lining it up with the site now" << std::endl;
  const vg::config::MappingConfig config;
  const vg::RegistrationResult fit = site.locate(walk, config.registration, std::cout);
  if (!fit.accepted) {
    std::cout << "foxglove: the fit isn't convincing (" << fit.problem << "); showing it anyway"
              << std::endl;
  }
  return fit.reference_from_map;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.size() < 2 || args[0] == "--help" || args[0] == "-h") {
    return usage();
  }
  const std::filesystem::path session = args[0];
  const std::filesystem::path out = args[1];
  std::optional<std::filesystem::path> site_dir;
  std::optional<std::filesystem::path> walk;
  vg::io::FoxgloveExportOptions options;
  try {
    for (std::size_t i = 2; i < args.size(); ++i) {
      const std::string& a = args[i];
      const auto value = [&]() -> const std::string& {
        if (i + 1 >= args.size()) {
          throw std::invalid_argument(a + " needs a value");
        }
        return args[++i];
      };
      if (a == "--site") {
        site_dir = value();
      } else if (a == "--walk") {
        walk = value();
      } else if (a == "--stride") {
        options.pixel_stride = std::stoi(value());
      } else if (a == "--max-depth") {
        options.max_depth = std::stod(value());
      } else if (a == "--image-rate") {
        options.image_rate = std::stod(value());
      } else if (a == "--no-images") {
        options.images = false;
      } else if (a == "--keep-jumps") {
        options.remove_pose_jumps = false;
      } else {
        std::cerr << "vg_foxglove: unknown option " << a << "\n";
        return usage();
      }
    }
    if (site_dir.has_value() != walk.has_value()) {
      std::cerr << "vg_foxglove: --site and --walk go together\n";
      return usage();
    }

    if (site_dir) {
      const vg::Site site(*site_dir);
      options.site_from_world = place_in_site(site, *walk);
      const vg::MapTransform& t = *options.site_from_world;
      std::cout << "foxglove: site_from_world yaw " << t.yaw * 180.0 / kPi << " deg, shift ("
                << t.translation.x() << ", " << t.translation.y() << ", " << t.translation.z()
                << ") m" << std::endl;
      options.site_model = site.model();
      if (options.site_model[0].mesh.empty()) {
        std::cout << "foxglove: the site has no current maps yet (vg site maps "
                  << site_dir->string() << "); no model shown" << std::endl;
      }
    }

    const vg::io::FoxgloveExportSummary s =
        vg::io::export_foxglove(session, out, options, std::cout);
    std::cout << "foxglove: wrote " << out.string() << ": " << s.poses << " poses, " << s.clouds
              << " point clouds (" << s.points << " points), " << s.images << " images";
    if (s.pose_jumps > 0) {
      std::cout << ", " << s.pose_jumps << " pose jumps undone";
    }
    std::cout << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "vg_foxglove: " << e.what() << "\n";
    return 1;
  }
}
