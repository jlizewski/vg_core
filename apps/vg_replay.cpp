// vg_replay: build the full map package (3D map, surface, ground, objects, sun) from a recording.
//
//   vg_replay <session.mcap> <package_dir> [options]
//
// Replays the session into the 3D voxel map and derives every other map from
// it, writing the package described in vg_core/map_package.hpp:
//
//   map.vgm voxels.ply surface.asc ground.asc objects.ply sun_hours.asc
//   sun_energy.asc manifest.json
//
// Any earlier package in package_dir is replaced. To re-run a derived stage
// with new settings without replaying, use vg_map on the package.
//
// The sun stage needs to know where the map is: from the session's GPS, or
// --lat/--lon. Without either it's skipped and the rest is still written.

#include <cmath>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

#include "package_options.hpp"
#include "vg_core/io/map_build.hpp"
#include "vg_core/map_package.hpp"

namespace {

constexpr double kInch = 0.0254;
constexpr double kDegree = 3.14159265358979323846 / 180.0;

int usage() {
  std::cerr
      << "usage: vg_replay <session.mcap> <package_dir> [options]\n"
         "build stage:\n"
         "  --resolution M         cell size of the 2D maps (default 0.1524 = 6 in)\n"
         "  --voxel M              3D voxel size (default resolution / 2)\n"
         "  --trunc M              TSDF truncation distance (default 3 voxels)\n"
         "  --max-depth M          ignore depth readings farther than this (default 5)\n"
         "  --stride N             place blocks from every Nth depth pixel (default 2)\n"
         "  --keyframe-dist M      fuse a frame only after moving this far (default 0.05)\n"
         "  --keyframe-angle DEG   ... or turning this much (default 5); both 0 fuse every "
         "frame\n"
         "  --rate R               playback speed: 0 as fast as possible (default), 1 real time\n"
      << vg::app::kStageOptionsHelp;
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    return usage();
  }
  const std::string session_path = argv[1];
  const std::string package_dir = argv[2];

  vg::io::BuildOptions build;
  build.builder.tsdf.pixel_stride = 2;
  build.builder.tsdf.threads = 0;
  build.builder.keyframe_translation = 0.05;
  build.builder.keyframe_rotation = 5.0 * kDegree;
  build.builder.drop_non_keyframes = true;
  double resolution = 6.0 * kInch;
  std::optional<double> voxel;
  std::optional<double> trunc;
  vg::app::StageOptions stages;

  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    if (i + 1 >= argc) {
      return usage();
    }
    const char* text = argv[++i];
    const double value = std::atof(text);
    if (arg == "--resolution") {
      resolution = value;
    } else if (arg == "--voxel") {
      voxel = value;
    } else if (arg == "--trunc") {
      trunc = value;
    } else if (arg == "--max-depth") {
      build.builder.tsdf.max_depth = value;
    } else if (arg == "--stride") {
      build.builder.tsdf.pixel_stride = static_cast<int>(value);
    } else if (arg == "--keyframe-dist") {
      build.builder.keyframe_translation = value;
    } else if (arg == "--keyframe-angle") {
      build.builder.keyframe_rotation = value * kDegree;
    } else if (arg == "--rate") {
      build.rate = value;
    } else if (!stages.parse(arg, text)) {
      return usage();
    }
  }
  if (!(resolution > 0.0)) {
    std::cerr << "vg_replay: --resolution must be positive\n";
    return 2;
  }
  // Two voxels per cell edge keeps thin things (stems, posts, edging) in the
  // 3D map while the 2D maps stay at the resolution asked for.
  build.builder.tsdf.voxel_size = voxel.value_or(resolution / 2.0);
  build.builder.tsdf.truncation_distance = trunc.value_or(3.0 * build.builder.tsdf.voxel_size);
  build.cell_size = resolution;

  try {
    vg::io::build_map_package(session_path, package_dir, build, std::cout);
    vg::MapPackage package(package_dir, std::cout);
    package.make_surface(stages.surface);
    package.make_ground(stages.ground);
    package.make_sun(stages.sun);
    std::cout << "map package written to " << package_dir << "\n";
  } catch (const std::exception& e) {
    std::cerr << "vg_replay: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
