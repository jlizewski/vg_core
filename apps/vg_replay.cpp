// vg_replay: rebuild a map from a recorded capture session.
//
//   vg_replay <session.mcap> <heightmap.asc> [--voxels map.ply] [--ground ground.asc]
//             [--objects objects.ply] [--sun-map sun.asc] [--sun-energy energy.asc]
//             [--year Y] [--sun-step MIN] [--lat DEG --lon DEG] [--x-bearing DEG]
//             [--rate R] [--voxel M] [--trunc M] [--cell M]
//
// --voxels also writes the 3D voxel map as a PLY mesh of cubes.
// --ground writes the ground height map, with everything on the ground removed
// and the holes it leaves patched; --objects writes those removed voxels.
// --sun-map writes the hours of direct sun each ground cell gets over a year
// (default the current one) at --sun-step minute steps (default 60), raycast
// through the voxel map; --sun-energy writes the clear-sky direct energy in
// kWh/m^2. The map is placed on the Earth from the session's GPS and poses;
// --lat/--lon override the location and --x-bearing sets the compass bearing
// of the map's +x axis (degrees clockwise from north) when GPS can't.
// --rate 1 replays in real time, printing the map's progress as it grows;
// the default 0 runs as fast as possible.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include "vg_core/ground_segmentation.hpp"
#include "vg_core/height_map_io.hpp"
#include "vg_core/io/session_player.hpp"
#include "vg_core/io/session_reader.hpp"
#include "vg_core/map_builder.hpp"
#include "vg_core/solar.hpp"
#include "vg_core/sun_map.hpp"
#include "vg_core/voxel_map_io.hpp"

namespace {

int usage() {
  std::cerr << "usage: vg_replay <session.mcap> <heightmap.asc> [--voxels map.ply] "
               "[--ground ground.asc] [--objects objects.ply] [--sun-map sun.asc] "
               "[--sun-energy energy.asc] [--year Y] [--sun-step MIN] [--lat DEG --lon DEG] "
               "[--x-bearing DEG] [--rate R] [--voxel M] [--trunc M] [--cell M]\n";
  return 2;
}

constexpr double kPi = 3.14159265358979323846;

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    return usage();
  }
  const std::string session_path = argv[1];
  const std::string output_path = argv[2];
  vg::io::PlaybackOptions playback;
  playback.rate = 0.0;
  vg::MapBuilderConfig config;
  std::string voxels_path;
  std::string ground_path;
  std::string objects_path;
  std::string sun_path;
  std::string energy_path;
  const double now = static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(
                                             std::chrono::system_clock::now().time_since_epoch())
                                             .count());
  int year = vg::utc_year(now);
  double sun_step_minutes = 60.0;
  double latitude = std::nan("");
  double longitude = std::nan("");
  double x_bearing = std::nan("");
  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    if (i + 1 >= argc) {
      return usage();
    }
    if (arg == "--voxels") {
      voxels_path = argv[++i];
      continue;
    }
    if (arg == "--ground") {
      ground_path = argv[++i];
      continue;
    }
    if (arg == "--objects") {
      objects_path = argv[++i];
      continue;
    }
    if (arg == "--sun-map") {
      sun_path = argv[++i];
      continue;
    }
    if (arg == "--sun-energy") {
      energy_path = argv[++i];
      continue;
    }
    const double value = std::atof(argv[++i]);
    if (arg == "--rate") {
      playback.rate = value;
    } else if (arg == "--voxel") {
      config.tsdf.voxel_size = value;
    } else if (arg == "--trunc") {
      config.tsdf.truncation_distance = value;
    } else if (arg == "--cell") {
      config.height_cell_size = value;
    } else if (arg == "--year") {
      year = static_cast<int>(value);
    } else if (arg == "--sun-step") {
      sun_step_minutes = value;
    } else if (arg == "--lat") {
      latitude = value;
    } else if (arg == "--lon") {
      longitude = value;
    } else if (arg == "--x-bearing") {
      x_bearing = value;
    } else {
      return usage();
    }
  }

  try {
    vg::MapBuilder builder(config);
    vg::io::SessionMapper mapper(builder);
    vg::io::SessionReader reader(session_path);
    if (!reader.indexed()) {
      std::cerr << "warning: " << session_path
                << " has no index (the recorder never closed it); reading what was written\n";
    }
    const bool live = playback.rate > 0.0;
    std::size_t depth_frames = 0;
    std::vector<vg::GpsFix> fixes;
    std::vector<vg::PoseStamped> poses;
    const std::size_t messages = vg::io::play(
        reader,
        [&](const vg::io::SessionMessage& m) {
          mapper.handle(m);
          if (const auto* fix = std::get_if<vg::GpsFix>(&m)) {
            fixes.push_back(*fix);
          } else if (const auto* pose = std::get_if<vg::PoseStamped>(&m)) {
            poses.push_back(*pose);
          }
          // Refresh the live height map every 10 depth frames, as an app would
          // a few times a second.
          if (std::holds_alternative<vg::io::DepthImage>(m) && ++depth_frames % 10 == 0) {
            const auto changed = builder.update().height_cells;
            if (live) {
              std::cout << "t=" << static_cast<double>(vg::io::timestamp_of(m)) * 1e-9 << " s  "
                        << builder.stats().integrated << " frames  " << builder.height_map_cells()
                        << " cells mapped  " << changed.size() << " changed\n";
            }
          }
          return true;
        },
        playback);
    builder.flush();
    builder.update();

    const auto& stats = builder.stats();
    std::cout << messages << " messages, " << stats.integrated << " depth frames integrated, "
              << stats.skipped_no_pose << " skipped without a pose, " << stats.skipped_tracking
              << " skipped while tracking was lost\n";

    if (depth_frames == 0) {
      std::cerr << "warning: no depth images in " << session_path
                << " (no /cam/<name>/depth topic), so the map is empty\n";
    }

    const auto map = builder.height_map();
    vg::save_height_map(map, output_path);
    std::cout << "height map " << map.width << " x " << map.height << " cells written to "
              << output_path << "\n";

    if (!voxels_path.empty()) {
      const auto voxels = builder.volume().occupied_voxels();
      vg::save_voxels_ply(voxels, builder.volume().config().voxel_size, voxels_path);
      std::cout << "voxel map " << voxels.size() << " voxels written to " << voxels_path << "\n";
    }

    const bool want_sun = !sun_path.empty() || !energy_path.empty();
    if (!ground_path.empty() || !objects_path.empty() || want_sun) {
      vg::GroundConfig ground_config;
      ground_config.cell_size = config.height_cell_size;
      const auto split = vg::segment_ground(builder.volume(), ground_config);
      std::size_t grounded = 0;
      std::size_t object_voxels = 0;
      for (const auto& segment : split.segments) {
        grounded += segment.grounded ? 1 : 0;
        object_voxels += segment.voxels.size();
      }
      std::cout << split.ground_voxels.size() << " ground voxels, " << split.segments.size()
                << " objects (" << grounded << " standing on the ground, "
                << split.segments.size() - grounded << " overhanging)\n";
      if (!ground_path.empty()) {
        const auto& ground = split.ground.heights;
        vg::save_height_map(ground, ground_path);
        std::size_t filled = 0;
        for (const auto f : split.ground.filled) {
          filled += f;
        }
        std::cout << "ground map " << ground.width << " x " << ground.height << " cells (" << filled
                  << " patched) written to " << ground_path << "\n";
      }
      if (!objects_path.empty()) {
        std::vector<Eigen::Vector3i> objects;
        objects.reserve(object_voxels);
        for (const auto& segment : split.segments) {
          objects.insert(objects.end(), segment.voxels.begin(), segment.voxels.end());
        }
        vg::save_voxels_ply(objects, builder.volume().config().voxel_size, objects_path);
        std::cout << "objects " << objects.size() << " voxels written to " << objects_path << "\n";
      }
      if (want_sun) {
        auto geo = vg::estimate_geo_reference(fixes, poses);
        if (!std::isnan(latitude) && !std::isnan(longitude)) {
          if (!geo) {
            geo.emplace();
            geo->heading_sigma = std::numeric_limits<double>::infinity();
          }
          geo->latitude = latitude;
          geo->longitude = longitude;
        }
        if (!geo) {
          std::cerr << "vg_replay: the session has no GPS fixes; pass --lat and --lon for the "
                       "sun map\n";
          return 1;
        }
        if (!std::isnan(x_bearing)) {
          geo->heading = (90.0 - x_bearing) * kPi / 180.0;
          geo->heading_sigma = 0.0;
        }
        const auto precision = std::cout.precision(9);
        std::cout << "map at " << geo->latitude << ", " << geo->longitude;
        std::cout.precision(precision);
        std::cout << " (" << fixes.size() << " GPS fixes), +x axis bearing "
                  << std::fmod(450.0 - geo->heading * 180.0 / kPi, 360.0) << " deg";
        if (std::isinf(geo->heading_sigma)) {
          std::cout << " (unknown: no pose track to compare the GPS with; set --x-bearing)\n";
        } else {
          std::cout << " +/- " << geo->heading_sigma * 180.0 / kPi << " deg\n";
          if (geo->heading_sigma > 10.0 * kPi / 180.0) {
            std::cout << "warning: the map's heading is poorly known from GPS (the walk was short "
                         "for the GPS accuracy), so shadows may point the wrong way; set "
                         "--x-bearing if you know it\n";
          }
        }

        const auto sun_config = vg::sun_map_config_for_year(year, sun_step_minutes * 60.0);
        // Everything mapped casts shadows: objects, and the ground itself on slopes.
        const auto occluders = builder.volume().occupied_voxels();
        const auto start = std::chrono::steady_clock::now();
        const auto sun = vg::compute_sun_map(
            split.ground, occluders, builder.volume().config().voxel_size, *geo, sun_config);
        const double seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::cout << "sun map for " << year << ": " << sun.sun_steps << " steps with the sun up ("
                  << sun.daylight_hours << " h), " << seconds << " s\n";
        if (!sun_path.empty()) {
          vg::save_height_map(sun.sun_hours, sun_path);
          std::cout << "sun hours written to " << sun_path << "\n";
        }
        if (!energy_path.empty()) {
          vg::save_height_map(sun.irradiation, energy_path);
          std::cout << "sun energy (kWh/m^2) written to " << energy_path << "\n";
        }
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "vg_replay: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
