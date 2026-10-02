// vg_replay: rebuild a map from a recorded capture session.
//
//   vg_replay <session.mcap> <heightmap.asc> [--voxels map.ply] [--ground ground.asc]
//             [--objects objects.ply] [--rate R] [--voxel M] [--trunc M] [--cell M]
//
// --voxels also writes the 3D voxel map as a PLY mesh of cubes.
// --ground writes the ground height map, with everything on the ground removed
// and the holes it leaves patched; --objects writes those removed voxels.
// --rate 1 replays in real time, printing the map's progress as it grows;
// the default 0 runs as fast as possible.

#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <variant>
#include <vector>

#include "vg_core/ground_segmentation.hpp"
#include "vg_core/height_map_io.hpp"
#include "vg_core/io/session_player.hpp"
#include "vg_core/io/session_reader.hpp"
#include "vg_core/map_builder.hpp"
#include "vg_core/voxel_map_io.hpp"

namespace {

int usage() {
  std::cerr << "usage: vg_replay <session.mcap> <heightmap.asc> [--voxels map.ply] "
               "[--ground ground.asc] [--objects objects.ply] [--rate R] [--voxel M] [--trunc M] "
               "[--cell M]\n";
  return 2;
}

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
    const double value = std::atof(argv[++i]);
    if (arg == "--rate") {
      playback.rate = value;
    } else if (arg == "--voxel") {
      config.tsdf.voxel_size = value;
    } else if (arg == "--trunc") {
      config.tsdf.truncation_distance = value;
    } else if (arg == "--cell") {
      config.height_cell_size = value;
    } else {
      return usage();
    }
  }

  try {
    vg::MapBuilder builder(config);
    vg::io::SessionMapper mapper(builder);
    vg::io::SessionReader reader(session_path);
    const bool live = playback.rate > 0.0;
    std::size_t depth_frames = 0;
    const std::size_t messages = vg::io::play(
        reader,
        [&](const vg::io::SessionMessage& m) {
          mapper.handle(m);
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

    const auto map = builder.height_map();
    vg::save_height_map(map, output_path);
    std::cout << "height map " << map.width << " x " << map.height << " cells written to "
              << output_path << "\n";

    if (!voxels_path.empty()) {
      const auto voxels = builder.volume().occupied_voxels();
      vg::save_voxels_ply(voxels, builder.volume().config().voxel_size, voxels_path);
      std::cout << "voxel map " << voxels.size() << " voxels written to " << voxels_path << "\n";
    }

    if (!ground_path.empty() || !objects_path.empty()) {
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
    }
  } catch (const std::exception& e) {
    std::cerr << "vg_replay: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
