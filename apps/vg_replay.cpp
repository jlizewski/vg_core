// vg_replay: rebuild a map from a recorded capture session.
//
//   vg_replay <session.mcap> <heightmap.asc> [--rate R] [--voxel M] [--trunc M] [--cell M]
//
// --rate 1 replays in real time; the default 0 runs as fast as possible.

#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include "vg_core/height_map.hpp"
#include "vg_core/height_map_io.hpp"
#include "vg_core/io/session_player.hpp"
#include "vg_core/io/session_reader.hpp"
#include "vg_core/tsdf_volume.hpp"

namespace {

int usage() {
  std::cerr << "usage: vg_replay <session.mcap> <heightmap.asc> [--rate R] [--voxel M] "
               "[--trunc M] [--cell M]\n";
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
  vg::TsdfConfig config;
  double cell_size = 0.05;
  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    if (i + 1 >= argc) {
      return usage();
    }
    const double value = std::atof(argv[++i]);
    if (arg == "--rate") {
      playback.rate = value;
    } else if (arg == "--voxel") {
      config.voxel_size = value;
    } else if (arg == "--trunc") {
      config.truncation_distance = value;
    } else if (arg == "--cell") {
      cell_size = value;
    } else {
      return usage();
    }
  }

  try {
    vg::TsdfVolume volume(config);
    vg::io::SessionMapper mapper(volume);
    vg::io::SessionReader reader(session_path);
    const std::size_t messages = vg::io::play(
        reader,
        [&mapper](const vg::io::SessionMessage& m) {
          mapper.handle(m);
          return true;
        },
        playback);
    mapper.flush();

    const auto& stats = mapper.stats();
    std::cout << messages << " messages, " << stats.integrated << " depth frames integrated, "
              << stats.skipped_no_pose << " skipped without a pose, " << stats.skipped_tracking
              << " skipped while tracking was lost\n";

    const auto map = vg::make_height_map(volume.extract_surface_points(), cell_size);
    vg::save_height_map(map, output_path);
    std::cout << "height map " << map.width << " x " << map.height << " cells written to "
              << output_path << "\n";
  } catch (const std::exception& e) {
    std::cerr << "vg_replay: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
