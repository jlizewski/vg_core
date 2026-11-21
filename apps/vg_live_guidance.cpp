// vg_live_guidance: play a ground fill through the live map against a site's holes, as the phone
// would.
//
//   vg_live_guidance <site_dir> <session.mcap> [--every S] [--cell M] [--max-height M]
//                    [--max-depth M] [--min-weight W] [--cells-out cells.csv]
//
// The recording must share a tracking frame with one of the site's walks
// (recorded in the same run), which places it in the site. Its depth is fused
// as the phone fuses it while walking (keyframes live), and every S seconds
// of the recording (default 5) it prints how much of each hole the live map
// has seen, what's left, and where to go next. At the end, when each hole was
// done. --cells-out writes every hole cell (x, y in the site frame, hole
// rank, seen 0/1), to check the live estimate against what adding the walk to
// the site really did. See include/vg_core/live_guidance.hpp.

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "vg_core/config/mapping_config.hpp"
#include "vg_core/io/session_player.hpp"
#include "vg_core/io/session_reader.hpp"
#include "vg_core/live_guidance.hpp"
#include "vg_core/map_builder.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

int usage() {
  std::cerr << "usage: vg_live_guidance <site_dir> <session.mcap> [--every S] [--cell M]\n"
               "                        [--max-height M] [--max-depth M] [--min-weight W]\n"
               "                        [--cells-out FILE]\n"
               "  --every S         print every S seconds of the recording (default 5)\n"
               "  --cell M          live height map cell size, m (default 0.1524)\n"
               "  --max-height M    LiveGuidanceConfig::max_height\n"
               "  --max-depth M     LiveGuidanceConfig::max_depth\n"
               "  --min-weight W    fused weight a voxel needs (default: the stages' for depth\n"
               "                    with confidence)\n"
               "  --cells-out FILE  every hole cell and whether it was seen, as CSV\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    return usage();
  }
  double every = 5.0;
  double cell = 0.1524;
  vg::LiveGuidanceConfig live_config;
  std::string cells_out;
  // The fused weight a voxel needs, as the map's stages use for depth with
  // confidence (ARCore's): without it, faint far readings "see" holes the
  // walk didn't really see (measured on the yard: 335 of 807 hole cells
  // called seen that stayed unknown, against 13 with it).
  float min_weight =
      static_cast<float>(vg::config::MappingConfig().build.min_weight_with_confidence);
  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    if (i + 1 >= argc) {
      return usage();
    }
    if (arg == "--every") {
      every = std::stod(argv[++i]);
    } else if (arg == "--cell") {
      cell = std::stod(argv[++i]);
    } else if (arg == "--max-height") {
      live_config.max_height = std::stod(argv[++i]);
    } else if (arg == "--max-depth") {
      live_config.max_depth = std::stod(argv[++i]);
    } else if (arg == "--min-weight") {
      min_weight = std::stof(argv[++i]);
    } else if (arg == "--cells-out") {
      cells_out = argv[++i];
    } else {
      return usage();
    }
  }
  try {
    const std::filesystem::path site = argv[1];
    const std::filesystem::path session = argv[2];

    // The recording's tracking frame, from its session info.
    std::string frame;
    {
      vg::io::SessionReader reader(session);
      const auto first = reader.next();
      if (const auto* info = first ? std::get_if<vg::io::SessionInfo>(&*first) : nullptr) {
        frame = info->tracking_frame;
      }
    }
    vg::LiveGuidance live = vg::LiveGuidance::for_site(site, frame, live_config);
    std::cout << "site " << site.string() << ": " << live.targets().size() << " holes, "
              << std::fixed << std::setprecision(1) << live.unseen_area()
              << " m^2 unknown; the walk is in its tracking frame " << frame << "\n";

    // As the phone maps while walking: vg replay's settings (what the C API's
    // vg_map_builder_config_defaults gives the app), keyframes live.
    vg::MapBuilderConfig config = vg::config::MappingConfig().build_options().builder;
    config.height_cell_size = cell;
    config.keyframe_translation = 0.15;
    config.keyframe_rotation = 0.17;
    config.drop_non_keyframes = true;
    vg::MapBuilder builder(config);
    builder.set_min_weight(min_weight);
    vg::io::SessionMapper mapper(builder);
    vg::io::SessionReader reader(session);
    Eigen::Isometry3d body_from_camera = Eigen::Isometry3d::Identity();
    std::optional<vg::Timestamp> start;
    double next_print = 0.0;
    std::vector<double> done_at(live.targets().size(), -1.0);
    std::size_t depth_frames = 0;
    const auto report = [&](double t) {
      std::cout << "t=" << std::setw(5) << std::setprecision(1) << t << " s  unseen "
                << std::setprecision(1) << live.unseen_area() << " m^2 ";
      for (const vg::LiveTarget& target : live.targets()) {
        std::cout << "  " << target.rank << ": " << std::setprecision(0) << 100.0 * target.seen
                  << "%" << (target.done ? " done" : "");
      }
      if (const auto next = live.next()) {
        std::cout << "   next: hole " << next->rank << ", a spot " << std::setprecision(1)
                  << next->distance << " m away, turn " << std::setprecision(0)
                  << next->turn * 180.0 / kPi << " deg";
      } else if (live.done()) {
        std::cout << "   the ground is done";
      }
      std::cout << "\n";
    };
    while (const auto m = reader.next()) {
      mapper.handle(*m);
      if (const auto* tf = std::get_if<vg::io::StaticTransforms>(&*m)) {
        for (const auto& t : tf->transforms) {
          if (t.parent_frame == "body" && t.child_frame.rfind("cam/", 0) == 0) {
            body_from_camera = t.parent_from_child;
          }
        }
      }
      if (const auto* pose = std::get_if<vg::PoseStamped>(&*m)) {
        if (!start) {
          start = pose->timestamp;
        }
        live.set_camera_pose(pose->world_from_frame * body_from_camera);
      }
      if (std::holds_alternative<vg::io::DepthImage>(*m) && start) {
        // A few times a second on the phone; here every 5 depth frames.
        if (++depth_frames % 5 != 0) {
          continue;
        }
        live.update(builder.update().height_cells, cell);
        const double t =
            static_cast<double>(std::get<vg::io::DepthImage>(*m).frame.timestamp - *start) * 1e-9;
        for (std::size_t k = 0; k < live.targets().size(); ++k) {
          if (live.targets()[k].done && done_at[k] < 0.0) {
            done_at[k] = t;
          }
        }
        if (t >= next_print) {
          report(t);
          next_print = t + every;
        }
      }
    }
    builder.flush();
    live.update(builder.update().height_cells, cell);
    std::cout << "end:\n";
    for (std::size_t k = 0; k < live.targets().size(); ++k) {
      const vg::LiveTarget& target = live.targets()[k];
      std::cout << "  hole " << target.rank << " (" << std::setprecision(1) << target.area
                << " m^2): " << std::setprecision(0) << 100.0 * target.seen << "% seen";
      if (done_at[k] >= 0.0) {
        std::cout << ", done at t=" << std::setprecision(1) << done_at[k] << " s";
      }
      std::cout << "\n";
    }
    std::cout << std::setprecision(1) << "unseen at the end: " << live.unseen_area() << " m^2\n";
    if (!cells_out.empty()) {
      std::ofstream out(cells_out);
      out << "x,y,hole,seen\n" << std::setprecision(4);
      for (const auto& c : live.hole_cells()) {
        out << c.center.x() << ',' << c.center.y() << ','
            << live.targets()[static_cast<std::size_t>(c.target)].rank << ',' << (c.seen ? 1 : 0)
            << '\n';
      }
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "vg_live_guidance: " << e.what() << "\n";
    return 1;
  }
}
