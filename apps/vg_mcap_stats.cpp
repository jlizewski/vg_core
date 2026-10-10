// vg_mcap_stats: report what a recording holds and how good its poses and depth look.
//
//   vg_mcap_stats <session.mcap> [--jumps N] [--edge-ratio R]
//
// Prints the messages per topic type, then for diagnosing a rough map:
//   - tracking: every tracking state change;
//   - poses: rate, path, gaps, and jumps (moves faster than a hand-held phone),
//     the usual cause of ghost (doubled) surfaces, with any tracking change
//     near each;
//   - depth: resolution and rate, how far the readings are, how many sit on
//     depth edges (flying pixels, a cause of floating blobs) by range, the
//     confidence when recorded, and how far each frame is from its pose.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "vg_core/diagnostics.hpp"
#include "vg_core/io/session_player.hpp"
#include "vg_core/io/session_reader.hpp"

namespace {

int usage() {
  std::cerr << "usage: vg_mcap_stats <session.mcap> [--jumps N] [--edge-ratio R]\n"
               "  --jumps N        pose jumps to list, largest first (default 15)\n"
               "  --edge-ratio R   a depth edge is a jump of more than R x depth to a neighbor "
               "(default 0.05)\n";
  return 2;
}

constexpr double kDegree = 3.14159265358979323846 / 180.0;

const char* state_name(vg::io::TrackingState state) {
  switch (state) {
    case vg::io::TrackingState::Normal:
      return "normal";
    case vg::io::TrackingState::Limited:
      return "limited";
    case vg::io::TrackingState::Lost:
      return "lost";
    default:
      return "unknown";
  }
}

double percent(std::size_t part, std::size_t whole) {
  return whole == 0 ? 0.0 : 100.0 * static_cast<double>(part) / static_cast<double>(whole);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    return usage();
  }
  const std::string session_path = argv[1];
  std::size_t max_jumps = 15;
  double edge_ratio = 0.05;
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (i + 1 >= argc) {
      return usage();
    }
    const double value = std::atof(argv[++i]);
    if (arg == "--jumps") {
      max_jumps = static_cast<std::size_t>(std::max(0.0, value));
    } else if (arg == "--edge-ratio") {
      edge_ratio = value;
    } else {
      return usage();
    }
  }

  try {
    vg::io::SessionReader reader(session_path);
    if (!reader.indexed()) {
      std::cout << "warning: no index (the recorder never closed it); reading what was written\n";
    }
    std::map<std::string, std::size_t> counts;
    std::vector<vg::PoseStamped> poses;
    std::vector<vg::io::TrackingStatus> tracking;
    std::vector<vg::Timestamp> depth_times;
    vg::DepthStats depth(edge_ratio);
    vg::Timestamp first = std::numeric_limits<vg::Timestamp>::max();
    std::size_t gps = 0;
    std::string device;

    vg::io::PlaybackOptions playback;
    playback.rate = 0.0;
    vg::io::play(
        reader,
        [&](const vg::io::SessionMessage& m) {
          // Times are reported from the first sensor reading (session info
          // and calibration may be stamped 0).
          if (std::holds_alternative<vg::PoseStamped>(m) ||
              std::holds_alternative<vg::io::DepthImage>(m) ||
              std::holds_alternative<vg::ImuSample>(m)) {
            first = std::min(first, vg::io::timestamp_of(m));
          }
          std::visit(
              [&](const auto& msg) {
                using T = std::decay_t<decltype(msg)>;
                if constexpr (std::is_same_v<T, vg::io::SessionInfo>) {
                  ++counts["session info"];
                  device = msg.platform + " " + msg.device_model;
                } else if constexpr (std::is_same_v<T, vg::io::CameraCalibration>) {
                  ++counts["camera calibration"];
                } else if constexpr (std::is_same_v<T, vg::io::StaticTransforms>) {
                  ++counts["static transforms"];
                } else if constexpr (std::is_same_v<T, vg::io::CameraImage>) {
                  ++counts["camera images"];
                } else if constexpr (std::is_same_v<T, vg::io::DepthImage>) {
                  ++counts["depth images"];
                  depth.add(msg.frame);
                  depth_times.push_back(msg.frame.timestamp);
                } else if constexpr (std::is_same_v<T, vg::PoseStamped>) {
                  ++counts["poses"];
                  poses.push_back(msg);
                } else if constexpr (std::is_same_v<T, vg::io::TrackingStatus>) {
                  ++counts["tracking status"];
                  tracking.push_back(msg);
                } else if constexpr (std::is_same_v<T, vg::ImuSample>) {
                  ++counts["imu samples"];
                } else if constexpr (std::is_same_v<T, vg::GpsFix>) {
                  ++counts["gps fixes"];
                  ++gps;
                }
              },
              m);
          return true;
        },
        playback);

    const auto t = [&](vg::Timestamp ts) { return static_cast<double>(ts - first) * 1e-9; };
    std::cout << std::fixed << std::setprecision(1);
    std::cout << session_path << (device.empty() ? "" : "  (" + device + ")") << "\n\nmessages\n";
    for (const auto& [name, n] : counts) {
      std::cout << "  " << std::setw(20) << std::left << name << std::right << n << "\n";
    }

    std::cout << "\ntracking state changes\n";
    if (tracking.empty()) {
      std::cout << "  none recorded\n";
    }
    for (const auto& status : tracking) {
      std::cout << "  t=" << std::setw(7) << t(status.timestamp) << " s  "
                << state_name(status.state) << (status.reason.empty() ? "" : "  " + status.reason)
                << "\n";
    }

    std::sort(poses.begin(), poses.end(), [](const vg::PoseStamped& a, const vg::PoseStamped& b) {
      return a.timestamp < b.timestamp;
    });
    const auto track = vg::analyze_pose_track(poses);
    std::cout << "\nposes\n  " << track.poses << " over " << track.duration << " s ("
              << (track.duration > 0 ? static_cast<double>(track.poses) / track.duration : 0.0)
              << " Hz), path " << track.path_length << " m, " << track.gaps
              << " gaps (longest " << track.longest_gap << " s)\n  " << track.jumps.size()
              << " jumps (faster than 2 m/s or 3 rad/s)";
    std::cout << (track.jumps.empty() ? "\n" : ", largest first:\n");
    for (std::size_t j = 0; j < std::min(max_jumps, track.jumps.size()); ++j) {
      const auto& jump = track.jumps[j];
      std::cout << std::setprecision(3) << "  t=" << std::setw(7) << t(jump.timestamp) << " s  "
                << std::setw(6) << jump.translation * 100.0 << " cm  " << std::setw(6)
                << jump.rotation / kDegree << " deg  in " << jump.dt * 1000.0 << " ms";
      for (const auto& status : tracking) {
        if (std::llabs(status.timestamp - jump.timestamp) < 2'000'000'000) {
          std::cout << "  (tracking " << state_name(status.state) << " at t="
                    << t(status.timestamp) << ")";
        }
      }
      std::cout << "\n";
    }
    std::cout << std::setprecision(1);

    std::cout << "\ndepth\n";
    if (depth.frames() == 0) {
      std::cout << "  none: no /cam/<name>/depth topic, so nothing can be mapped\n";
    } else {
      const double span = depth_times.size() > 1
                              ? static_cast<double>(depth_times.back() - depth_times.front()) * 1e-9
                              : 0.0;
      std::cout << "  " << depth.frames() << " frames of " << depth.width() << " x "
                << depth.height() << " ("
                << (span > 0 ? static_cast<double>(depth.frames() - 1) / span : 0.0) << " Hz), "
                << percent(depth.valid(), depth.pixels()) << "% of pixels have a reading\n";
      std::cout << "  range      readings   on edges (>" << edge_ratio * 100 << "% jump)\n";
      for (int b = 0; b < vg::DepthStats::kBands; ++b) {
        const auto band = static_cast<std::size_t>(b);
        const std::string label = b + 1 < vg::DepthStats::kBands
                                      ? std::to_string(b) + "-" + std::to_string(b + 1) + " m"
                                      : std::to_string(b) + "+ m";
        std::cout << "  " << std::setw(6) << std::left << label << std::right << std::setw(10)
                  << percent(depth.valid_by_range()[band], depth.valid()) << "%  " << std::setw(10)
                  << percent(depth.edges_by_range()[band], depth.valid_by_range()[band]) << "%\n";
      }
      if (depth.frames_with_confidence() == 0) {
        std::cout << "  no confidence recorded (smoothed depth, e.g. ARCore "
                     "acquireDepthImage16Bits)\n";
      } else {
        const auto& c = depth.by_confidence();
        std::cout << "  confidence in " << depth.frames_with_confidence() << " frames: 0-63 "
                  << percent(c[0], depth.valid()) << "%, 64-127 " << percent(c[1], depth.valid())
                  << "%, 128-191 " << percent(c[2], depth.valid()) << "%, 192-255 "
                  << percent(c[3], depth.valid()) << "%\n";
      }

      // How far each depth frame is from the closest pose.
      std::vector<double> offsets;
      for (const vg::Timestamp ts : depth_times) {
        const auto it = std::lower_bound(
            poses.begin(), poses.end(), ts,
            [](const vg::PoseStamped& p, vg::Timestamp value) { return p.timestamp < value; });
        double best = std::numeric_limits<double>::infinity();
        if (it != poses.end()) {
          best = std::min(best, std::abs(static_cast<double>(it->timestamp - ts)));
        }
        if (it != poses.begin()) {
          best = std::min(best, std::abs(static_cast<double>(std::prev(it)->timestamp - ts)));
        }
        offsets.push_back(best * 1e-6);
      }
      std::sort(offsets.begin(), offsets.end());
      if (!offsets.empty() && std::isfinite(offsets.front())) {
        std::cout << "  closest pose: median " << offsets[offsets.size() / 2] << " ms, max "
                  << offsets.back() << " ms\n";
      }
    }
    std::cout << "\ngps\n  " << gps << " fixes\n";
  } catch (const std::exception& e) {
    std::cerr << "vg_mcap_stats: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
