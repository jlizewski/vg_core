#include "vg_core/io/map_build.hpp"

#include <chrono>
#include <fstream>
#include <locale>
#include <optional>
#include <variant>
#include <utility>
#include <vector>

#include "vg_core/geo_reference.hpp"
#include "vg_core/io/session_player.hpp"
#include "vg_core/io/session_reader.hpp"
#include "vg_core/map_package.hpp"
#include "vg_core/pose_jumps.hpp"

namespace vg::io {

void build_map_package(const std::filesystem::path& session, const std::filesystem::path& dir,
                       const BuildOptions& options, std::ostream& log) {
  const auto start = std::chrono::steady_clock::now();
  MapBuilderConfig config = options.builder;
  config.height_cell_size = options.cell_size;
  MapBuilder builder(config);
  SessionMapper mapper(builder);
  SessionReader reader(session);
  if (!reader.indexed()) {
    log << "warning: " << session.string()
        << " has no index (the recorder never closed it); reading what was written\n";
  }

  PlaybackOptions playback;
  playback.rate = options.rate;
  const bool live = options.rate > 0.0;
  std::size_t depth_frames = 0;
  std::vector<GpsFix> fixes;
  std::vector<PoseStamped> poses;
  std::optional<Timestamp> first_reading;
  PoseJumpFilter jumps;
  std::vector<std::pair<double, Eigen::Vector3d>> camera_path;
  std::size_t outside_window = 0;
  std::size_t frames_with_confidence = 0;
  const std::size_t messages = play(
      reader,
      [&](const SessionMessage& m) {
        const bool sensor = std::holds_alternative<PoseStamped>(m) ||
                            std::holds_alternative<DepthImage>(m) ||
                            std::holds_alternative<ImuSample>(m);
        if (sensor && !first_reading) {
          first_reading = timestamp_of(m);
        }
        if (std::holds_alternative<DepthImage>(m) && first_reading && !poses.empty()) {
          camera_path.emplace_back(
              static_cast<double>(timestamp_of(m) - *first_reading) * 1e-9,
              poses.back().world_from_frame.translation());
        }
        if (std::holds_alternative<DepthImage>(m) && first_reading) {
          const double t = static_cast<double>(timestamp_of(m) - *first_reading) * 1e-9;
          if (t < options.from || t > options.to) {
            ++outside_window;
            return true;
          }
        }
        if (const auto* pose = std::get_if<PoseStamped>(&m)) {
          const PoseStamped corrected = options.remove_pose_jumps ? jumps.correct(*pose) : *pose;
          mapper.handle(SessionMessage(corrected));
          poses.push_back(corrected);
          return true;
        }
        mapper.handle(m);
        if (const auto* fix = std::get_if<GpsFix>(&m)) {
          fixes.push_back(*fix);
        } else if (const auto* depth = std::get_if<DepthImage>(&m)) {
          ++depth_frames;
          frames_with_confidence += depth->frame.confidence.empty() ? 0 : 1;
          // When watching it replay live, refresh the live height map every 10
          // depth frames, as an app would a few times a second.
          if (live && depth_frames % 10 == 0) {
            const auto changed = builder.update().height_cells;
            log << "t=" << static_cast<double>(timestamp_of(m)) * 1e-9 << " s  "
                << builder.stats().integrated << " frames  " << builder.height_map_cells()
                << " cells mapped  " << changed.size() << " changed\n";
          }
        }
        return true;
      },
      playback);
  builder.flush();

  const auto& stats = builder.stats();
  const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  log << messages << " messages, " << depth_frames << " depth frames: " << stats.integrated
      << " integrated, " << stats.dropped << " dropped (not keyframes), " << stats.skipped_no_pose
      << " without a pose, " << stats.skipped_tracking << " while tracking was lost (" << seconds
      << " s)\n";
  if (jumps.jumps() > 0) {
    log << "undid " << jumps.jumps() << " tracking jumps (the tracker snapping its pose)\n";
  }
  if (outside_window > 0) {
    log << outside_window << " depth frames outside " << options.from << " to " << options.to
        << " s left out\n";
  }
  if (depth_frames == 0) {
    log << "warning: no depth images in " << session.string()
        << " (no /cam/<name>/depth topic), so the map is empty\n";
  }

  // Most frames having confidence means its weighting is in play.
  const bool confidence = frames_with_confidence * 2 > depth_frames;
  const float min_weight = options.min_weight.value_or(
      confidence ? options.min_weight_with_confidence : options.min_weight_without_confidence);
  builder.set_min_weight(min_weight);
  log << (confidence ? "depth has confidence" : "depth has no confidence")
      << ", so voxels need a fused weight of " << min_weight
      << (options.min_weight ? "" : " (set --min-weight to change)") << '\n';

  MapMetadata metadata;
  metadata.session = session.filename().string();
  metadata.cell_size = options.cell_size;
  metadata.geo = estimate_geo_reference(fixes, poses);
  metadata.frames_integrated = stats.integrated;
  if (!metadata.geo) {
    log << "warning: no GPS fixes in the session, so the map isn't placed on the Earth\n";
  }

  const TsdfConfig& tsdf = config.tsdf;
  StageSettings settings;
  settings.add("session", metadata.session)
      .add("voxel_size_m", tsdf.voxel_size)
      .add("truncation_m", tsdf.truncation_distance)
      .add("min_depth_m", tsdf.min_depth)
      .add("max_depth_m", tsdf.max_depth)
      .add("min_confidence", static_cast<int>(tsdf.min_confidence))
      .add("edge_ratio", tsdf.edge_ratio)
      .add("max_incidence_deg", tsdf.max_incidence_deg)
      .add("quality_weights", tsdf.quality_weights)
      .add("min_weight", static_cast<double>(min_weight))
      .add("depth_with_confidence", frames_with_confidence)
      .add("carve", tsdf.carve)
      .add("carve_stride", tsdf.carve_stride)
      .add("pixel_stride", tsdf.pixel_stride)
      .add("keyframe_translation_m", config.keyframe_translation)
      .add("keyframe_rotation_deg", config.keyframe_rotation * 180.0 / 3.14159265358979323846)
      .add("messages", messages)
      .add("depth_frames", depth_frames)
      .add("frames_integrated", stats.integrated)
      .add("frames_dropped", stats.dropped)
      .add("frames_without_pose", stats.skipped_no_pose)
      .add("frames_tracking_lost", stats.skipped_tracking)
      .add("gps_fixes", fixes.size())
      .add("remove_pose_jumps", options.remove_pose_jumps)
      .add("pose_jumps_undone", jumps.jumps())
      .add("from_s", options.from)
      .add("to_s", options.to)
      .add("seconds", seconds);
  create_map_package(dir, builder.volume(), metadata, settings, log);
  // The camera's path, as fused (one point per depth frame), for overlaying
  // on the maps, e.g. in CloudCompare.
  std::ofstream path_file(dir / package_file::kTrajectory);
  path_file.imbue(std::locale::classic());
  path_file << "t,x,y,z\n";
  for (const auto& [t, position] : camera_path) {
    path_file << t << ',' << position.x() << ',' << position.y() << ',' << position.z() << '\n';
  }
}

}  // namespace vg::io
