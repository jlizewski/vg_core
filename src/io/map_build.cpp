#include "vg_core/io/map_build.hpp"

#include <chrono>
#include <variant>
#include <vector>

#include "vg_core/geo_reference.hpp"
#include "vg_core/io/session_player.hpp"
#include "vg_core/io/session_reader.hpp"
#include "vg_core/map_package.hpp"

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
  const std::size_t messages = play(
      reader,
      [&](const SessionMessage& m) {
        mapper.handle(m);
        if (const auto* fix = std::get_if<GpsFix>(&m)) {
          fixes.push_back(*fix);
        } else if (const auto* pose = std::get_if<PoseStamped>(&m)) {
          poses.push_back(*pose);
        } else if (std::holds_alternative<DepthImage>(m)) {
          ++depth_frames;
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
  if (depth_frames == 0) {
    log << "warning: no depth images in " << session.string()
        << " (no /cam/<name>/depth topic), so the map is empty\n";
  }

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
      .add("seconds", seconds);
  create_map_package(dir, builder.volume(), metadata, settings, log);
}

}  // namespace vg::io
