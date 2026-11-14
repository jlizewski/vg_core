// The C API's live map builder: vg::MapBuilder, with the pose jump filter
// vg replay applies, and buffers for the arrays it hands out.

#include <memory>
#include <stdexcept>
#include <vector>

#include "common.hpp"
#include "vg_core/config/mapping_config.hpp"
#include "vg_core/map_builder.hpp"
#include "vg_core/pose_jumps.hpp"

using vg::c_api::guard;
using vg::c_api::require;

struct vg_map_builder {
  vg_map_builder(const vg::MapBuilderConfig& config, bool remove_jumps,
                 const vg::PoseJumpConfig& jump_config)
      : builder(config), remove_pose_jumps(remove_jumps), jumps(jump_config) {}

  vg::MapBuilder builder;
  bool remove_pose_jumps;
  vg::PoseJumpFilter jumps;
  // What update() and block_voxels() last handed out.
  std::vector<vg_height_cell> cells;
  std::vector<int32_t> blocks;
  std::vector<int32_t> voxels;
};

namespace {

// vg replay's build settings, before any settings file.
vg::io::BuildOptions default_build() { return vg::config::MappingConfig().build_options(); }

vg::MapBuilderConfig to_builder_config(const vg_map_builder_config& c) {
  if (!(c.voxel_size > 0.0) || !(c.truncation_distance > 0.0) || !(c.height_cell_size > 0.0)) {
    throw std::invalid_argument(
        "map builder config: voxel_size, truncation_distance and height_cell_size must be "
        "positive");
  }
  if (c.min_confidence < 0 || c.min_confidence > 255) {
    throw std::invalid_argument("map builder config: min_confidence must be 0-255");
  }
  if (c.pixel_stride < 1 || c.threads < 0) {
    throw std::invalid_argument(
        "map builder config: pixel_stride must be at least 1, threads at least 0");
  }
  // Settings this struct doesn't expose keep vg replay's values.
  vg::MapBuilderConfig b = default_build().builder;
  b.tsdf.voxel_size = c.voxel_size;
  b.tsdf.truncation_distance = c.truncation_distance;
  b.tsdf.min_depth = c.min_depth;
  b.tsdf.max_depth = c.max_depth;
  b.tsdf.min_confidence = static_cast<std::uint8_t>(c.min_confidence);
  b.tsdf.edge_ratio = c.edge_ratio;
  b.tsdf.max_incidence_deg = c.max_incidence_deg;
  b.tsdf.quality_weights = c.quality_weights != 0;
  b.tsdf.carve = c.carve != 0;
  b.tsdf.pixel_stride = c.pixel_stride;
  b.tsdf.threads = c.threads;
  b.tsdf.min_weight = c.min_weight;
  b.tsdf.drop_labels = c.drop_classes;
  b.height_cell_size = c.height_cell_size;
  b.max_pose_age = c.max_pose_age_ns;
  b.keyframe_translation = c.keyframe_translation;
  b.keyframe_rotation = c.keyframe_rotation;
  // vg replay's batch rebuild drops non-keyframes; apps using this API map live
  // and fuse the deferred frames later (vg_map_builder_integrate_deferred).
  b.drop_non_keyframes = false;
  b.deferred_path = vg::c_api::text(c.deferred_path);
  return b;
}

}  // namespace

vg_status vg_map_builder_config_defaults(vg_map_builder_config* config) {
  return guard([&] {
    require(config, "config");
    const vg::io::BuildOptions build = default_build();
    const vg::MapBuilderConfig& b = build.builder;
    const vg::TsdfConfig& t = b.tsdf;
    vg_map_builder_config c{};
    c.voxel_size = t.voxel_size;
    c.truncation_distance = t.truncation_distance;
    c.min_depth = t.min_depth;
    c.max_depth = t.max_depth;
    c.min_confidence = t.min_confidence;
    c.edge_ratio = t.edge_ratio;
    c.max_incidence_deg = t.max_incidence_deg;
    c.quality_weights = t.quality_weights ? 1 : 0;
    c.carve = t.carve ? 1 : 0;
    c.pixel_stride = t.pixel_stride;
    c.threads = t.threads;
    c.min_weight = 0.0f;
    c.drop_classes = t.drop_labels;
    // The live height map on the same grid as the package's 2D maps.
    c.height_cell_size = build.cell_size;
    c.max_pose_age_ns = b.max_pose_age;
    c.keyframe_translation = b.keyframe_translation;
    c.keyframe_rotation = b.keyframe_rotation;
    c.deferred_path = nullptr;
    c.remove_pose_jumps = build.remove_pose_jumps ? 1 : 0;
    *config = c;
  });
}

vg_status vg_map_builder_new(const vg_map_builder_config* config, vg_map_builder** out) {
  return guard([&] {
    require(config, "config");
    require(out, "out");
    const vg::MapBuilderConfig builder_config = to_builder_config(*config);
    *out = std::make_unique<vg_map_builder>(builder_config, config->remove_pose_jumps != 0,
                                            default_build().pose_jumps)
               .release();
  });
}

void vg_map_builder_free(vg_map_builder* builder) { delete builder; }

vg_status vg_map_builder_set_camera_extrinsics(vg_map_builder* builder, const char* camera,
                                               const vg_pose* body_from_camera) {
  return guard([&] {
    require(builder, "builder");
    require(camera, "camera");
    require(body_from_camera, "body_from_camera");
    builder->builder.set_camera_extrinsics(camera, vg::c_api::to_isometry(*body_from_camera));
  });
}

vg_status vg_map_builder_add_pose(vg_map_builder* builder, int64_t timestamp_ns,
                                  const vg_pose* world_from_body) {
  return guard([&] {
    require(builder, "builder");
    require(world_from_body, "world_from_body");
    vg::PoseStamped pose;
    pose.timestamp = timestamp_ns;
    pose.world_from_frame = vg::c_api::to_isometry(*world_from_body);
    builder->builder.add_pose(builder->remove_pose_jumps ? builder->jumps.correct(pose) : pose);
  });
}

vg_status vg_map_builder_set_tracking_ok(vg_map_builder* builder, int32_t ok) {
  return guard([&] {
    require(builder, "builder");
    builder->builder.set_tracking_ok(ok != 0);
  });
}

vg_status vg_map_builder_set_live(vg_map_builder* builder, int32_t live) {
  return guard([&] {
    require(builder, "builder");
    builder->builder.set_live(live != 0);
  });
}

vg_status vg_map_builder_add_depth(vg_map_builder* builder, const char* camera,
                                   const vg_depth_frame* frame) {
  return guard([&] {
    require(builder, "builder");
    require(camera, "camera");
    require(frame, "frame");
    builder->builder.add_depth(camera, vg::c_api::to_depth_frame(*frame));
  });
}

vg_status vg_map_builder_flush(vg_map_builder* builder) {
  return guard([&] {
    require(builder, "builder");
    builder->builder.flush();
  });
}

vg_status vg_map_builder_integrate_deferred(vg_map_builder* builder, size_t max_frames,
                                            size_t* remaining) {
  return guard([&] {
    require(builder, "builder");
    const std::size_t left = builder->builder.integrate_deferred(max_frames);
    if (remaining != nullptr) {
      *remaining = left;
    }
  });
}

vg_status vg_map_builder_set_min_weight(vg_map_builder* builder, float min_weight) {
  return guard([&] {
    require(builder, "builder");
    builder->builder.set_min_weight(min_weight);
  });
}

vg_status vg_map_builder_update(vg_map_builder* builder, vg_map_update* out) {
  return guard([&] {
    require(builder, "builder");
    require(out, "out");
    const vg::MapBuilder::Update update = builder->builder.update();
    builder->cells.clear();
    builder->cells.reserve(update.height_cells.size());
    for (const vg::HeightCell& cell : update.height_cells) {
      builder->cells.push_back(vg_height_cell{cell.x, cell.y, cell.height});
    }
    builder->blocks.clear();
    builder->blocks.reserve(3 * update.blocks.size());
    for (const Eigen::Vector3i& block : update.blocks) {
      builder->blocks.insert(builder->blocks.end(), {block.x(), block.y(), block.z()});
    }
    out->cells = builder->cells.data();
    out->cell_count = builder->cells.size();
    out->blocks = builder->blocks.data();
    out->block_count = update.blocks.size();
  });
}

vg_status vg_map_builder_block_voxels(vg_map_builder* builder, const int32_t block[3],
                                      const int32_t** voxels, size_t* count) {
  return guard([&] {
    require(builder, "builder");
    require(block, "block");
    require(voxels, "voxels");
    require(count, "count");
    const auto occupied =
        builder->builder.volume().occupied_voxels(Eigen::Vector3i(block[0], block[1], block[2]));
    builder->voxels.clear();
    builder->voxels.reserve(3 * occupied.size());
    for (const Eigen::Vector3i& v : occupied) {
      builder->voxels.insert(builder->voxels.end(), {v.x(), v.y(), v.z()});
    }
    *voxels = builder->voxels.data();
    *count = occupied.size();
  });
}

vg_status vg_map_builder_get_stats(const vg_map_builder* builder, vg_map_builder_stats* out) {
  return guard([&] {
    require(builder, "builder");
    require(out, "out");
    const vg::MapBuilder::Stats& s = builder->builder.stats();
    vg_map_builder_stats stats{};
    stats.integrated = s.integrated;
    stats.skipped_no_pose = s.skipped_no_pose;
    stats.skipped_tracking = s.skipped_tracking;
    stats.deferred = s.deferred;
    stats.dropped = s.dropped;
    stats.waiting = builder->builder.deferred_frames();
    stats.height_cells = builder->builder.height_map_cells();
    *out = stats;
  });
}

vg_status vg_map_builder_height_map(const vg_map_builder* builder, vg_grid** out) {
  return guard([&] {
    require(builder, "builder");
    require(out, "out");
    auto grid = std::make_unique<vg_grid>();
    grid->map = builder->builder.height_map();
    *out = grid.release();
  });
}
