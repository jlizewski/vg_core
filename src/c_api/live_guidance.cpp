// The C API's live guidance: vg::LiveGuidance behind a handle.

#include "vg_core/live_guidance.hpp"

#include <memory>
#include <optional>
#include <vector>

#include "common.hpp"

using vg::c_api::guard;
using vg::c_api::require;

struct vg_live_guidance {
  explicit vg_live_guidance(vg::LiveGuidance l) : live(std::move(l)) {}
  vg::LiveGuidance live;
  std::vector<vg::HeightCell> cells;    // Reused for each update.
  std::vector<vg_live_target> targets;  // vg_live_guidance_targets' array.
};

vg_status vg_live_guidance_open(const char* site_dir, const char* tracking_frame,
                                vg_live_guidance** out) {
  return guard([&] {
    const auto site = vg::c_api::path_of(site_dir, "site_dir");
    require(tracking_frame, "tracking_frame");
    require(out, "out");
    *out = new vg_live_guidance(vg::LiveGuidance::for_site(site, tracking_frame));
  });
}

void vg_live_guidance_free(vg_live_guidance* live) { delete live; }

vg_status vg_live_guidance_update(vg_live_guidance* live, const vg_height_cell* cells, size_t count,
                                  double cell_size) {
  return guard([&] {
    require(live, "live");
    if (count > 0) {
      require(cells, "cells");
    }
    if (!(cell_size > 0.0)) {
      throw std::invalid_argument("cell_size must be positive");
    }
    live->cells.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      live->cells[i] = {cells[i].x, cells[i].y, cells[i].height};
    }
    live->live.update(live->cells, cell_size);
  });
}

vg_status vg_live_guidance_set_camera_pose(vg_live_guidance* live,
                                           const vg_pose* world_from_camera) {
  return guard([&] {
    require(live, "live");
    require(world_from_camera, "world_from_camera");
    live->live.set_camera_pose(vg::c_api::to_isometry(*world_from_camera));
  });
}

vg_status vg_live_guidance_get_state(const vg_live_guidance* live, vg_live_guidance_state* out) {
  return guard([&] {
    require(live, "live");
    require(out, "out");
    vg_live_guidance_state state{};
    state.done = live->live.done() ? 1 : 0;
    state.unseen_area = live->live.unseen_area();
    if (const auto p = live->live.position()) {
      state.has_position = 1;
      state.x = p->x();
      state.y = p->y();
    }
    if (const auto next = live->live.next()) {
      state.has_next = 1;
      state.next_rank = next->rank;
      state.next_x = next->position.x();
      state.next_y = next->position.y();
      state.next_heading = next->heading;
      state.next_distance = next->distance;
      state.next_turn = next->turn;
    }
    *out = state;
  });
}

vg_status vg_live_guidance_targets(vg_live_guidance* live, const vg_live_target** targets,
                                   size_t* count) {
  return guard([&] {
    require(live, "live");
    require(targets, "targets");
    require(count, "count");
    live->targets.clear();
    for (const vg::LiveTarget& t : live->live.targets()) {
      vg_live_target c{};
      c.rank = t.rank;
      c.order = t.order;
      c.area = t.area;
      c.center_x = t.center.x();
      c.center_y = t.center.y();
      c.cells = t.cells;
      c.seen_cells = t.seen_cells;
      c.seen = t.seen;
      c.done = t.done ? 1 : 0;
      live->targets.push_back(c);
    }
    *targets = live->targets.data();
    *count = live->targets.size();
  });
}
