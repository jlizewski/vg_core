// The C API's site layers: vg::SiteLayers read once, flattened into C
// arrays the handle owns, and the jobs that change them.

#include <algorithm>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "common.hpp"
#include "vg_core/config/jobs.hpp"
#include "vg_core/height_map_io.hpp"
#include "vg_core/site_layers.hpp"

using vg::c_api::guard;
using vg::c_api::require;

struct vg_site_layers {
  std::filesystem::path dir;
  vg::SiteLayers layers;
  double unknown_area = 0.0;
  std::vector<vg_boundary_vertex> boundary;
  std::vector<vg_feature> features;
  std::vector<vg_guidance_target> targets;
  std::vector<vg_look_spot> spots;
  std::vector<std::vector<double>> edit_points;
  std::vector<std::string> edit_walks;
  std::vector<vg_site_edit> edits;
};

namespace {

void flatten(vg_site_layers& h) {
  const vg::SiteLayers& l = h.layers;
  for (const vg::BoundaryVertex& v : l.boundary) {
    h.boundary.push_back({v.position.x(), v.position.y(), static_cast<int32_t>(v.edge), v.push});
  }
  for (const vg::SiteFeature& e : l.features) {
    const vg::Feature& f = e.feature;
    vg_feature c{};
    c.id = e.id.c_str();
    c.kind = static_cast<int32_t>(f.kind);
    c.semantic_class = static_cast<int32_t>(f.label);
    c.x = f.center.x();
    c.y = f.center.y();
    c.length = f.length;
    c.width = f.width;
    c.axis = f.axis;
    c.height = f.height;
    c.seen = e.seen;
    c.missed = e.missed;
    c.retired = e.retired ? 1 : 0;
    c.spread = e.spread;
    h.features.push_back(c);
  }
  // Targets in visiting order.
  std::vector<const vg::GuidanceTarget*> ordered;
  for (const vg::GuidanceTarget& t : l.guidance.targets) {
    ordered.push_back(&t);
    h.unknown_area += t.area;
  }
  std::stable_sort(ordered.begin(), ordered.end(),
                   [](const auto* a, const auto* b) { return a->order < b->order; });
  for (const vg::GuidanceTarget* t : ordered) {
    vg_guidance_target c{};
    c.rank = t->rank;
    c.order = t->order;
    c.area = t->area;
    c.center_x = t->center.x();
    c.center_y = t->center.y();
    c.seen = t->seen;
    c.first_spot = h.spots.size();
    c.spot_count = t->spots.size();
    for (const vg::LookSpot& s : t->spots) {
      h.spots.push_back({s.position.x(), s.position.y(), s.heading, s.distance, s.sees});
    }
    h.targets.push_back(c);
  }
  // Edits: their points and review walks first (the C structs point into them).
  h.edit_points.reserve(l.edits.size());
  h.edit_walks.reserve(l.edits.size());
  for (const vg::GroundEdit& e : l.edits) {
    std::vector<double>& xy = h.edit_points.emplace_back();
    for (const Eigen::Vector2d& p : e.polygon) {
      xy.push_back(p.x());
      xy.push_back(p.y());
    }
    vg_site_edit c{};
    c.id = e.id.c_str();
    c.time = e.time.c_str();
    c.kind = static_cast<int32_t>(e.kind);
    c.points = xy.data();
    c.point_count = e.polygon.size();
    c.note = e.note.c_str();
    std::string& walks = h.edit_walks.emplace_back();
    const auto review = std::find_if(l.reviews.begin(), l.reviews.end(),
                                     [&](const auto& r) { return r.id == e.id; });
    if (review != l.reviews.end()) {
      c.reviewed = 1;
      c.cells = review->cells;
      c.disagreeing_cells = review->disagreeing;
      for (std::size_t k = 0; k < review->walks.size(); ++k) {
        walks += (k > 0 ? ";" : "") + review->walks[k];
      }
    }
    c.disagreeing_walks = walks.c_str();
    h.edits.push_back(c);
  }
}

}  // namespace

vg_status vg_site_layers_open(const char* dir, vg_site_layers** out) {
  return guard([&] {
    const auto site = vg::c_api::path_of(dir, "dir");
    require(out, "out");
    auto h = std::make_unique<vg_site_layers>();
    h->dir = site;
    h->layers = vg::load_site_layers(site);
    flatten(*h);
    *out = h.release();
  });
}

void vg_site_layers_free(vg_site_layers* layers) { delete layers; }

vg_status vg_site_layers_get_info(const vg_site_layers* layers, vg_site_layers_info* out) {
  return guard([&] {
    require(layers, "layers");
    require(out, "out");
    vg_site_layers_info info{};
    info.has_ground = layers->layers.has_ground ? 1 : 0;
    info.has_guidance = layers->layers.has_guidance ? 1 : 0;
    info.done = layers->layers.has_guidance && layers->layers.guidance.done ? 1 : 0;
    info.unknown_area = layers->unknown_area;
    info.boundary_vertex_count = layers->boundary.size();
    info.feature_count = layers->features.size();
    info.target_count = layers->targets.size();
    info.spot_count = layers->spots.size();
    info.edit_count = layers->edits.size();
    *out = info;
  });
}

vg_status vg_site_layers_ground_grid(const vg_site_layers* layers, int32_t which, vg_grid** out) {
  return guard([&] {
    require(layers, "layers");
    require(out, "out");
    const char* name = nullptr;
    switch (which) {
      case VG_GROUND_GRID_HEIGHT:
        name = vg::ground_layer_file::kHeights;
        break;
      case VG_GROUND_GRID_STATE:
        name = vg::ground_layer_file::kState;
        break;
      case VG_GROUND_GRID_PROVENANCE:
        name = vg::ground_layer_file::kProvenance;
        break;
      case VG_GROUND_GRID_OBSERVATIONS:
        name = vg::ground_layer_file::kObservations;
        break;
      case VG_GROUND_GRID_SPREAD:
        name = vg::ground_layer_file::kSpread;
        break;
      default:
        throw std::invalid_argument("which: not a VG_GROUND_GRID_* value");
    }
    if (!layers->layers.has_ground) {
      throw std::runtime_error("the site has no ground layer yet (make it with vg site ground)");
    }
    auto grid = std::make_unique<vg_grid>();
    grid->map = vg::load_height_map(layers->dir / vg::ground_layer_file::kDir / name);
    *out = grid.release();
  });
}

vg_status vg_site_layers_boundary(const vg_site_layers* layers, const vg_boundary_vertex** vertices,
                                  size_t* count) {
  return guard([&] {
    require(layers, "layers");
    require(vertices, "vertices");
    require(count, "count");
    *vertices = layers->boundary.data();
    *count = layers->boundary.size();
  });
}

vg_status vg_site_layers_features(const vg_site_layers* layers, const vg_feature** features,
                                  size_t* count) {
  return guard([&] {
    require(layers, "layers");
    require(features, "features");
    require(count, "count");
    *features = layers->features.data();
    *count = layers->features.size();
  });
}

vg_status vg_site_layers_guidance(const vg_site_layers* layers, const vg_guidance_target** targets,
                                  size_t* target_count, const vg_look_spot** spots,
                                  size_t* spot_count) {
  return guard([&] {
    require(layers, "layers");
    require(targets, "targets");
    require(target_count, "target_count");
    require(spots, "spots");
    require(spot_count, "spot_count");
    *targets = layers->targets.data();
    *target_count = layers->targets.size();
    *spots = layers->spots.data();
    *spot_count = layers->spots.size();
  });
}

vg_status vg_site_layers_edits(const vg_site_layers* layers, const vg_site_edit** edits,
                               size_t* count) {
  return guard([&] {
    require(layers, "layers");
    require(edits, "edits");
    require(count, "count");
    *edits = layers->edits.data();
    *count = layers->edits.size();
  });
}

vg_status vg_site_make_ground(const char* site_dir, const char* const* settings,
                              size_t settings_count, vg_log_fn log, void* user_data) {
  return guard([&] {
    const auto site = vg::c_api::path_of(site_dir, "site_dir");
    const auto args = vg::c_api::strings(settings, settings_count, "settings");
    vg::c_api::Log out(log, user_data);
    vg::config::make_site_ground(site, args, out.stream());
  });
}

vg_status vg_site_add_edit(const char* site_dir, int32_t kind, const double* points,
                           size_t point_count, const char* note, char* id, size_t id_size,
                           vg_log_fn log, void* user_data) {
  return guard([&] {
    const auto site = vg::c_api::path_of(site_dir, "site_dir");
    vg::GroundEdit edit;
    switch (kind) {
      case VG_EDIT_GROUND:
        edit.kind = vg::GroundEdit::Kind::Ground;
        break;
      case VG_EDIT_NOT_GROUND:
        edit.kind = vg::GroundEdit::Kind::NotGround;
        break;
      case VG_EDIT_BOUNDARY:
        edit.kind = vg::GroundEdit::Kind::Boundary;
        break;
      default:
        throw std::invalid_argument("kind: not a VG_EDIT_* value");
    }
    if (point_count > 0) {
      require(points, "points");
    }
    for (std::size_t i = 0; i < point_count; ++i) {
      edit.polygon.emplace_back(points[2 * i], points[2 * i + 1]);
    }
    edit.note = vg::c_api::text(note);
    vg::c_api::Log out(log, user_data);
    const std::string made = vg::config::add_site_edit(site, std::move(edit), out.stream());
    if (id != nullptr && made.size() < id_size) {
      std::memcpy(id, made.c_str(), made.size() + 1);
    } else if (id != nullptr && id_size > 0) {
      id[0] = '\0';  // Too small: no id, but the edit is made.
    }
  });
}

vg_status vg_site_delete_edit(const char* site_dir, const char* id, vg_log_fn log,
                              void* user_data) {
  return guard([&] {
    const auto site = vg::c_api::path_of(site_dir, "site_dir");
    require(id, "id");
    vg::c_api::Log out(log, user_data);
    vg::config::delete_site_edit(site, id, out.stream());
  });
}
