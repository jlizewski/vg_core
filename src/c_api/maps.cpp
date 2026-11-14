// The C API's read side: 2D grids, map packages and sites.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "common.hpp"
#include "vg_core/config/mapping_config.hpp"
#include "vg_core/height_map_io.hpp"
#include "vg_core/map_package.hpp"
#include "vg_core/site.hpp"

using vg::c_api::guard;
using vg::c_api::require;

struct vg_package {
  explicit vg_package(const std::filesystem::path& dir) : package(dir, quiet) {}

  std::ostream quiet{nullptr};  // Drops MapPackage's log; before package.
  vg::MapPackage package;
  bool voxels_read = false;
  std::vector<int32_t> voxels;
  std::vector<uint8_t> classes;
};

struct vg_site {
  explicit vg_site(const std::filesystem::path& dir) : site(dir) {
    for (const vg::SiteVisit& visit : site.visits()) {
      package_dirs.push_back(site.package_dir(visit).u8string());
    }
  }

  vg::Site site;
  std::vector<std::string> package_dirs;  // Per visit, resolved.
};

// ----- Grids ---------------------------------------------------------------

vg_status vg_grid_load(const char* path, vg_grid** out) {
  return guard([&] {
    const auto file = vg::c_api::path_of(path, "path");
    require(out, "out");
    auto grid = std::make_unique<vg_grid>();
    grid->map = vg::load_height_map(file);
    *out = grid.release();
  });
}

void vg_grid_free(vg_grid* grid) { delete grid; }

vg_status vg_grid_get_info(const vg_grid* grid, vg_grid_info* out) {
  return guard([&] {
    require(grid, "grid");
    require(out, "out");
    vg_grid_info info{};
    info.width = grid->map.width;
    info.height = grid->map.height;
    info.origin_x = grid->map.origin.x();
    info.origin_y = grid->map.origin.y();
    info.cell_size = grid->map.cell_size;
    *out = info;
  });
}

const float* vg_grid_values(const vg_grid* grid) {
  return grid != nullptr ? grid->map.heights.data() : nullptr;
}

vg_status vg_grid_range(const vg_grid* grid, float* lowest, float* highest, size_t* cells) {
  return guard([&] {
    require(grid, "grid");
    float lo = std::numeric_limits<float>::quiet_NaN();
    float hi = std::numeric_limits<float>::quiet_NaN();
    std::size_t n = 0;
    for (const float v : grid->map.heights) {
      if (std::isnan(v)) {
        continue;
      }
      lo = n == 0 ? v : std::min(lo, v);
      hi = n == 0 ? v : std::max(hi, v);
      ++n;
    }
    if (lowest != nullptr) {
      *lowest = lo;
    }
    if (highest != nullptr) {
      *highest = hi;
    }
    if (cells != nullptr) {
      *cells = n;
    }
  });
}

// ----- Map packages --------------------------------------------------------

vg_status vg_package_open(const char* dir, vg_package** out) {
  return guard([&] {
    const auto path = vg::c_api::path_of(dir, "dir");
    require(out, "out");
    *out = std::make_unique<vg_package>(path).release();
  });
}

void vg_package_free(vg_package* package) { delete package; }

vg_status vg_package_voxels(vg_package* package, const int32_t** voxels, const uint8_t** classes,
                            size_t* count, double* voxel_size) {
  return guard([&] {
    require(package, "package");
    require(voxels, "voxels");
    require(count, "count");
    const double size = package->package.volume().config().voxel_size;
    if (!package->voxels_read) {
      // As the stages see them: specks left out, and a class only with
      // enough votes.
      const int min_votes = vg::config::MappingConfig().min_label_votes;
      const vg::TsdfVolume& volume = package->package.volume();
      for (const Eigen::Vector3d& center : package->package.occupied_centers()) {
        const Eigen::Vector3i v = (center / size).array().floor().cast<int>().matrix();
        package->voxels.insert(package->voxels.end(), {v.x(), v.y(), v.z()});
        const auto label = volume.label_of(v, min_votes);
        package->classes.push_back(label ? static_cast<uint8_t>(*label) : uint8_t{0});
      }
      package->voxels_read = true;
    }
    *voxels = package->voxels.data();
    if (classes != nullptr) {
      *classes = package->classes.data();
    }
    *count = package->classes.size();
    if (voxel_size != nullptr) {
      *voxel_size = size;
    }
  });
}

vg_status vg_package_geo_reference(const vg_package* package, vg_geo_reference* out,
                                   int32_t* has) {
  return guard([&] {
    require(package, "package");
    require(out, "out");
    require(has, "has");
    const auto& geo = package->package.metadata().geo;
    *has = geo ? 1 : 0;
    if (!geo) {
      return;
    }
    constexpr double kDegree = 3.14159265358979323846 / 180.0;
    vg_geo_reference g{};
    g.latitude = geo->latitude;
    g.longitude = geo->longitude;
    g.altitude = geo->altitude;
    g.x_bearing_deg = std::fmod(450.0 - geo->heading / kDegree, 360.0);  // As the manifest.
    g.bearing_sigma_deg = geo->heading_sigma / kDegree;
    *out = g;
  });
}

// ----- Sites ---------------------------------------------------------------

vg_status vg_site_open(const char* dir, vg_site** out) {
  return guard([&] {
    const auto path = vg::c_api::path_of(dir, "dir");
    require(out, "out");
    *out = std::make_unique<vg_site>(path).release();
  });
}

void vg_site_free(vg_site* site) { delete site; }

vg_status vg_site_visit_count(const vg_site* site, size_t* count) {
  return guard([&] {
    require(site, "site");
    require(count, "count");
    *count = site->site.visits().size();
  });
}

vg_status vg_site_get_visit(const vg_site* site, size_t index, vg_site_visit* out) {
  return guard([&] {
    require(site, "site");
    require(out, "out");
    const auto& visits = site->site.visits();
    if (index >= visits.size()) {
      throw std::invalid_argument("visit " + std::to_string(index) + " of " +
                                  std::to_string(visits.size()));
    }
    const vg::SiteVisit& v = visits[index];
    vg_site_visit visit{};
    visit.package_dir = site->package_dirs[index].c_str();
    visit.recorded = v.recorded.c_str();
    visit.yaw = v.site_from_map.yaw;
    visit.x = v.site_from_map.translation.x();
    visit.y = v.site_from_map.translation.y();
    visit.z = v.site_from_map.translation.z();
    visit.has_fit = v.fit ? 1 : 0;
    if (v.fit) {
      visit.overlap = v.fit->overlap;
      visit.inliers_30cm = v.fit->inliers_30cm;
      visit.median_distance = v.fit->median_distance;
    }
    *out = visit;
  });
}
