#include "vg_core/map_package.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

#include "vg_core/diagnostics.hpp"
#include "vg_core/height_map.hpp"
#include "vg_core/height_map_io.hpp"
#include "vg_core/semantics.hpp"
#include "vg_core/sun_map.hpp"
#include "vg_core/version.hpp"
#include "vg_core/voxel_cleanup.hpp"
#include "vg_core/voxel_map_io.hpp"

namespace vg {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegree = kPi / 180.0;

struct FileInfo {
  const char* name;
  const char* units;
  const char* description;
};

// Every file a package can hold, in manifest order.
constexpr std::array<FileInfo, 13> kFiles = {{
    {package_file::kMap, "", "3D map: TSDF voxel volume and geo-reference (input to every stage)"},
    {package_file::kVoxels, "m", "Occupied voxels of the 3D map as a mesh of cubes"},
    {package_file::kSurface, "m", "Height of the highest surface per cell"},
    {package_file::kGround, "m", "Height of the bare ground per cell, holes under objects patched"},
    {package_file::kGroundPatched, "",
     "Diagnostic: 1 where ground.asc is patched (interpolated), 0 where it was seen"},
    {package_file::kObjects, "m", "Voxels of everything on or over the ground"},
    {package_file::kSunHours, "h", "Hours of direct sun per ground cell over the year"},
    {package_file::kSunEnergy, "kWh/m^2",
     "Clear-sky direct solar energy per ground cell over the year"},
    {package_file::kConfig, "",
     "Settings the package was made with: edit and re-run stages with vg map"},
    {package_file::kTrajectory, "s, m",
     "Path of the depth camera while mapping: time from the first reading, x, y, z"},
    {package_file::kLabels, "m", "Occupied voxels colored by semantic class"},
    {package_file::kSemantic, "class",
     "Semantic class of the highest labeled voxel per cell: 1 ground, 2 plant, 3 tree, "
     "4 structure, 5 sky, 6 dynamic"},
    {package_file::kLayers, "m",
     "Diagnostic: gap from the lowest surface to a second one above it (ghosts if small)"},
}};

// Stages in the order they run, for the manifest.
constexpr std::array<const char*, 8> kStages = {"build",  "voxels", "labels", "surface",
                                                "ground", "sun",    "semantic", "layers"};

std::string json_number(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out.precision(10);
  out << value;
  return out.str();
}

std::string json_string(const std::string& value) {
  std::string out = "\"";
  for (const char c : value) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      out += ' ';
    } else {
      out += c;
    }
  }
  return out + "\"";
}

double seconds_since(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

// Compass bearing of the world +x axis, in degrees clockwise from north.
double x_bearing_deg(const GeoReference& geo) {
  return std::fmod(450.0 - geo.heading / kDegree, 360.0);
}

std::vector<Eigen::Vector3i> sorted_occupied_voxels(const TsdfVolume& volume) {
  auto voxels = volume.occupied_voxels();
  std::sort(voxels.begin(), voxels.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
    if (a.z() != b.z()) return a.z() < b.z();
    if (a.y() != b.y()) return a.y() < b.y();
    return a.x() < b.x();
  });
  return voxels;
}

// The stage entries of an existing manifest, by stage name, as JSON text.
// Each is on a line of its own, as update_manifest() writes them.
std::map<std::string, std::string> read_stage_entries(const std::filesystem::path& path) {
  std::map<std::string, std::string> entries;
  std::ifstream in(path);
  std::string line;
  bool in_stages = false;
  while (std::getline(in, line)) {
    if (line == "  \"stages\": {") {
      in_stages = true;
      continue;
    }
    if (!in_stages) {
      continue;
    }
    if (line.rfind("  }", 0) == 0) {
      break;
    }
    const auto open = line.find('"');
    const auto close = line.find("\": {");
    if (open == std::string::npos || close == std::string::npos || close <= open) {
      continue;
    }
    std::string value = line.substr(close + 3);
    if (!value.empty() && value.back() == ',') {
      value.pop_back();
    }
    entries[line.substr(open + 1, close - open - 1)] = value;
  }
  return entries;
}

void write_manifest(const std::filesystem::path& dir, const MapMetadata& metadata,
                    double voxel_size, const std::map<std::string, std::string>& stages) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "{\n";
  out << "  \"format\": \"vg-map-package\",\n";
  out << "  \"format_version\": 1,\n";
  out << "  \"vg_core_version\": " << json_string(std::string(version_string())) << ",\n";
  out << "  \"session\": " << json_string(metadata.session) << ",\n";
  out << "  \"voxel_size_m\": " << json_number(voxel_size) << ",\n";
  out << "  \"cell_size_m\": " << json_number(metadata.cell_size) << ",\n";
  out << "  \"frames_integrated\": " << metadata.frames_integrated << ",\n";
  out << "  \"coordinates\": \"local map frame in meters, z up; geo_reference places it\",\n";
  if (metadata.geo) {
    const GeoReference& geo = *metadata.geo;
    out << "  \"geo_reference\": {\"latitude\": " << json_number(geo.latitude)
        << ", \"longitude\": " << json_number(geo.longitude)
        << ", \"altitude_m\": " << json_number(geo.altitude)
        << ", \"x_axis_bearing_deg\": " << json_number(x_bearing_deg(geo))
        << ", \"bearing_sigma_deg\": " << json_number(geo.heading_sigma / kDegree) << "},\n";
  } else {
    out << "  \"geo_reference\": null,\n";
  }

  // Known stages in run order, then any others.
  std::vector<std::pair<std::string, std::string>> ordered;
  for (const char* stage : kStages) {
    if (const auto it = stages.find(stage); it != stages.end()) {
      ordered.emplace_back(*it);
    }
  }
  for (const auto& entry : stages) {
    if (std::find(kStages.begin(), kStages.end(), entry.first) == kStages.end()) {
      ordered.push_back(entry);
    }
  }
  out << "  \"stages\": {\n";
  for (std::size_t i = 0; i < ordered.size(); ++i) {
    out << "    " << json_string(ordered[i].first) << ": " << ordered[i].second
        << (i + 1 < ordered.size() ? ",\n" : "\n");
  }
  out << "  },\n";

  std::vector<const FileInfo*> present;
  for (const FileInfo& file : kFiles) {
    if (std::filesystem::is_regular_file(dir / file.name)) {
      present.push_back(&file);
    }
  }
  out << "  \"files\": [\n";
  for (std::size_t i = 0; i < present.size(); ++i) {
    out << "    {\"name\": " << json_string(present[i]->name)
        << ", \"units\": " << json_string(present[i]->units)
        << ", \"description\": " << json_string(present[i]->description) << "}"
        << (i + 1 < present.size() ? ",\n" : "\n");
  }
  out << "  ]\n}\n";

  const auto path = dir / package_file::kManifest;
  std::ofstream file(path, std::ios::binary);
  file << out.str();
  if (!file) {
    throw std::runtime_error("cannot write " + path.string());
  }
}

}  // namespace

StageSettings& StageSettings::add(const std::string& key, double value) {
  entries_.emplace_back(key, json_number(value));
  return *this;
}

StageSettings& StageSettings::add(const std::string& key, float value) {
  // A float's own precision, so 0.05f reads 0.05.
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out.precision(7);
  out << value;
  entries_.emplace_back(key, std::isfinite(value) ? out.str() : "null");
  return *this;
}

StageSettings& StageSettings::add(const std::string& key, int value) {
  entries_.emplace_back(key, std::to_string(value));
  return *this;
}

StageSettings& StageSettings::add(const std::string& key, std::size_t value) {
  entries_.emplace_back(key, std::to_string(value));
  return *this;
}

StageSettings& StageSettings::add(const std::string& key, bool value) {
  entries_.emplace_back(key, value ? "true" : "false");
  return *this;
}

StageSettings& StageSettings::add(const std::string& key, const std::string& value) {
  entries_.emplace_back(key, json_string(value));
  return *this;
}

StageSettings& StageSettings::add(const std::string& key, const char* value) {
  return add(key, std::string(value));
}

std::string StageSettings::json() const {
  std::string out = "{";
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    out += (i > 0 ? ", " : "") + json_string(entries_[i].first) + ": " + entries_[i].second;
  }
  return out + "}";
}

void update_manifest(const std::filesystem::path& dir, const MapMetadata& metadata,
                     double voxel_size, const std::string& stage, const StageSettings& settings) {
  auto stages = read_stage_entries(dir / package_file::kManifest);
  stages[stage] = settings.json();
  write_manifest(dir, metadata, voxel_size, stages);
}

void create_map_package(const std::filesystem::path& dir, const TsdfVolume& volume,
                        const MapMetadata& metadata, const StageSettings& build_settings,
                        std::ostream& log) {
  std::filesystem::create_directories(dir);
  // A new map makes everything derived from an old one stale.
  for (const FileInfo& file : kFiles) {
    std::filesystem::remove(dir / file.name);
  }
  std::filesystem::remove(dir / package_file::kManifest);

  const auto start = std::chrono::steady_clock::now();
  save_map(volume, metadata, dir / package_file::kMap);
  write_manifest(dir, metadata, volume.config().voxel_size, {{"build", build_settings.json()}});
  log << "3D map: " << volume.num_blocks() << " blocks -> " << package_file::kMap << " ("
      << seconds_since(start) << " s)" << std::endl;
}

MapPackage::MapPackage(std::filesystem::path dir, std::ostream& log)
    : dir_(std::move(dir)), log_(log), map_(load_map(dir_ / package_file::kMap)) {}

void MapPackage::set_min_weight(float min_weight) {
  map_.volume.set_min_weight(min_weight);
  occupied_.reset();
}

void MapPackage::set_min_blob_volume(double volume) {
  min_blob_volume_ = volume;
  occupied_.reset();
}

const std::vector<Eigen::Vector3i>& MapPackage::occupied() {
  if (!occupied_) {
    const auto all = sorted_occupied_voxels(map_.volume);
    const double voxel = map_.volume.config().voxel_size;
    const auto min_voxels =
        static_cast<std::size_t>(std::ceil(min_blob_volume_ / (voxel * voxel * voxel) - 1e-9));
    auto cleaned = remove_small_components(all, min_voxels);
    specks_.clear();
    if (cleaned.removed_voxels > 0) {
      std::set<std::tuple<int, int, int>> kept;
      for (const auto& v : cleaned.voxels) {
        kept.emplace(v.x(), v.y(), v.z());
      }
      for (const auto& v : all) {
        if (kept.count({v.x(), v.y(), v.z()}) == 0) {
          specks_.push_back(v);
        }
      }
      log_ << "left out " << cleaned.removed_components << " specks under " << min_blob_volume_
           << " m^3 (" << cleaned.removed_voxels << " of " << all.size() << " voxels)" << std::endl;
    }
    occupied_ = std::move(cleaned.voxels);
  }
  return *occupied_;
}

void MapPackage::make_voxels() {
  const auto start = std::chrono::steady_clock::now();
  const double voxel_size = map_.volume.config().voxel_size;
  save_voxels_ply(occupied(), voxel_size, path(package_file::kVoxels));
  update_manifest(dir_, map_.metadata, voxel_size, "voxels",
                  StageSettings()
                      .add("min_weight", map_.volume.config().min_weight)
                      .add("min_blob_m3", min_blob_volume_)
                      .add("voxels", occupied().size()));
  log_ << "voxels: " << occupied().size() << " occupied -> " << package_file::kVoxels << " ("
       << seconds_since(start) << " s)" << std::endl;
  if (!map_.volume.has_labels()) {
    return;
  }
  // The same voxels colored by class, and how many of each.
  std::vector<std::array<std::uint8_t, 3>> colors;
  colors.reserve(occupied().size());
  std::array<std::size_t, kSemanticClasses> counts{};
  for (const auto& v : occupied()) {
    const SemanticClass c =
        map_.volume.label_of(v, min_label_votes_).value_or(SemanticClass::Unknown);
    colors.push_back(semantic_class_color(c));
    ++counts[static_cast<std::size_t>(c)];
  }
  save_voxels_ply(occupied(), colors, voxel_size, path(package_file::kLabels));
  StageSettings settings;
  settings.add("min_label_votes", min_label_votes_);
  log_ << "labels:";
  for (std::size_t c = 0; c < kSemanticClasses; ++c) {
    const std::string name(semantic_class_name(static_cast<SemanticClass>(c)));
    settings.add(name + "_voxels", counts[c]);
    log_ << ' ' << name << ' ' << counts[c];
  }
  update_manifest(dir_, map_.metadata, voxel_size, "labels", settings);
  log_ << " -> " << package_file::kLabels << std::endl;
}

bool MapPackage::make_semantic(const SurfaceOptions& options) {
  if (!map_.volume.has_labels()) {
    log_ << "semantic: skipped, the map has no semantic labels (record depth/labels)"
         << std::endl;
    return false;
  }
  const auto start = std::chrono::steady_clock::now();
  const double cell_size = options.cell_size.value_or(map_.metadata.cell_size);
  const double voxel_size = map_.volume.config().voxel_size;
  // Cells on the same global grid as the other maps.
  const auto cell_of = [&](const Eigen::Vector3i& v) -> Eigen::Vector2i {
    return {static_cast<int>(std::floor((v.x() + 0.5) * voxel_size / cell_size)),
            static_cast<int>(std::floor((v.y() + 0.5) * voxel_size / cell_size))};
  };
  std::map<std::pair<int, int>, std::pair<int, SemanticClass>> top;  // Cell: (z, class).
  for (const auto& v : occupied()) {
    const auto label = map_.volume.label_of(v, min_label_votes_);
    if (!label) {
      continue;
    }
    const Eigen::Vector2i c = cell_of(v);
    auto [it, added] = top.try_emplace({c.x(), c.y()}, v.z(), *label);
    if (!added && v.z() > it->second.first) {
      it->second = {v.z(), *label};
    }
  }
  HeightMap map;
  map.cell_size = cell_size;
  if (!top.empty()) {
    Eigen::Vector2i min(top.begin()->first.first, top.begin()->first.second);
    Eigen::Vector2i max = min;
    for (const auto& [cell, value] : top) {
      min = min.cwiseMin(Eigen::Vector2i(cell.first, cell.second));
      max = max.cwiseMax(Eigen::Vector2i(cell.first, cell.second));
    }
    map.origin = min.cast<double>() * cell_size;
    map.width = max.x() - min.x() + 1;
    map.height = max.y() - min.y() + 1;
    map.heights.assign(static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height),
                       std::numeric_limits<float>::quiet_NaN());
    for (const auto& [cell, value] : top) {
      const auto x = static_cast<std::size_t>(cell.first - min.x());
      const auto y = static_cast<std::size_t>(cell.second - min.y());
      map.heights[y * static_cast<std::size_t>(map.width) + x] =
          static_cast<float>(static_cast<int>(value.second));
    }
  }
  save_height_map(map, path(package_file::kSemantic));
  update_manifest(dir_, map_.metadata, voxel_size, "semantic",
                  StageSettings()
                      .add("cell_size_m", cell_size)
                      .add("min_label_votes", min_label_votes_)
                      .add("labeled_cells", top.size()));
  log_ << "semantic: " << top.size() << " labeled cells -> " << package_file::kSemantic << " ("
       << seconds_since(start) << " s)" << std::endl;
  return true;
}

void MapPackage::make_surface(const SurfaceOptions& options) {
  const auto start = std::chrono::steady_clock::now();
  const double cell_size = options.cell_size.value_or(map_.metadata.cell_size);
  auto points = map_.volume.extract_surface_points();
  occupied();  // Finds the specks.
  if (!specks_.empty()) {
    // A surface point belongs to a speck if its voxel or a face neighbor is one.
    const double voxel = map_.volume.config().voxel_size;
    std::set<std::tuple<int, int, int>> specks;
    for (const auto& v : specks_) {
      specks.emplace(v.x(), v.y(), v.z());
    }
    const auto in_speck = [&](const Eigen::Vector3d& p) {
      const Eigen::Vector3i v = (p / voxel).array().floor().cast<int>();
      for (int axis = -1; axis < 3; ++axis) {
        for (const int side : {-1, 1}) {
          const Eigen::Vector3i n =
              axis < 0 ? v : Eigen::Vector3i(v + side * Eigen::Vector3i::Unit(axis));
          if (specks.count({n.x(), n.y(), n.z()}) != 0) {
            return true;
          }
        }
      }
      return false;
    };
    points.erase(std::remove_if(points.begin(), points.end(), in_speck), points.end());
  }
  const HeightMap map = make_height_map(points, cell_size);
  save_height_map(map, path(package_file::kSurface));
  update_manifest(dir_, map_.metadata, map_.volume.config().voxel_size, "surface",
                  StageSettings()
                      .add("cell_size_m", cell_size)
                      .add("min_weight", map_.volume.config().min_weight)
                      .add("min_blob_m3", min_blob_volume_));
  log_ << "surface: " << map.width << " x " << map.height << " cells -> "
       << package_file::kSurface << " (" << seconds_since(start) << " s)\n";
}

void MapPackage::make_ground(const GroundOptions& options) {
  const auto start = std::chrono::steady_clock::now();
  GroundConfig config = options.config;
  config.cell_size = options.cell_size.value_or(map_.metadata.cell_size);
  const double voxel_size = map_.volume.config().voxel_size;
  // Each voxel's voted class, when the map has labels.
  std::vector<SemanticClass> labels;
  const bool labeled = config.use_labels && map_.volume.has_labels();
  if (labeled) {
    labels.reserve(occupied().size());
    for (const auto& v : occupied()) {
      labels.push_back(map_.volume.label_of(v, min_label_votes_).value_or(SemanticClass::Unknown));
    }
  }
  const GroundSegmentation split = segment_ground(occupied(), voxel_size, config, labels);

  save_height_map(split.ground.heights, path(package_file::kGround));
  HeightMap patched_map = split.ground.heights;
  for (std::size_t i = 0; i < patched_map.heights.size(); ++i) {
    if (!std::isnan(patched_map.heights[i])) {
      patched_map.heights[i] = split.ground.filled[i] != 0 ? 1.0f : 0.0f;
    }
  }
  save_height_map(patched_map, path(package_file::kGroundPatched));
  std::vector<Eigen::Vector3i> objects;
  std::size_t grounded = 0;
  for (const MapSegment& segment : split.segments) {
    objects.insert(objects.end(), segment.voxels.begin(), segment.voxels.end());
    grounded += segment.grounded ? 1 : 0;
  }
  save_voxels_ply(objects, voxel_size, path(package_file::kObjects));
  std::size_t patched = 0;
  for (const auto f : split.ground.filled) {
    patched += f;
  }
  // Objects by what they mostly are.
  std::array<std::size_t, kSemanticClasses> objects_by_label{};
  for (const MapSegment& segment : split.segments) {
    ++objects_by_label[static_cast<std::size_t>(segment.label)];
  }

  StageSettings settings;
  settings.add("cell_size_m", config.cell_size)
      .add("min_weight", map_.volume.config().min_weight)
      .add("min_blob_m3", min_blob_volume_)
      .add("max_slope", config.max_slope)
      .add("step_tolerance_m", config.step_tolerance)
      .add("max_gap_m", config.max_gap)
      .add("min_isolated_area_m2", config.min_isolated_area)
      .add("max_hole_area_m2", config.max_hole_area)
      .add("ground_voxels", split.ground_voxels.size())
      .add("objects", split.segments.size())
      .add("objects_grounded", grounded)
      .add("use_labels", labeled)
      .add("columns_not_ground_by_label", split.columns_not_ground_by_label);
  if (labeled) {
    for (std::size_t c = 0; c < kSemanticClasses; ++c) {
      settings.add(std::string(semantic_class_name(static_cast<SemanticClass>(c))) + "_objects",
                   objects_by_label[c]);
    }
  }
  update_manifest(dir_, map_.metadata, voxel_size, "ground", settings);
  log_ << "ground: " << split.ground.heights.width << " x " << split.ground.heights.height
       << " cells (" << patched << " patched), " << split.segments.size() << " objects ("
       << grounded << " standing on the ground) -> " << package_file::kGround << ", "
       << package_file::kObjects << " (" << seconds_since(start) << " s)" << std::endl;
  if (labeled) {
    log_ << "ground: " << split.columns_not_ground_by_label
         << " columns not ground by their label; objects:";
    for (std::size_t c = 0; c < kSemanticClasses; ++c) {
      log_ << ' ' << semantic_class_name(static_cast<SemanticClass>(c)) << ' '
           << objects_by_label[c];
    }
    log_ << std::endl;
  }
}

bool MapPackage::make_sun(const SunOptions& options) {
  if (!std::filesystem::is_regular_file(path(package_file::kGround))) {
    log_ << "sun: skipped, no " << package_file::kGround << " (run the ground stage first)\n";
    return false;
  }
  if (occupied().empty()) {
    log_ << "sun: skipped, the map is empty\n";
    return false;
  }
  std::optional<GeoReference> geo = map_.metadata.geo;
  if (options.latitude && options.longitude) {
    if (!geo) {
      geo.emplace();
      geo->heading_sigma = std::numeric_limits<double>::infinity();
    }
    geo->latitude = *options.latitude;
    geo->longitude = *options.longitude;
  }
  if (!geo) {
    log_ << "sun: skipped, the session has no GPS fixes (give the latitude and longitude)\n";
    return false;
  }
  if (options.x_bearing_deg) {
    geo->heading = (90.0 - *options.x_bearing_deg) * kDegree;
    geo->heading_sigma = 0.0;
  }
  {
    const auto precision = log_.precision(9);
    log_ << "sun: map at " << geo->latitude << ", " << geo->longitude;
    log_.precision(precision);
  }
  log_ << ", +x axis bearing " << x_bearing_deg(*geo) << " deg";
  if (std::isinf(geo->heading_sigma)) {
    log_ << " (unknown: no pose track to compare the GPS with; set the x bearing)\n";
  } else {
    log_ << " +/- " << geo->heading_sigma / kDegree << " deg\n";
    if (geo->heading_sigma > 10.0 * kDegree) {
      log_ << "warning: the map's heading is poorly known from GPS (the walk was short for the "
              "GPS accuracy), so shadows may point the wrong way; set the x bearing if you "
              "know it\n";
    }
  }

  const auto start = std::chrono::steady_clock::now();
  const double voxel_size = map_.volume.config().voxel_size;
  SunMapConfig config = sun_map_config_for_year(options.year, options.step_minutes * 60.0);
  config.sample_height = options.sample_height.value_or(std::max(0.04, voxel_size));
  config.min_elevation_deg = options.min_elevation_deg;
  const HeightMap ground = load_height_map(path(package_file::kGround));
  // Everything mapped casts shadows: objects, and the ground itself on slopes.
  const SunMap sun = compute_sun_map(ground, occupied(), voxel_size, *geo, config);
  save_height_map(sun.sun_hours, path(package_file::kSunHours));
  save_height_map(sun.irradiation, path(package_file::kSunEnergy));

  update_manifest(dir_, map_.metadata, voxel_size, "sun",
                  StageSettings()
                      .add("year", options.year)
                      .add("min_weight", map_.volume.config().min_weight)
                      .add("min_blob_m3", min_blob_volume_)
                      .add("step_minutes", options.step_minutes)
                      .add("sample_height_m", config.sample_height)
                      .add("latitude", geo->latitude)
                      .add("longitude", geo->longitude)
                      .add("x_axis_bearing_deg", x_bearing_deg(*geo))
                      .add("bearing_sigma_deg", geo->heading_sigma / kDegree)
                      .add("daylight_hours", sun.daylight_hours)
                      .add("sun_steps", sun.sun_steps));
  log_ << "sun: " << options.year << ", " << sun.sun_steps << " steps with the sun up ("
       << sun.daylight_hours << " h) -> " << package_file::kSunHours << ", "
       << package_file::kSunEnergy << " (" << seconds_since(start) << " s)\n";
  return true;
}

void MapPackage::make_layers(const LayersOptions& options) {
  const auto start = std::chrono::steady_clock::now();
  const double cell_size = options.cell_size.value_or(map_.metadata.cell_size);
  const double voxel_size = map_.volume.config().voxel_size;
  const SurfaceLayers layers =
      find_surface_layers(occupied(), voxel_size, cell_size, options.min_gap_voxels);
  save_height_map(layers.gap, path(package_file::kLayers));
  update_manifest(dir_, map_.metadata, voxel_size, "layers",
                  StageSettings()
                      .add("cell_size_m", cell_size)
                      .add("min_weight", map_.volume.config().min_weight)
                      .add("min_blob_m3", min_blob_volume_)
                      .add("columns", layers.columns)
                      .add("layered_columns", layers.layered_columns)
                      .add("gap_under_0.15_m", layers.by_gap[0])
                      .add("gap_0.15_to_0.3_m", layers.by_gap[1])
                      .add("gap_0.3_to_0.6_m", layers.by_gap[2])
                      .add("gap_0.6_to_1.2_m", layers.by_gap[3])
                      .add("gap_over_1.2_m", layers.by_gap[4]));
  const auto percent = [&](std::size_t n) {
    return layers.columns == 0 ? 0.0 : 100.0 * static_cast<double>(n) / layers.columns;
  };
  log_ << "layers: " << layers.layered_columns << " of " << layers.columns << " voxel columns ("
       << percent(layers.layered_columns) << "%) have a second surface above the lowest; gap "
       << "<0.15 m " << percent(layers.by_gap[0]) << "%, 0.15-0.3 m " << percent(layers.by_gap[1])
       << "%, 0.3-0.6 m " << percent(layers.by_gap[2]) << "%, 0.6-1.2 m "
       << percent(layers.by_gap[3]) << "%, >1.2 m " << percent(layers.by_gap[4]) << "% -> "
       << package_file::kLayers << " (" << seconds_since(start) << " s)\n";
}

}  // namespace vg
