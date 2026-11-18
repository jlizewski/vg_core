#include "vg_core/config/jobs.hpp"

#include <algorithm>
#include <locale>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <fstream>
#include <string>
#include <map>
#include <utility>

#include "vg_core/config/mapping_config.hpp"
#include "vg_core/io/map_build.hpp"
#include "vg_core/map_package.hpp"
#include "vg_core/site.hpp"

namespace vg::config {

namespace {

// Whether the package has the ground stage outputs the site's layers need.
bool objects_table_current(const std::filesystem::path& dir) {
  std::ifstream in(dir / package_file::kObjectsTable);
  std::string header;
  std::getline(in, header);
  return header.find("axis_deg") != std::string::npos &&
         std::filesystem::is_regular_file(dir / package_file::kElements);
}

// Runs the stages a package needs for registration that it hasn't run yet,
// with the settings it was made with.
void prepare_for_site(const std::filesystem::path& dir, std::ostream& log) {
  std::vector<std::string> stages;
  for (const auto& [file, stage] :
       {std::pair{package_file::kSurface, "surface"}, std::pair{package_file::kGroundState, "ground"},
        std::pair{package_file::kTerrain, "terrain"}}) {
    if (!stages.empty() || !std::filesystem::is_regular_file(dir / file) ||
        (std::string(stage) == "ground" && !objects_table_current(dir))) {
      stages.push_back(stage);  // And everything after it.
    }
  }
  if (stages.empty()) {
    return;
  }
  MappingConfig config;
  ParamSet params = bind(config);
  if (std::filesystem::exists(dir / package_file::kConfig)) {
    params.load_yaml_file((dir / package_file::kConfig).string());
  }
  MapPackage package(dir, log);
  run_stages(package, config, stages);
  save_settings(params, dir / package_file::kConfig, kPackageHeader);
}

}  // namespace

bool replay(const std::filesystem::path& session, const std::filesystem::path& package_dir,
            const std::vector<std::string>& settings, std::ostream& log) {
  MappingConfig config;
  ParamSet params = bind(config);
  params.apply_args(settings);
  config.validate();

  const io::BuildOptions build = config.build_options();
  const io::BuildSummary summary = io::build_map_package(session, package_dir, build, log);
  // Record what was worked out, so config.yaml says what the map was made with.
  config.voxel_size = build.builder.tsdf.voxel_size;
  config.truncation = build.builder.tsdf.truncation_distance;
  config.build.min_weight = summary.min_weight;
  config.year = config.sun_options().year;

  MapPackage package(package_dir, log);
  const bool all_ran = run_stages(package, config, config.pipeline);
  save_settings(params, package.dir() / package_file::kConfig, kPackageHeader);
  log << "map package written to " << package_dir.string() << "\n";
  return all_ran;
}

bool rerun_stages(const std::filesystem::path& package_dir, std::vector<std::string> stages,
                  const std::vector<std::string>& settings, std::ostream& log) {
  MappingConfig config;
  ParamSet params = bind(config);
  const auto saved = package_dir / package_file::kConfig;
  if (std::filesystem::exists(saved)) {
    params.load_yaml_file(saved.string());
  } else {
    log << "note: " << saved.string()
        << " not found (an older package); starting from the defaults\n";
  }
  // map and build settings describe the saved 3D map; they can't change here.
  std::map<std::string, std::string> built;
  for (const std::string& path : params.paths()) {
    if (is_build_setting(path)) {
      built[path] = params.get(path);
    }
  }
  params.apply_args(settings);
  for (const auto& [path, value] : built) {
    if (params.get(path) != value) {
      throw ConfigError(path + " is a build setting: rebuild with vg replay to change it");
    }
  }
  if (std::find(stages.begin(), stages.end(), "all") != stages.end()) {
    stages = config.pipeline;
  }
  config.validate();
  check_stages(stages);
  const auto wants = [&](const std::string& stage) {
    return std::find(stages.begin(), stages.end(), stage) != stages.end();
  };

  MapPackage package(package_dir, log);
  config.year = config.sun_options().year;
  const bool all_ran = run_stages(package, config, stages);
  if (wants("ground") && !wants("sun") &&
      std::filesystem::exists(package.dir() / package_file::kSunHours)) {
    log << "note: the sun maps were made over the previous ground; run the sun stage to update "
           "them\n";
  }
  save_settings(params, package.dir() / package_file::kConfig, kPackageHeader);
  return all_ran;
}

bool add_to_site(const std::filesystem::path& site_dir,
                 const std::vector<std::filesystem::path>& packages, bool force,
                 const std::vector<std::string>& settings, std::ostream& log) {
  MappingConfig config;
  ParamSet params = bind(config);
  params.apply_args(settings);
  config.validate();

  Site site(site_dir);
  bool all_added = true;
  for (const auto& package : packages) {
    prepare_for_site(package, log);
    const RegistrationResult result = site.add(package, config.registration, log, force);
    all_added = all_added && (result.accepted || force);
  }
  site.make_maps(log, config.sky.config);
  // The layers, so current/ and the model show the site's ground.
  make_site_ground(site_dir, settings, log);
  return all_added;
}

std::string add_site_edit(const std::filesystem::path& site_dir, const std::string& kind,
                          const std::string& points, const std::string& note,
                          std::ostream& log) {
  GroundEdit e;
  if (kind == "ground") {
    e.kind = GroundEdit::Kind::Ground;
  } else if (kind == "not_ground") {
    e.kind = GroundEdit::Kind::NotGround;
  } else if (kind == "boundary") {
    e.kind = GroundEdit::Kind::Boundary;
  } else {
    throw ConfigError("edit: kind must be ground, not_ground or boundary, not '" + kind + "'");
  }
  std::istringstream vertices(points);
  for (std::string vertex; std::getline(vertices, vertex, ';');) {
    std::istringstream xy(vertex);
    xy.imbue(std::locale::classic());
    double x = 0.0;
    double y = 0.0;
    if (!(xy >> x >> y)) {
      throw ConfigError("edit: point '" + vertex + "' isn't `x y`");
    }
    e.polygon.emplace_back(x, y);
  }
  const bool line = e.kind == GroundEdit::Kind::Boundary;
  if (e.polygon.size() < (line ? 2u : 3u)) {
    throw ConfigError(line ? "edit: a boundary line needs at least 2 points"
                           : "edit: an area needs at least 3 points");
  }
  if (note.find('\n') != std::string::npos) {
    throw ConfigError("edit: the note must be one line");
  }
  e.note = note;
  // The time now, UTC, ISO 8601.
  const std::time_t now = std::time(nullptr);
  std::tm utc{};
#ifdef _WIN32
  gmtime_s(&utc, &now);
#else
  gmtime_r(&now, &utc);
#endif
  std::ostringstream time;
  time << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  e.time = time.str();

  // Ids run across both layers' edits.
  const auto ground_file = site_dir / ground_layer_file::kDir / ground_layer_file::kEdits;
  const auto boundary_file = site_dir / boundary_layer_file::kDir / boundary_layer_file::kEdits;
  std::vector<GroundEdit> ground = load_ground_edits(ground_file);
  std::vector<GroundEdit> boundary = load_ground_edits(boundary_file);
  e.id = "edit-" + std::to_string(ground.size() + boundary.size() + 1);
  std::vector<GroundEdit>& into = line ? boundary : ground;
  const auto& file = line ? boundary_file : ground_file;
  into.push_back(e);
  std::filesystem::create_directories(file.parent_path());
  save_ground_edits(into, file);
  log << "edit: " << e.id << " (" << kind << ", " << e.polygon.size() << " points) -> "
      << file.string() << std::endl;
  make_site_ground(site_dir, {}, log);
  return e.id;
}

void make_site_ground(const std::filesystem::path& site_dir,
                      const std::vector<std::string>& settings, std::ostream& log) {
  MappingConfig config;
  ParamSet params = bind(config);
  params.apply_args(settings);
  config.validate();
  const Site site(site_dir);
  for (const SiteVisit& visit : site.visits()) {
    prepare_for_site(site.package_dir(visit), log);
  }
  site.make_ground_layer(log, config.ground_layer, config.boundary, config.guidance);
  site.make_feature_layer(log, config.features);
}

void make_site_maps(const std::filesystem::path& site_dir,
                    const std::vector<std::string>& settings, std::ostream& log) {
  MappingConfig config;
  ParamSet params = bind(config);
  params.apply_args(settings);
  config.validate();
  Site(site_dir).make_maps(log, config.sky.config);
  make_site_ground(site_dir, settings, log);
}

}  // namespace vg::config
