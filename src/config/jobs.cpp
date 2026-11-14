#include "vg_core/config/jobs.hpp"

#include <algorithm>
#include <map>
#include <utility>

#include "vg_core/config/mapping_config.hpp"
#include "vg_core/io/map_build.hpp"
#include "vg_core/map_package.hpp"
#include "vg_core/site.hpp"

namespace vg::config {

namespace {

// Runs the stages a package needs for registration that it hasn't run yet,
// with the settings it was made with.
void prepare_for_site(const std::filesystem::path& dir, std::ostream& log) {
  std::vector<std::string> stages;
  for (const auto& [file, stage] :
       {std::pair{package_file::kSurface, "surface"}, std::pair{package_file::kGround, "ground"},
        std::pair{package_file::kTerrain, "terrain"}}) {
    if (!stages.empty() || !std::filesystem::is_regular_file(dir / file)) {
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
  return all_added;
}

void make_site_maps(const std::filesystem::path& site_dir,
                    const std::vector<std::string>& settings, std::ostream& log) {
  MappingConfig config;
  ParamSet params = bind(config);
  params.apply_args(settings);
  config.validate();
  Site(site_dir).make_maps(log, config.sky.config);
}

}  // namespace vg::config
