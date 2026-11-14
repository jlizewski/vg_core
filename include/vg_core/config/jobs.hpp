#pragma once

// The pipeline's jobs as functions: what vg replay, vg map and vg site do,
// callable from any program (the command-line apps, the C API, a desktop or
// phone app). Each takes its settings as command-line arguments, applied in
// order over the defaults (params.hpp):
//
//   --config FILE     a YAML settings file
//   --set PATH=VALUE  any setting
//   --FLAG VALUE      a setting's short flag
//
// and writes its progress to log. Bad settings throw ConfigError before any
// work starts; other failures (a recording or package that can't be read,
// a file that can't be written) throw std::runtime_error.

#include <filesystem>
#include <ostream>
#include <string>
#include <vector>

namespace vg::config {

// vg replay: builds a map package in package_dir from a recording (replacing
// any package there), runs the stages in the `pipeline` setting, and saves
// the settings it was made with in the package's config.yaml. Returns false
// if a stage couldn't run (the sun stage, with no location).
bool replay(const std::filesystem::path& session, const std::filesystem::path& package_dir,
            const std::vector<std::string>& settings, std::ostream& log);

// vg map: re-runs derived stages of a package from its saved 3D map. Settings
// start from the package's config.yaml; build settings can't change (that
// needs replay), and trying throws ConfigError. "all" runs the `pipeline`
// setting. Returns false if a stage couldn't run.
bool rerun_stages(const std::filesystem::path& package_dir, std::vector<std::string> stages,
                  const std::vector<std::string>& settings, std::ostream& log);

// vg site add: registers each package into the site in site_dir (created if
// needed), running the stages registration needs on any package missing
// them, then remakes the site's current maps. A package that doesn't line up
// convincingly isn't added unless force. Returns false if one wasn't added.
bool add_to_site(const std::filesystem::path& site_dir,
                 const std::vector<std::filesystem::path>& packages, bool force,
                 const std::vector<std::string>& settings, std::ostream& log);

// vg site maps: remakes the site's current maps.
void make_site_maps(const std::filesystem::path& site_dir,
                    const std::vector<std::string>& settings, std::ostream& log);

}  // namespace vg::config
