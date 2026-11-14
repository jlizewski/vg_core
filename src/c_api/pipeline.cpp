// The C API's pipeline jobs: vg::config's jobs (jobs.hpp) with their log
// passed to a callback.

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "common.hpp"
#include "vg_core/config/jobs.hpp"

using vg::c_api::guard;
using vg::c_api::strings;

vg_status vg_replay(const char* session, const char* package_dir, const char* const* settings,
                    size_t settings_count, vg_log_fn log, void* user_data, int32_t* complete) {
  return guard([&] {
    const auto session_path = vg::c_api::path_of(session, "session");
    const auto package_path = vg::c_api::path_of(package_dir, "package_dir");
    const auto args = strings(settings, settings_count, "settings");
    vg::c_api::Log out(log, user_data);
    const bool all_ran = vg::config::replay(session_path, package_path, args, out.stream());
    if (complete != nullptr) {
      *complete = all_ran ? 1 : 0;
    }
  });
}

vg_status vg_map_stages(const char* package_dir, const char* const* stages, size_t stage_count,
                        const char* const* settings, size_t settings_count, vg_log_fn log,
                        void* user_data, int32_t* complete) {
  return guard([&] {
    const auto package_path = vg::c_api::path_of(package_dir, "package_dir");
    auto names = strings(stages, stage_count, "stages");
    if (names.empty()) {
      throw std::invalid_argument("no stages given");
    }
    const auto args = strings(settings, settings_count, "settings");
    vg::c_api::Log out(log, user_data);
    const bool all_ran =
        vg::config::rerun_stages(package_path, std::move(names), args, out.stream());
    if (complete != nullptr) {
      *complete = all_ran ? 1 : 0;
    }
  });
}

vg_status vg_site_add(const char* site_dir, const char* const* package_dirs, size_t package_count,
                      int32_t force, const char* const* settings, size_t settings_count,
                      vg_log_fn log, void* user_data, int32_t* all_added) {
  return guard([&] {
    const auto site_path = vg::c_api::path_of(site_dir, "site_dir");
    std::vector<std::filesystem::path> packages;
    for (const std::string& dir : strings(package_dirs, package_count, "package_dirs")) {
      packages.push_back(std::filesystem::u8path(dir));
    }
    if (packages.empty()) {
      throw std::invalid_argument("no packages given");
    }
    const auto args = strings(settings, settings_count, "settings");
    vg::c_api::Log out(log, user_data);
    const bool added = vg::config::add_to_site(site_path, packages, force != 0, args, out.stream());
    if (all_added != nullptr) {
      *all_added = added ? 1 : 0;
    }
  });
}

vg_status vg_site_make_maps(const char* site_dir, const char* const* settings,
                            size_t settings_count, vg_log_fn log, void* user_data) {
  return guard([&] {
    const auto site_path = vg::c_api::path_of(site_dir, "site_dir");
    const auto args = strings(settings, settings_count, "settings");
    vg::c_api::Log out(log, user_data);
    vg::config::make_site_maps(site_path, args, out.stream());
  });
}
