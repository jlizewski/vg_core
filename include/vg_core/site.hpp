#pragma once

// A site: one place (a yard) mapped over many visits. Each visit is a map
// package (a walk), registered into the site's frame (see registration.hpp),
// so walks on different days line up.
//
//   site.csv            the visits, oldest first: package, when it was
//                       recorded, the transform into the site frame, and how
//                       well it fit
//   current/            the site's maps in the site frame, each cell from the
//                       latest visit that saw it:
//     surface.asc, ground.asc, terrain.asc, features.asc, semantic.asc
//     visit.asc         which visit (1 = the first) each cell came from
//     changes.asc       surface height change at each cell since the visit
//                       before the latest one that saw it (growth, m)
//     canopy.asc, canopy_top.asc, canopy.ply
//                       the canopy from every visit's sky evidence together
//
// The first visit defines the site frame. Each later one is registered
// against all the visits before it, combined. Packages stay where they are;
// site.csv refers to them (relative to the site when it can).

#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "vg_core/registration.hpp"
#include "vg_core/sky.hpp"

namespace vg {

namespace site_file {
inline constexpr const char* kVisits = "site.csv";
inline constexpr const char* kCurrent = "current";
inline constexpr const char* kVisit = "visit.asc";
inline constexpr const char* kChanges = "changes.asc";
}  // namespace site_file

struct SiteVisit {
  // The package directory, as given (absolute, or relative to the site).
  std::filesystem::path package;
  // When it was recorded, YYYYMMDD-HHMMSS, from the recording's name when it
  // has one; empty if not known.
  std::string recorded;
  MapTransform site_from_map;
  // The fit when it was added (empty for the first visit).
  std::optional<RegistrationResult> fit;
};

class Site {
 public:
  // Opens the site in dir (created if needed, with no visits).
  explicit Site(std::filesystem::path dir);

  const std::filesystem::path& dir() const { return dir_; }
  // Oldest first (by recorded time, then the order added).
  const std::vector<SiteVisit>& visits() const { return visits_; }

  // Registers the package (which needs its voxels, features.asc and
  // terrain.asc) against the visits so far and adds it. The first visit is
  // added as is. A fit that fails its checks isn't added, unless force; the
  // result says why. Saves site.csv.
  RegistrationResult add(const std::filesystem::path& package, const RegistrationConfig& config,
                         std::ostream& log, bool force = false);

  // Writes current/: the latest visit's view of each cell, the changes, and
  // the canopy from all visits' sky evidence.
  void make_maps(std::ostream& log, const CanopyConfig& canopy = {}) const;

  // The package directory of a visit, resolved against the site.
  std::filesystem::path package_dir(const SiteVisit& visit) const;

 private:
  void save() const;

  std::filesystem::path dir_;
  std::vector<SiteVisit> visits_;
};

// The YYYYMMDD-HHMMSS stamp in a recording's name, if it has one.
std::optional<std::string> recorded_time(const std::string& session);

}  // namespace vg
