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
//     ground.asc, ground_state.asc
//                       with a ground layer, its ground and state per cell
//     model.glb         the site in 3D (Site::model), for viewing
//   layers/             the site's layers (docs/design/guided-mapping.md):
//     ground/           the ground, combined from every visit, with the
//                       user's edits (ground_layer.hpp)
//     features/         posts, lines and blocks, matched across visits
//                       (features.hpp)
//     boundary/         the site's edge, from its boundary walks (boundary.hpp)
//     guidance/         where to go and look to finish the ground (guidance.hpp)
//
// The first visit defines the site frame. Each later one is registered
// against all the visits before it, combined. Packages stay where they are;
// site.csv refers to them (relative to the site when it can).

#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "vg_core/boundary.hpp"
#include "vg_core/features.hpp"
#include "vg_core/ground_layer.hpp"
#include "vg_core/guidance.hpp"
#include "vg_core/registration.hpp"
#include "vg_core/sky.hpp"
#include "vg_core/surface_mesh.hpp"

namespace vg {

class MapPackage;

namespace site_file {
inline constexpr const char* kVisits = "site.csv";
inline constexpr const char* kCurrent = "current";
inline constexpr const char* kVisit = "visit.asc";
inline constexpr const char* kChanges = "changes.asc";
inline constexpr const char* kModel = "model.glb";
inline constexpr const char* kGroundState = "ground_state.asc";
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

  // Registers the package (as add) against every visit, without adding it:
  // where a walk sits in the site, e.g. to check one before adding it, or to
  // view it over the site's maps. Throws std::runtime_error if the site has no
  // visits.
  RegistrationResult locate(const std::filesystem::path& package, const RegistrationConfig& config,
                            std::ostream& log) const;

  // Writes current/: the latest visit's view of each cell, the changes, and
  // the canopy from all visits' sky evidence.
  void make_maps(std::ostream& log, const CanopyConfig& canopy = {}) const;

  // Builds the ground layer (ground_layer.hpp) from every visit's ground stage
  // outputs and the edits in layers/ground/edits.csv, and writes it there;
  // and, with boundary walks among the visits, the site's boundary
  // (boundary.hpp, layers/boundary/), outside which the ground is Outside.
  // Throws std::runtime_error if a visit's ground outputs are missing.
  // Then plans ground-fill guidance for it (guidance.hpp, layers/guidance/).
  GroundLayer make_ground_layer(std::ostream& log, const GroundLayerConfig& config = {},
                                const BoundaryConfig& boundary = {},
                                const GuidanceConfig& guidance = {}) const;

  // Builds the feature elements (features.hpp) from every visit's objects,
  // matched across visits, and writes them to layers/features/.
  FeatureLayer make_feature_layer(std::ostream& log, const FeatureConfig& config = {}) const;

  // The site in 3D. With a ground layer (make_ground_layer): its clean
  // ground ("ground": colored by the class seen from above where measured, a
  // lighter tint where filled, dark where covered, yellow over unknown holes,
  // their heights filled in for showing), what stands on it ("objects": the
  // surface where it's over the ground, with sides down to it), the canopy's
  // underside over the terrain, see-through ("canopy"), and the boundary as a
  // low white kerb ("boundary"). Without one, from current/ (make_maps): the
  // surface colored by class ("surface") and the canopy. Parts whose maps are
  // missing are empty; the first is the ground (or surface).
  std::vector<MeshPart> model() const;

  // Writes model() to current/model.glb.
  void write_model(std::ostream& log) const;

  // The package directory of a visit, resolved against the site.
  std::filesystem::path package_dir(const SiteVisit& visit) const;

 private:
  void save() const;
  // The site's boundary from its boundary walks, applied to ground (cells
  // outside it Outside) and written to layers/boundary/.
  void make_boundary(GroundLayer& ground, const BoundaryConfig& config, std::ostream& log) const;
  // Registers the package against every visit, or, given a known alignment,
  // checks how well it fits there (evaluate_alignment()).
  RegistrationResult fit(const std::filesystem::path& package, MapPackage& map,
                         const RegistrationInput& input, const RegistrationConfig& config,
                         std::ostream& log,
                         const std::optional<MapTransform>& known = std::nullopt) const;

  std::filesystem::path dir_;
  std::vector<SiteVisit> visits_;
};

// The YYYYMMDD-HHMMSS stamp in a recording's name, if it has one.
std::optional<std::string> recorded_time(const std::string& session);

}  // namespace vg
