#pragma once

// A map package: a directory holding the 3D map of one capture session and
// the maps derived from it.
//
//   map.vgm         the 3D map (TSDF volume + geo-reference), see volume_io.hpp
//   voxels.ply      its occupied voxels, for viewing
//   surface.asc     top surface height per cell (m)        <- surface stage
//   ground.asc      bare ground height per cell (m)        <- ground stage
//   objects.ply     everything on or over the ground       <- ground stage
//   sun_hours.asc   hours of direct sun per cell           <- sun stage
//   sun_energy.asc  clear-sky direct energy (kWh/m^2)      <- sun stage
//   manifest.json   what each file is, and the settings each stage last ran with
//
// The 3D map is the core: it is built once from the recording (see
// vg::io::build_map_package), and each derived map is a stage that reads only
// map.vgm and the files of the stages before it, so any stage can be re-run
// with new settings without replaying the recording. The sun stage reads
// ground.asc, so re-run it after the ground stage to pick up a new ground.

#include <Eigen/Core>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "vg_core/ground_segmentation.hpp"
#include "vg_core/volume_io.hpp"

namespace vg {

namespace package_file {
inline constexpr const char* kMap = "map.vgm";
inline constexpr const char* kVoxels = "voxels.ply";
inline constexpr const char* kSurface = "surface.asc";
inline constexpr const char* kGround = "ground.asc";
inline constexpr const char* kObjects = "objects.ply";
inline constexpr const char* kSunHours = "sun_hours.asc";
inline constexpr const char* kSunEnergy = "sun_energy.asc";
inline constexpr const char* kManifest = "manifest.json";
}  // namespace package_file

// Settings a stage ran with, recorded in the manifest. Values are kept as JSON
// text; use the add() overloads to build them.
class StageSettings {
 public:
  StageSettings& add(const std::string& key, double value);
  StageSettings& add(const std::string& key, int value);
  StageSettings& add(const std::string& key, std::size_t value);
  StageSettings& add(const std::string& key, bool value);
  StageSettings& add(const std::string& key, const std::string& value);
  StageSettings& add(const std::string& key, const char* value);

  // As a one-line JSON object.
  std::string json() const;

 private:
  std::vector<std::pair<std::string, std::string>> entries_;
};

struct SurfaceOptions {
  // Cell size in meters; the package's if unset.
  std::optional<double> cell_size;
};

struct GroundOptions {
  // config.cell_size is replaced by cell_size, or the package's if unset.
  GroundConfig config;
  std::optional<double> cell_size;
};

struct SunOptions {
  int year = 2026;
  double step_minutes = 60.0;
  // Override the location and heading from the session's GPS.
  std::optional<double> latitude;
  std::optional<double> longitude;
  // Compass bearing of the map's +x axis, in degrees clockwise from north.
  std::optional<double> x_bearing_deg;
  // Height above the ground the sun rays start from, in meters; by default
  // one voxel, and at least 4 cm.
  std::optional<double> sample_height;
};

// Starts a package in dir (created if needed) from a freshly built 3D map:
// clears any earlier package there, writes map.vgm and voxels.ply, and a new
// manifest recording build_settings.
void create_map_package(const std::filesystem::path& dir, const TsdfVolume& volume,
                        const MapMetadata& metadata, const StageSettings& build_settings,
                        std::ostream& log);

// An existing package, for running its derived stages. Progress and timings
// are written to log.
class MapPackage {
 public:
  // Loads dir/map.vgm. Throws std::runtime_error if it can't.
  MapPackage(std::filesystem::path dir, std::ostream& log);

  const std::filesystem::path& dir() const { return dir_; }
  const TsdfVolume& volume() const { return map_.volume; }
  const MapMetadata& metadata() const { return map_.metadata; }

  // surface.asc: the highest surface point per cell.
  void make_surface(const SurfaceOptions& options = {});

  // ground.asc and objects.ply: see segment_ground().
  void make_ground(const GroundOptions& options = {});

  // sun_hours.asc and sun_energy.asc over ground.asc. Returns false, writing
  // nothing, if the package has no ground map or its location is unknown (no
  // GPS in the session and no latitude/longitude given).
  bool make_sun(const SunOptions& options);

 private:
  // All occupied voxels, sorted, computed once.
  const std::vector<Eigen::Vector3i>& occupied();
  std::filesystem::path path(const char* file) const { return dir_ / file; }

  std::filesystem::path dir_;
  std::ostream& log_;
  SavedMap map_;
  std::optional<std::vector<Eigen::Vector3i>> occupied_;
};

// Rewrites dir's manifest.json, recording settings for stage (replacing what
// it last ran with) and keeping the other stages' entries. Done by each stage.
void update_manifest(const std::filesystem::path& dir, const MapMetadata& metadata,
                     double voxel_size, const std::string& stage, const StageSettings& settings);

}  // namespace vg
