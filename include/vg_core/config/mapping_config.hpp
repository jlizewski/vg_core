#pragma once

// Every setting of the map package pipeline (vg_core/map_package.hpp), in one
// struct, and its binding to YAML paths and command-line flags (params.hpp).
//
// The YAML has a section per part of the pipeline: map, build (replaying the
// recording into the 3D map), voxels (reading the 3D map out), maps, ground,
// sun, layers; and the pipeline itself, the derived stages to run in order.
// config/mapping.yaml in the repository lists every setting with its default
// (`vg config defaults` writes it). A map package keeps the settings it was
// made with in its config.yaml.
//
// To add a setting: give the option struct a field, bind it in bind() with a
// path and help (and a flag if it's often changed), and regenerate
// config/mapping.yaml.

#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "vg_core/config/params.hpp"
#include "vg_core/ground_segmentation.hpp"
#include "vg_core/io/map_build.hpp"
#include "vg_core/map_package.hpp"

namespace vg::config {

// The derived stages run_stages() knows, in their natural order.
inline const std::vector<std::string> kStages = {"voxels", "surface", "ground", "sun",
                                                 "semantic", "layers"};

struct MappingConfig {
  // The defaults, as vg replay uses them.
  MappingConfig();

  // map: cell size of the 2D maps, in meters.
  double resolution = 0.1524;

  // build: replaying the recording. Unset optionals are worked out from the
  // others (see build_options()).
  std::optional<double> voxel_size;
  std::optional<double> truncation;
  std::optional<double> to;
  io::BuildOptions build;

  // voxels: reading the 3D map out (build.min_weight and friends live in
  // BuildOptions, which picks the automatic value).
  double min_blob_volume = MapPackage::kDefaultMinBlobVolume;

  // build.semantics: classes whose readings are left out of the map (names,
  // see semantics.hpp); semantics: votes a voxel's class needs to count.
  std::vector<std::string> drop_classes;
  int min_label_votes = 2;

  // maps, ground, sun, layers.
  std::optional<double> cell_size;
  GroundConfig ground;
  SunOptions sun;
  std::optional<int> year;
  LayersOptions layers;

  // Derived stages vg replay runs after the build, in order.
  std::vector<std::string> pipeline;

  // Throws ConfigError if a setting names an unknown stage or class.
  void validate() const;

  // The options each part of the pipeline takes, with automatic values
  // worked out.
  io::BuildOptions build_options() const;
  SurfaceOptions surface_options() const;
  GroundOptions ground_options() const;
  SunOptions sun_options() const;
  LayersOptions layers_options() const;
};

// Binds every setting of config. config must outlive the result.
ParamSet bind(MappingConfig& config);

// Whether a setting at path is read by the build, so only takes effect when
// the 3D map is rebuilt (vg replay), not when stages are re-run (vg map).
bool is_build_setting(const std::string& path);

// Throws ConfigError unless every name is in kStages.
void check_stages(const std::vector<std::string>& stages);

// Runs the named derived stages on package in order, with config's voxel
// read-out settings. Returns false if the sun stage was skipped (it logs why:
// no location). The semantic stage skipping a map without labels is fine.
bool run_stages(MapPackage& package, const MappingConfig& config,
                const std::vector<std::string>& stages);

// Writes the settings to file as YAML, for a package's config.yaml.
void save_settings(const ParamSet& params, const std::filesystem::path& file,
                   const std::string& header);

// The header of config/mapping.yaml (`vg config defaults`).
extern const char* const kDefaultsHeader;
// The header of a package's config.yaml.
extern const char* const kPackageHeader;

}  // namespace vg::config
