#pragma once

// Saving and loading the 3D map: the TSDF volume itself, with what is known
// about where it came from and where it sits on the Earth. It is the core map
// that every derived map (height, ground, objects, sun) is computed from, so
// those can be recomputed with new settings without replaying the recording.
//
// The file (.vgm) is binary and little-endian: a header, the TsdfConfig, the
// metadata, then every allocated block (index and kBlockSize^3 tsdf/weight
// pairs) in TsdfVolume::block_indices() order, so saving the same map always
// gives the same bytes.

#include <cstddef>
#include <filesystem>
#include <istream>
#include <optional>
#include <ostream>
#include <string>

#include "vg_core/geo_reference.hpp"
#include "vg_core/tsdf_volume.hpp"

namespace vg {

struct MapMetadata {
  // File name of the recording the map was built from.
  std::string session;
  // Cell size of the 2D maps derived from this one, in meters.
  double cell_size = 0.0;
  // Where the map sits on the Earth, from the session's GPS and poses, if it
  // had any GPS.
  std::optional<GeoReference> geo;
  std::size_t frames_integrated = 0;
};

struct SavedMap {
  TsdfVolume volume;
  MapMetadata metadata;
};

void write_map(const TsdfVolume& volume, const MapMetadata& metadata, std::ostream& out);

// Throws std::runtime_error if the stream is not a valid map.
SavedMap read_map(std::istream& in);

// File wrappers. Throw std::runtime_error if the file can't be opened or written.
void save_map(const TsdfVolume& volume, const MapMetadata& metadata,
              const std::filesystem::path& path);
SavedMap load_map(const std::filesystem::path& path);

}  // namespace vg
