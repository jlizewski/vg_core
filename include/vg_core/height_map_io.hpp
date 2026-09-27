#pragma once

// Saving and loading height maps as ESRI ASCII grids (.asc). The format is plain
// text, opens directly in QGIS or any GDAL-based tool, and is easy to read from
// Python. Coordinates are the map's local world frame in meters; the grid is
// not georeferenced.

#include <filesystem>
#include <istream>
#include <ostream>

#include "vg_core/height_map.hpp"

namespace vg {

// Value written for cells with no data.
inline constexpr double kHeightMapNoData = -9999.0;

void write_height_map_ascii(const HeightMap& map, std::ostream& out);

// Throws std::runtime_error if the stream is not a valid ASCII grid.
HeightMap read_height_map_ascii(std::istream& in);

// File wrappers. Throw std::runtime_error if the file can't be opened or written.
void save_height_map(const HeightMap& map, const std::filesystem::path& path);
HeightMap load_height_map(const std::filesystem::path& path);

}  // namespace vg
