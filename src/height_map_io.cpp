#include "vg_core/height_map_io.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <limits>
#include <locale>
#include <stdexcept>
#include <string>

namespace vg {
namespace {

std::string lowercase(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

}  // namespace

void write_height_map_ascii(const HeightMap& map, std::ostream& out) {
  out.imbue(std::locale::classic());
  // 15 significant digits round-trips typical header values (e.g. 0.05)
  // without printing binary noise.
  out.precision(15);
  out << "ncols " << map.width << '\n'
      << "nrows " << map.height << '\n'
      << "xllcorner " << map.origin.x() << '\n'
      << "yllcorner " << map.origin.y() << '\n'
      << "cellsize " << map.cell_size << '\n'
      << "NODATA_value " << kHeightMapNoData << '\n';

  // The grid's first row is the top (highest y); ours is row 0 = lowest y.
  // Heights to 7 significant digits: well under a millimeter for any garden.
  out.precision(7);
  for (int y = map.height - 1; y >= 0; --y) {
    for (int x = 0; x < map.width; ++x) {
      if (x > 0) {
        out << ' ';
      }
      if (map.has_data(x, y)) {
        out << map.at(x, y);
      } else {
        out << kHeightMapNoData;
      }
    }
    out << '\n';
  }
}

HeightMap read_height_map_ascii(std::istream& in) {
  in.imbue(std::locale::classic());
  HeightMap map;
  bool center = false;
  double nodata = kHeightMapNoData;

  // Header: six "key value" lines, NODATA_value optional.
  for (int field = 0; field < 6; ++field) {
    const auto pos = in.tellg();
    std::string key;
    if (!(in >> key)) {
      throw std::runtime_error("height map: truncated header");
    }
    key = lowercase(key);
    if (key == "ncols") {
      in >> map.width;
    } else if (key == "nrows") {
      in >> map.height;
    } else if (key == "xllcorner" || key == "xllcenter") {
      in >> map.origin.x();
      center = key == "xllcenter";
    } else if (key == "yllcorner" || key == "yllcenter") {
      in >> map.origin.y();
    } else if (key == "cellsize") {
      in >> map.cell_size;
    } else if (key == "nodata_value") {
      in >> nodata;
    } else {
      // No NODATA line: this was the first grid value.
      in.seekg(pos);
      break;
    }
    if (!in) {
      throw std::runtime_error("height map: bad value for " + key);
    }
  }
  if (map.width <= 0 || map.height <= 0 || map.cell_size <= 0.0) {
    throw std::runtime_error("height map: missing or invalid ncols, nrows or cellsize");
  }
  if (center) {
    map.origin.array() -= map.cell_size / 2.0;
  }

  map.heights.assign(static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height),
                     std::numeric_limits<float>::quiet_NaN());
  for (int y = map.height - 1; y >= 0; --y) {
    for (int x = 0; x < map.width; ++x) {
      double value = 0.0;
      if (!(in >> value)) {
        throw std::runtime_error("height map: fewer values than ncols * nrows");
      }
      if (value != nodata) {
        map.heights[static_cast<std::size_t>(y) * static_cast<std::size_t>(map.width) +
                    static_cast<std::size_t>(x)] = static_cast<float>(value);
      }
    }
  }
  return map;
}

void save_height_map(const HeightMap& map, const std::filesystem::path& path) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("height map: cannot open " + path.string() + " for writing");
  }
  write_height_map_ascii(map, out);
  out.flush();
  if (!out) {
    throw std::runtime_error("height map: failed writing " + path.string());
  }
}

HeightMap load_height_map(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("height map: cannot open " + path.string());
  }
  return read_height_map_ascii(in);
}

}  // namespace vg
