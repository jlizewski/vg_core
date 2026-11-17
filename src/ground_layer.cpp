#include "vg_core/ground_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

#include "vg_core/ground_segmentation.hpp"
#include "vg_core/height_map_io.hpp"
#include "vg_core/map_package.hpp"

namespace vg {
namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr double kDefaultCell = 0.1524;

std::uint8_t to_state(float v) {
  return std::isnan(v) || v < 0.0f || v > 255.0f ? 0 : static_cast<std::uint8_t>(v);
}

double parse_number(const std::string& text, const std::string& what) {
  std::istringstream in(text);
  in.imbue(std::locale::classic());
  double value = 0.0;
  if (!(in >> value)) {
    throw std::runtime_error("ground edits: bad number '" + text + "' in " + what);
  }
  return value;
}

}  // namespace

WalkGround load_walk_ground(const std::filesystem::path& package) {
  for (const char* file :
       {package_file::kGround, package_file::kGroundState, package_file::kGroundWeight}) {
    if (!std::filesystem::is_regular_file(package / file)) {
      throw std::runtime_error("ground layer: " + (package / file).string() +
                               " is missing (re-run the ground stage: vg map " + package.string() +
                               " ground)");
    }
  }
  WalkGround walk;
  walk.id = package.filename().string();
  walk.heights = load_height_map(package / package_file::kGround);
  const HeightMap state = load_height_map(package / package_file::kGroundState);
  const HeightMap weight = load_height_map(package / package_file::kGroundWeight);
  if (state.width != walk.heights.width || state.height != walk.heights.height ||
      weight.width != walk.heights.width || weight.height != walk.heights.height) {
    throw std::runtime_error("ground layer: " + package.string() +
                             ": ground maps of different sizes (re-run its ground stage)");
  }
  walk.state.reserve(state.heights.size());
  walk.weight.reserve(weight.heights.size());
  for (std::size_t i = 0; i < state.heights.size(); ++i) {
    walk.state.push_back(to_state(state.heights[i]));
    walk.weight.push_back(std::isnan(weight.heights[i]) ? 0.0f : weight.heights[i]);
  }
  return walk;
}

bool inside_polygon(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& p) {
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Eigen::Vector2d& a = polygon[i];
    const Eigen::Vector2d& b = polygon[j];
    if ((a.y() > p.y()) != (b.y() > p.y()) &&
        p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x()) {
      inside = !inside;
    }
  }
  return inside;
}

std::vector<GroundEdit> load_ground_edits(const std::filesystem::path& path) {
  std::vector<GroundEdit> edits;
  std::ifstream in(path);
  if (!in) {
    return edits;
  }
  std::string line;
  std::getline(in, line);  // Header.
  int number = 1;
  while (std::getline(in, line)) {
    ++number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    const std::string where = path.filename().string() + " line " + std::to_string(number);
    // id, time, kind, polygon; the note is the rest (it may hold commas).
    std::vector<std::string> fields;
    std::size_t start = 0;
    for (int f = 0; f < 4; ++f) {
      const std::size_t comma = line.find(',', start);
      if (comma == std::string::npos) {
        throw std::runtime_error("ground edits: too few fields in " + where);
      }
      fields.push_back(line.substr(start, comma - start));
      start = comma + 1;
    }
    GroundEdit edit;
    edit.id = fields[0];
    edit.time = fields[1];
    if (fields[2] == "ground") {
      edit.kind = GroundEdit::Kind::Ground;
    } else if (fields[2] == "not_ground") {
      edit.kind = GroundEdit::Kind::NotGround;
    } else {
      throw std::runtime_error("ground edits: kind '" + fields[2] + "' in " + where +
                               " (ground or not_ground)");
    }
    std::istringstream vertices(fields[3]);
    std::string vertex;
    while (std::getline(vertices, vertex, ';')) {
      const std::size_t space = vertex.find(' ');
      if (space == std::string::npos) {
        throw std::runtime_error("ground edits: vertex '" + vertex + "' in " + where + " (x y)");
      }
      edit.polygon.emplace_back(parse_number(vertex.substr(0, space), where),
                                parse_number(vertex.substr(space + 1), where));
    }
    if (edit.polygon.size() < 3) {
      throw std::runtime_error("ground edits: a polygon needs 3 vertices, in " + where);
    }
    edit.note = line.substr(start);
    edits.push_back(std::move(edit));
  }
  return edits;
}

void save_ground_edits(const std::vector<GroundEdit>& edits, const std::filesystem::path& path) {
  std::ofstream out(path);
  out.imbue(std::locale::classic());
  out.precision(10);
  out << "id,time,kind,polygon,note\n";
  for (const GroundEdit& e : edits) {
    out << e.id << ',' << e.time << ','
        << (e.kind == GroundEdit::Kind::Ground ? "ground" : "not_ground") << ',';
    for (std::size_t i = 0; i < e.polygon.size(); ++i) {
      out << (i > 0 ? ";" : "") << e.polygon[i].x() << ' ' << e.polygon[i].y();
    }
    out << ',' << e.note << '\n';
  }
  if (!out) {
    throw std::runtime_error("cannot write " + path.string());
  }
}

GroundLayer estimate_ground(const std::vector<WalkGround>& walks,
                            const std::vector<GroundEdit>& edits, const GroundLayerConfig& config) {
  (void)config;  // One way to combine so far: the latest walk.
  GroundLayer layer;
  HeightMap& map = layer.heights;

  // The site grid: the first walk's cell size, covering every walk.
  double cell = 0.0;
  Eigen::Vector2d lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d hi = -lo;
  for (const WalkGround& w : walks) {
    const HeightMap& h = w.heights;
    if (h.width == 0 || h.height == 0) {
      continue;
    }
    if (cell == 0.0) {
      cell = h.cell_size;
    }
    const Eigen::Vector2d size = Eigen::Vector2d(h.width, h.height) * h.cell_size;
    for (const Eigen::Vector2d& corner : {Eigen::Vector2d(0, 0), Eigen::Vector2d(size.x(), 0),
                                          Eigen::Vector2d(0, size.y()), size}) {
      const Eigen::Vector2d p = w.site_from_map.apply_xy(h.origin + corner);
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
  }
  map.cell_size = cell > 0.0 ? cell : kDefaultCell;
  if (cell == 0.0) {
    return layer;
  }
  map.origin = (lo / cell).array().floor().matrix() * cell;
  map.width = static_cast<int>(std::ceil((hi.x() - map.origin.x()) / cell));
  map.height = static_cast<int>(std::ceil((hi.y() - map.origin.y()) / cell));
  const std::size_t cells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  map.heights.assign(cells, kNaN);
  layer.state.assign(cells, GroundState::Outside);
  layer.provenance.assign(cells, Provenance::None);
  layer.observations.assign(cells, 0);
  layer.source.assign(cells, -1);

  std::vector<MapTransform> map_from_site;
  for (const WalkGround& w : walks) {
    map_from_site.push_back(w.site_from_map.inverse());
  }
  const auto walk_cell = [&](std::size_t k, const Eigen::Vector2d& site) -> std::ptrdiff_t {
    const HeightMap& h = walks[k].heights;
    if (h.width == 0) {
      return -1;
    }
    const Eigen::Vector2d f = (map_from_site[k].apply_xy(site) - h.origin) / h.cell_size;
    const int x = static_cast<int>(std::floor(f.x()));
    const int y = static_cast<int>(std::floor(f.y()));
    if (x < 0 || y < 0 || x >= h.width || y >= h.height) {
      return -1;
    }
    return static_cast<std::ptrdiff_t>(y) * h.width + x;
  };

  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      const std::size_t i = layer.index(x, y);
      const Eigen::Vector2d center = map.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * cell;
      // The latest walk (they're oldest first) in each state, and its height.
      std::array<int, 5> latest;
      latest.fill(-1);
      std::array<float, 5> height{};
      for (std::size_t k = 0; k < walks.size(); ++k) {
        const std::ptrdiff_t c = walk_cell(k, center);
        if (c < 0) {
          continue;
        }
        const std::uint8_t s = walks[k].state[static_cast<std::size_t>(c)];
        if (s == 0 || s > 4) {
          continue;
        }
        latest[s] = static_cast<int>(k);
        height[s] = walks[k].heights.heights[static_cast<std::size_t>(c)] +
                    static_cast<float>(walks[k].site_from_map.translation.z());
        if (s == static_cast<std::uint8_t>(GroundCell::Measured)) {
          ++layer.observations[i];
        }
      }
      const auto take = [&](GroundCell from, GroundState to) {
        const auto s = static_cast<std::size_t>(from);
        if (latest[s] < 0) {
          return false;
        }
        layer.state[i] = to;
        layer.provenance[i] =
            to == GroundState::Measured ? Provenance::Measured : Provenance::Inferred;
        map.heights[i] = height[s];
        layer.source[i] = latest[s];
        return true;
      };
      if (!take(GroundCell::Measured, GroundState::Measured) &&
          !take(GroundCell::UnderObject, GroundState::Covered) &&
          !take(GroundCell::SmallHole, GroundState::Filled) &&
          latest[static_cast<std::size_t>(GroundCell::LargeHole)] >= 0) {
        layer.state[i] = GroundState::Unknown;
      }
    }
  }

  // The user's edits, in order: the ground under them is filled afresh from
  // the ground around them.
  for (const GroundEdit& edit : edits) {
    Eigen::Vector2d a = edit.polygon.front();
    Eigen::Vector2d b = a;
    for (const Eigen::Vector2d& p : edit.polygon) {
      a = a.cwiseMin(p);
      b = b.cwiseMax(p);
    }
    const int x0 = std::max(0, static_cast<int>(std::floor((a.x() - map.origin.x()) / cell)));
    const int y0 = std::max(0, static_cast<int>(std::floor((a.y() - map.origin.y()) / cell)));
    const int x1 =
        std::min(map.width - 1, static_cast<int>(std::floor((b.x() - map.origin.x()) / cell)));
    const int y1 =
        std::min(map.height - 1, static_cast<int>(std::floor((b.y() - map.origin.y()) / cell)));
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const Eigen::Vector2d center = map.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * cell;
        if (!inside_polygon(edit.polygon, center)) {
          continue;
        }
        const std::size_t i = layer.index(x, y);
        if (edit.kind == GroundEdit::Kind::Ground && layer.state[i] == GroundState::Measured) {
          continue;  // Seen as ground already.
        }
        layer.state[i] =
            edit.kind == GroundEdit::Kind::Ground ? GroundState::Filled : GroundState::Covered;
        layer.provenance[i] = Provenance::Edited;
        map.heights[i] = kNaN;
        layer.source[i] = -1;
      }
    }
  }

  // Filled or covered cells with no height yet: smoothly from around them.
  std::vector<char> fill(cells, 0);
  for (std::size_t i = 0; i < cells; ++i) {
    fill[i] = (layer.state[i] == GroundState::Filled || layer.state[i] == GroundState::Covered) &&
              std::isnan(map.heights[i]);
  }
  std::vector<std::uint8_t> filled(cells, 0);
  fill_height_holes(map, fill, filled);
  return layer;
}

}  // namespace vg
