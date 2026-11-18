#include "vg_core/ground_layer.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

#include "vg_core/ground_segmentation.hpp"
#include "vg_core/height_map_io.hpp"
#include "vg_core/map_package.hpp"
#include "vg_core/semantics.hpp"

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

// One walk's height for a cell, and how much to trust it.
struct Sample {
  float height = 0.0f;
  float weight = 0.0f;
  int walk = -1;
};

struct Combined {
  float height = 0.0f;
  int source = -1;
  float spread = 0.0f;
};

// The samples (in walk order, oldest first) combined as config says.
Combined combine(std::vector<Sample>& samples, const GroundLayerConfig& config) {
  Combined out;
  if (config.latest_wins) {
    out.height = samples.back().height;
    out.source = samples.back().walk;
  } else {
    // A walk with no weight recorded counts as one observation.
    for (Sample& s : samples) {
      s.weight = s.weight > 0.0f ? s.weight : 1.0f;
    }
    std::vector<Sample> sorted = samples;
    std::stable_sort(sorted.begin(), sorted.end(),
                     [](const Sample& a, const Sample& b) { return a.height < b.height; });
    double total = 0.0;
    for (const Sample& s : sorted) {
      total += s.weight;
    }
    // The weighted median: the lowest height with half the weight at or below it.
    double below = 0.0;
    float median = sorted.back().height;
    for (const Sample& s : sorted) {
      below += s.weight;
      if (below >= 0.5 * total) {
        median = s.height;
        break;
      }
    }
    // The weighted mean of the walks that agree with it.
    double sum = 0.0;
    double weight = 0.0;
    float heaviest = -1.0f;
    for (const Sample& s : samples) {
      if (std::abs(s.height - median) <= config.agree_within) {
        sum += static_cast<double>(s.weight) * s.height;
        weight += s.weight;
        if (s.weight >= heaviest) {  // Ties: the later walk.
          heaviest = s.weight;
          out.source = s.walk;
        }
      }
    }
    out.height = static_cast<float>(sum / weight);
  }
  for (const Sample& s : samples) {
    out.spread = std::max(out.spread, std::abs(s.height - out.height));
  }
  return out;
}

Eigen::Vector2d center_of(const HeightMap& map, int x, int y) {
  return map.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * map.cell_size;
}

double median_of(std::vector<double> v) {
  if (v.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
  std::nth_element(v.begin(), mid, v.end());
  return *mid;
}

// Fewer shared cells than this: no correction at all.
constexpr std::size_t kMinOffsetCells = 30;
// A tilt is only fitted along directions the shared cells spread at least
// this far across (m, standard deviation): a narrow strip can't tell a tilt
// across it.
constexpr double kTiltSpread = 0.5;
// Shared cells farther off the fit than this many robust standard deviations
// (and at least kMinOutlier m) are left out of the next round.
constexpr double kOutlierSigmas = 2.5;
constexpr double kMinOutlier = 0.05;

// The correction (offset and tilt) that best takes differences (x, y, this
// walk's height minus the others') to zero, leaving out outliers.
WalkAlignment fit_alignment(const std::vector<Eigen::Vector3d>& differences,
                            const GroundLayerConfig& config) {
  WalkAlignment a;
  a.cells = differences.size();
  std::vector<double> abs_d;
  for (const Eigen::Vector3d& d : differences) {
    abs_d.push_back(std::abs(d.z()));
  }
  a.before = median_of(abs_d);
  if (differences.size() < kMinOffsetCells) {
    a.after = a.before;
    return a;
  }
  for (const Eigen::Vector3d& d : differences) {
    a.center += d.head<2>();
  }
  a.center /= static_cast<double>(differences.size());
  const bool tilt = static_cast<int>(differences.size()) >= config.align_min_cells;

  std::vector<char> keep(differences.size(), 1);
  const auto residual = [&](const Eigen::Vector3d& d) {
    return d.z() - a.correction_at(d.head<2>());
  };
  const auto fit_offset = [&]() {
    std::vector<double> kept;
    for (std::size_t n = 0; n < differences.size(); ++n) {
      if (keep[n]) {
        kept.push_back(differences[n].z() - a.slope.dot(differences[n].head<2>() - a.center));
      }
    }
    a.offset = median_of(kept);
  };
  for (int round = 0; round < 6; ++round) {
    if (tilt) {
      // Least squares for offset and slope, the slope only along directions
      // the kept cells spread at least kTiltSpread across (the overlap's
      // principal axes): a narrow strip says nothing of a tilt across it.
      Eigen::Matrix2d spread = Eigen::Matrix2d::Zero();
      double n = 0.0;
      for (std::size_t m = 0; m < differences.size(); ++m) {
        if (keep[m]) {
          const Eigen::Vector2d q = differences[m].head<2>() - a.center;
          spread += q * q.transpose();
          n += 1.0;
        }
      }
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> axes(spread / std::max(1.0, n));
      std::vector<Eigen::Vector2d> free;
      for (int e = 0; e < 2; ++e) {
        if (axes.eigenvalues()(e) >= kTiltSpread * kTiltSpread) {
          free.push_back(axes.eigenvectors().col(e));
        }
      }
      const auto unknowns = static_cast<Eigen::Index>(1 + free.size());
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(unknowns, unknowns);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(unknowns);
      for (std::size_t m = 0; m < differences.size(); ++m) {
        if (!keep[m]) {
          continue;
        }
        Eigen::VectorXd row(unknowns);
        row(0) = 1.0;
        for (std::size_t f = 0; f < free.size(); ++f) {
          row(static_cast<Eigen::Index>(f + 1)) = free[f].dot(differences[m].head<2>() - a.center);
        }
        A += row * row.transpose();
        b += row * differences[m].z();
      }
      const Eigen::VectorXd solution = A.ldlt().solve(b);
      a.offset = solution(0);
      a.slope.setZero();
      for (std::size_t f = 0; f < free.size(); ++f) {
        a.slope += solution(static_cast<Eigen::Index>(f + 1)) * free[f];
      }
    } else {
      fit_offset();
    }
    std::vector<double> abs_r;
    for (std::size_t m = 0; m < differences.size(); ++m) {
      if (keep[m]) {
        abs_r.push_back(std::abs(residual(differences[m])));
      }
    }
    const double limit = std::max(kMinOutlier, kOutlierSigmas * 1.4826 * median_of(abs_r));
    for (std::size_t m = 0; m < differences.size(); ++m) {
      keep[m] = std::abs(residual(differences[m])) <= limit ? 1 : 0;
    }
  }
  // Steeper than this isn't drift the ground can show: offset only.
  if (std::atan(a.slope.norm()) > config.align_max_tilt_deg * 3.14159265358979323846 / 180.0) {
    a.slope.setZero();
    fit_offset();
  }
  a.tilted = !a.slope.isZero();
  std::vector<double> abs_r;
  for (const Eigen::Vector3d& d : differences) {
    abs_r.push_back(std::abs(residual(d)));
  }
  a.after = median_of(abs_r);
  return a;
}

// A number from objects.csv; NaN for "nan" or anything unreadable.
double table_number(const std::string& text) {
  char* end = nullptr;
  const double v = std::strtod(text.c_str(), &end);
  return end != text.c_str() ? v : std::numeric_limits<double>::quiet_NaN();
}

// objects.csv, in id order (ids are 1, 2, ...).
std::vector<WalkObject> load_walk_objects(const std::filesystem::path& path) {
  std::vector<WalkObject> objects;
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);  // Header.
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::vector<std::string> f;
    std::istringstream fields(line);
    for (std::string field; std::getline(fields, field, ',');) {
      f.push_back(field);
    }
    if (f.size() < 14) {
      continue;
    }
    WalkObject o;
    o.grounded = f[2] == "1";
    o.label = parse_semantic_class(f[3]).value_or(SemanticClass::Unknown);
    o.top_above_ground = table_number(f[6]);
    o.footprint_area = table_number(f[13]);
    if (f.size() >= 19) {
      o.center = {table_number(f[14]), table_number(f[15])};
      o.length = table_number(f[16]);
      o.width = table_number(f[17]);
      o.axis = table_number(f[18]) * 3.14159265358979323846 / 180.0;
    }
    objects.push_back(o);
  }
  return objects;
}

// elements.csv: id,columns,class,center_x,center_y,length_m,width_m,axis_deg,
// area_m2,top_above_ground_m. Missing: none.
std::vector<WalkObject> load_walk_elements(const std::filesystem::path& path) {
  std::vector<WalkObject> elements;
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);  // Header.
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::vector<std::string> f;
    std::istringstream fields(line);
    for (std::string field; std::getline(fields, field, ',');) {
      f.push_back(field);
    }
    if (f.size() < 10) {
      continue;
    }
    WalkObject e;
    e.grounded = true;
    e.label = parse_semantic_class(f[2]).value_or(SemanticClass::Unknown);
    e.center = {table_number(f[3]), table_number(f[4])};
    e.length = table_number(f[5]);
    e.width = table_number(f[6]);
    e.axis = table_number(f[7]) * 3.14159265358979323846 / 180.0;
    e.footprint_area = table_number(f[8]);
    e.top_above_ground = table_number(f[9]);
    elements.push_back(e);
  }
  return elements;
}

// What a walk saw on a cell it didn't measure as ground.
enum class Over : std::uint8_t { Nothing = 0, Low = 1, Standing = 2 };

// The walk's state at a cell (GroundCell) and the object over it (-1 none,
// -2 not known), as resampled onto the site grid.
Over over_cell(const WalkGround& walk, std::uint8_t state, int object,
               const GroundLayerConfig& config) {
  if (object == -2 || (object >= 0 && static_cast<std::size_t>(object) >= walk.objects.size())) {
    // Not known what's there: under an object counts as covered.
    return state == static_cast<std::uint8_t>(GroundCell::UnderObject) ? Over::Standing
                                                                       : Over::Nothing;
  }
  if (object < 0) {
    return Over::Nothing;
  }
  const WalkObject& o = walk.objects[static_cast<std::size_t>(object)];
  if (!o.grounded || o.label == SemanticClass::Ground) {
    return Over::Nothing;  // Overhangs and stretches of ground don't stand on it.
  }
  const bool tall = o.label == SemanticClass::Structure || o.label == SemanticClass::Tree ||
                    o.top_above_ground >= config.cover_min_height;
  return tall ? Over::Standing : Over::Low;
}

// The digits of a time stamp, YYYYMMDDHHMMSS; empty if it hasn't 14.
std::string stamp_digits(const std::string& time) {
  std::string digits;
  for (const char c : time) {
    if (c >= '0' && c <= '9') {
      digits += c;
    }
  }
  return digits.size() >= 14 ? digits.substr(0, 14) : std::string();
}

}  // namespace

std::string_view edit_kind_name(GroundEdit::Kind kind) {
  switch (kind) {
    case GroundEdit::Kind::Ground:
      return "ground";
    case GroundEdit::Kind::NotGround:
      return "not_ground";
    case GroundEdit::Kind::Boundary:
      return "boundary";
  }
  return "ground";
}

bool recorded_after(const std::string& recorded, const std::string& time) {
  const std::string a = stamp_digits(recorded);
  const std::string b = stamp_digits(time);
  return !a.empty() && !b.empty() && a > b;
}

WalkGround load_walk_ground(const std::filesystem::path& package) {
  for (const char* file :
       {package_file::kGround, package_file::kGroundState, package_file::kGroundWeight,
        package_file::kGroundObject, package_file::kObjectsTable}) {
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
  const HeightMap object = load_height_map(package / package_file::kGroundObject);
  if (state.width != walk.heights.width || state.height != walk.heights.height ||
      weight.width != walk.heights.width || weight.height != walk.heights.height ||
      object.width != walk.heights.width || object.height != walk.heights.height) {
    throw std::runtime_error("ground layer: " + package.string() +
                             ": ground maps of different sizes (re-run its ground stage)");
  }
  walk.state.reserve(state.heights.size());
  walk.weight.reserve(weight.heights.size());
  for (std::size_t i = 0; i < state.heights.size(); ++i) {
    walk.state.push_back(to_state(state.heights[i]));
    walk.weight.push_back(std::isnan(weight.heights[i]) ? 0.0f : weight.heights[i]);
    const float id = object.heights[i];  // 1-based; no data for none.
    walk.object.push_back(std::isnan(id) || id < 1.0f ? -1 : static_cast<int>(id) - 1);
  }
  walk.objects = load_walk_objects(package / package_file::kObjectsTable);
  walk.elements = load_walk_elements(package / package_file::kElements);
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
    } else if (fields[2] == "boundary") {
      edit.kind = GroundEdit::Kind::Boundary;
    } else {
      throw std::runtime_error("ground edits: kind '" + fields[2] + "' in " + where +
                               " (ground, not_ground or boundary)");
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
    if (edit.polygon.size() < (edit.kind == GroundEdit::Kind::Boundary ? 2u : 3u)) {
      throw std::runtime_error(edit.kind == GroundEdit::Kind::Boundary
                                   ? "ground edits: a boundary line needs 2 points, in " + where
                                   : "ground edits: an area needs 3 vertices, in " + where);
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
    out << e.id << ',' << e.time << ',' << edit_kind_name(e.kind) << ',';
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
  layer.spread.assign(cells, kNaN);

  // Each walk on the site grid: what it knew of each cell, the height there
  // in the site frame, and how much it saw it.
  struct OnGrid {
    std::vector<std::uint8_t> state;
    std::vector<float> height;
    std::vector<float> weight;
    std::vector<int> object;  // Index into the walk's objects; -1 for none.
  };
  std::vector<OnGrid> grid(walks.size());
  for (std::size_t k = 0; k < walks.size(); ++k) {
    const WalkGround& w = walks[k];
    OnGrid& g = grid[k];
    g.state.assign(cells, 0);
    g.height.assign(cells, kNaN);
    g.weight.assign(cells, 0.0f);
    g.object.assign(cells, -1);
    const HeightMap& h = w.heights;
    if (h.width == 0) {
      continue;
    }
    const MapTransform map_from_site = w.site_from_map.inverse();
    const auto z = static_cast<float>(w.site_from_map.translation.z());
    for (int y = 0; y < map.height; ++y) {
      for (int x = 0; x < map.width; ++x) {
        const Eigen::Vector2d f =
            (map_from_site.apply_xy(center_of(map, x, y)) - h.origin) / h.cell_size;
        const int mx = static_cast<int>(std::floor(f.x()));
        const int my = static_cast<int>(std::floor(f.y()));
        if (mx < 0 || my < 0 || mx >= h.width || my >= h.height) {
          continue;
        }
        const std::size_t at = static_cast<std::size_t>(my) * static_cast<std::size_t>(h.width) +
                               static_cast<std::size_t>(mx);
        if (w.state[at] == 0 || w.state[at] > 4) {
          continue;
        }
        const std::size_t i = layer.index(x, y);
        g.state[i] = w.state[at];
        g.height[i] = h.heights[at] + z;
        g.weight[i] = w.weight[at];
        g.object[i] = w.object.empty() ? -2 : w.object[at];  // -2: not known.
      }
    }
  }
  const auto cell_center = [&](std::size_t i) {
    return center_of(map, static_cast<int>(i % static_cast<std::size_t>(map.width)),
                     static_cast<int>(i / static_cast<std::size_t>(map.width)));
  };

  // Vertical alignment: the walk that sets the site frame stays; each other
  // walk's ground, oldest first, is moved to meet the walks aligned before it,
  // combined, where both measured the ground.
  constexpr auto kMeasured = static_cast<std::uint8_t>(GroundCell::Measured);
  layer.alignment.assign(walks.size(), WalkAlignment{});
  std::vector<std::size_t> order;
  for (std::size_t k = 0; k < walks.size(); ++k) {
    if (walks[k].sets_frame) {
      order.insert(order.begin(), k);
    } else {
      order.push_back(k);
    }
  }
  std::vector<Sample> before;
  for (std::size_t o = 1; config.align && o < order.size(); ++o) {
    const std::size_t k = order[o];
    std::vector<Eigen::Vector3d> differences;  // x, y, this walk minus the others.
    for (std::size_t i = 0; i < cells; ++i) {
      if (grid[k].state[i] != kMeasured) {
        continue;
      }
      before.clear();
      for (std::size_t p = 0; p < o; ++p) {
        const std::size_t j = order[p];
        if (grid[j].state[i] == kMeasured) {
          before.push_back({grid[j].height[i], grid[j].weight[i], static_cast<int>(j)});
        }
      }
      if (!before.empty()) {
        const Eigen::Vector2d c = cell_center(i);
        differences.emplace_back(c.x(), c.y(), grid[k].height[i] - combine(before, config).height);
      }
    }
    const WalkAlignment a = fit_alignment(differences, config);
    layer.alignment[k] = a;
    for (std::size_t i = 0; i < cells; ++i) {
      if (!std::isnan(grid[k].height[i])) {
        grid[k].height[i] -= static_cast<float>(a.correction_at(cell_center(i)));
      }
    }
  }

  // Cells some walk measured: combined.
  std::vector<Sample> measured;
  for (std::size_t i = 0; i < cells; ++i) {
    measured.clear();
    for (std::size_t k = 0; k < walks.size(); ++k) {
      if (grid[k].state[i] == kMeasured) {
        measured.push_back({grid[k].height[i], grid[k].weight[i], static_cast<int>(k)});
      }
    }
    layer.observations[i] = static_cast<std::uint16_t>(measured.size());
    if (!measured.empty()) {
      const Combined c = combine(measured, config);
      layer.state[i] = GroundState::Measured;
      layer.provenance[i] = Provenance::Measured;
      map.heights[i] = c.height;
      layer.source[i] = c.source;
      layer.spread[i] = c.spread;
    }
  }

  // The rest, by what stands on them in any walk: a structure, a tree or
  // anything tall covers the ground; anything lower (plants) leaves it
  // filled. Overhangs and objects labeled ground don't stand on it.
  std::vector<Over> over(cells, Over::Nothing);
  std::vector<char> known(cells, 0);  // Some walk knew something of the cell.
  for (std::size_t i = 0; i < cells; ++i) {
    if (layer.state[i] == GroundState::Measured) {
      continue;
    }
    for (std::size_t k = 0; k < walks.size(); ++k) {
      const std::uint8_t s = grid[k].state[i];
      if (s == 0) {
        continue;
      }
      known[i] = 1;
      over[i] = std::max(over[i], over_cell(walks[k], s, grid[k].object[i], config));
    }
    if (over[i] != Over::Nothing) {
      layer.state[i] = over[i] == Over::Standing ? GroundState::Covered : GroundState::Filled;
      layer.provenance[i] = Provenance::Inferred;
    }
  }

  // Unseen cells left: regions no walk knew anything of that reach the map's
  // edge are outside; everything else is a hole, filled if small.
  const auto open = [&](std::size_t i) {
    return layer.state[i] == GroundState::Outside;  // Not classified yet.
  };
  std::vector<char> visited(cells, 0);
  std::vector<std::size_t> region;
  const auto grow = [&](std::size_t start, auto&& joins) {
    region.assign(1, start);
    visited[start] = 1;
    bool edge = false;
    for (std::size_t m = 0; m < region.size(); ++m) {
      const int x = static_cast<int>(region[m] % static_cast<std::size_t>(map.width));
      const int y = static_cast<int>(region[m] / static_cast<std::size_t>(map.width));
      edge = edge || x == 0 || y == 0 || x == map.width - 1 || y == map.height - 1;
      const std::array<std::array<int, 2>, 4> steps = {{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
      for (const auto& d : steps) {
        const int nx = x + d[0];
        const int ny = y + d[1];
        if (nx < 0 || ny < 0 || nx >= map.width || ny >= map.height) {
          continue;
        }
        const std::size_t n = layer.index(nx, ny);
        if (!visited[n] && joins(n)) {
          visited[n] = 1;
          region.push_back(n);
        }
      }
    }
    return edge;
  };
  std::vector<char> outside(cells, 0);
  for (std::size_t i = 0; i < cells; ++i) {
    if (open(i) && !known[i] && !visited[i]) {
      const bool edge = grow(i, [&](std::size_t n) { return open(n) && !known[n]; });
      for (const std::size_t r : region) {
        outside[r] = edge ? 1 : 0;
      }
    }
  }
  std::fill(visited.begin(), visited.end(), 0);
  const double cell_area = cell * cell;
  for (std::size_t i = 0; i < cells; ++i) {
    if (open(i) && !outside[i] && !visited[i]) {
      grow(i, [&](std::size_t n) { return open(n) && !outside[n]; });
      const bool small = static_cast<double>(region.size()) * cell_area <= config.max_fill_area;
      for (const std::size_t r : region) {
        layer.state[r] = small ? GroundState::Filled : GroundState::Unknown;
        layer.provenance[r] = small ? Provenance::Inferred : Provenance::None;
      }
    }
  }

  // The user's edits, in order: the ground under them is filled afresh from
  // the ground around them.
  for (const GroundEdit& edit : edits) {
    if (edit.kind == GroundEdit::Kind::Boundary) {
      continue;  // The boundary's (boundary.hpp).
    }
    GroundLayer::EditReview review;
    review.id = edit.id;
    review.kind = edit.kind;
    std::vector<char> disagrees(walks.size(), 0);
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
        // Walks recorded after the edit that disagree with it here.
        ++review.cells;
        bool disagreeing = false;
        for (std::size_t k = 0; k < walks.size(); ++k) {
          if (!recorded_after(walks[k].recorded, edit.time) || grid[k].state[i] == 0) {
            continue;
          }
          const bool against = edit.kind == GroundEdit::Kind::NotGround
                                   ? grid[k].state[i] == kMeasured
                                   : over_cell(walks[k], grid[k].state[i], grid[k].object[i],
                                               config) == Over::Standing;
          if (against) {
            disagrees[k] = 1;
            disagreeing = true;
          }
        }
        review.disagreeing += disagreeing ? 1 : 0;
        if (edit.kind == GroundEdit::Kind::Ground && layer.state[i] == GroundState::Measured) {
          continue;  // Seen as ground already.
        }
        layer.state[i] =
            edit.kind == GroundEdit::Kind::Ground ? GroundState::Filled : GroundState::Covered;
        layer.provenance[i] = Provenance::Edited;
        map.heights[i] = kNaN;
        layer.source[i] = -1;
        layer.spread[i] = kNaN;
      }
    }
    for (std::size_t k = 0; k < walks.size(); ++k) {
      if (disagrees[k]) {
        review.walks.push_back(walks[k].id);
      }
    }
    layer.reviews.push_back(std::move(review));
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
