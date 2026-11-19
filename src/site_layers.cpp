#include "vg_core/site_layers.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <locale>
#include <sstream>
#include <stdexcept>

#include "vg_core/semantics.hpp"

namespace vg {
namespace {

constexpr double kPi = 3.14159265358979323846;

// A CSV file's rows after its header, split at commas (the layer files
// don't quote). A missing file has none.
std::vector<std::vector<std::string>> read_rows(const std::filesystem::path& path) {
  std::vector<std::vector<std::string>> rows;
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);  // Header.
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (true) {
      const std::size_t comma = line.find(',', start);
      fields.push_back(line.substr(start, comma - start));
      if (comma == std::string::npos) {
        break;
      }
      start = comma + 1;
    }
    rows.push_back(std::move(fields));
  }
  return rows;
}

// Field i of a row, which must be there.
const std::string& field(const std::vector<std::string>& row, std::size_t i,
                         const std::filesystem::path& path) {
  if (i >= row.size()) {
    throw std::runtime_error("site layers: too few fields in " + path.string());
  }
  return row[i];
}

double number(const std::vector<std::string>& row, std::size_t i,
              const std::filesystem::path& path) {
  const std::string& text = field(row, i, path);
  std::istringstream in(text);
  in.imbue(std::locale::classic());
  double value = 0.0;
  if (!(in >> value)) {
    throw std::runtime_error("site layers: bad number '" + text + "' in " + path.string());
  }
  return value;
}

}  // namespace

std::optional<BoundaryEdge> parse_boundary_edge(std::string_view name) {
  for (const BoundaryEdge e : {BoundaryEdge::Structure, BoundaryEdge::GroundEnd,
                               BoundaryEdge::Assumed, BoundaryEdge::Edited}) {
    if (boundary_edge_name(e) == name) {
      return e;
    }
  }
  return std::nullopt;
}

std::optional<FeatureKind> parse_feature_kind(std::string_view name) {
  for (const FeatureKind k : {FeatureKind::Post, FeatureKind::Line, FeatureKind::Block}) {
    if (feature_kind_name(k) == name) {
      return k;
    }
  }
  return std::nullopt;
}

std::optional<GroundEdit::Kind> parse_edit_kind(std::string_view name) {
  for (const GroundEdit::Kind k :
       {GroundEdit::Kind::Ground, GroundEdit::Kind::NotGround, GroundEdit::Kind::Boundary}) {
    if (edit_kind_name(k) == name) {
      return k;
    }
  }
  return std::nullopt;
}

SiteLayers load_site_layers(const std::filesystem::path& site_dir) {
  SiteLayers layers;
  const auto ground_dir = site_dir / ground_layer_file::kDir;
  layers.has_ground = std::filesystem::is_regular_file(ground_dir / ground_layer_file::kHeights) &&
                      std::filesystem::is_regular_file(ground_dir / ground_layer_file::kState);

  // boundary.csv: x,y,edge,push_m
  const auto boundary_dir = site_dir / boundary_layer_file::kDir;
  const auto boundary_file = boundary_dir / boundary_layer_file::kBoundary;
  for (const auto& row : read_rows(boundary_file)) {
    BoundaryVertex v;
    v.position = {number(row, 0, boundary_file), number(row, 1, boundary_file)};
    const auto edge = parse_boundary_edge(field(row, 2, boundary_file));
    if (!edge) {
      throw std::runtime_error("site layers: edge '" + row[2] + "' in " + boundary_file.string());
    }
    v.edge = *edge;
    v.push = row.size() > 3 ? number(row, 3, boundary_file) : 0.0;
    layers.boundary.push_back(v);
  }

  // elements.csv: id,kind,class,center_x,center_y,length_m,width_m,axis_deg,
  // height_m,seen,missed,first_seen,last_seen,status,spread_m
  const auto elements_file = site_dir / feature_layer_file::kDir / feature_layer_file::kElements;
  for (const auto& row : read_rows(elements_file)) {
    SiteFeature e;
    e.id = field(row, 0, elements_file);
    const auto kind = parse_feature_kind(field(row, 1, elements_file));
    if (!kind) {
      throw std::runtime_error("site layers: kind '" + row[1] + "' in " + elements_file.string());
    }
    e.feature.kind = *kind;
    e.feature.label =
        parse_semantic_class(field(row, 2, elements_file)).value_or(SemanticClass::Unknown);
    e.feature.center = {number(row, 3, elements_file), number(row, 4, elements_file)};
    e.feature.length = number(row, 5, elements_file);
    e.feature.width = number(row, 6, elements_file);
    e.feature.axis = number(row, 7, elements_file) * kPi / 180.0;
    e.feature.height = number(row, 8, elements_file);
    e.seen = static_cast<int>(number(row, 9, elements_file));
    e.missed = static_cast<int>(number(row, 10, elements_file));
    e.first_seen = field(row, 11, elements_file);
    e.last_seen = field(row, 12, elements_file);
    e.retired = field(row, 13, elements_file) == "retired";
    e.spread = number(row, 14, elements_file);
    layers.features.push_back(std::move(e));
  }

  // targets.csv: target,order,area_m2,cells,center_x,center_y,seen,spot,
  // spot_x,spot_y,heading_deg,distance_m,sees (one line per spot; spot
  // fields empty for a hole with none). guidance.json: "done".
  const auto guidance_dir = site_dir / guidance_file::kDir;
  const auto summary_file = guidance_dir / guidance_file::kSummary;
  const auto targets_file = guidance_dir / guidance_file::kTargets;
  if (std::filesystem::is_regular_file(summary_file)) {
    layers.has_guidance = true;
    std::ifstream in(summary_file);
    const std::string summary((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
    const std::size_t done = summary.find("\"done\"");
    layers.guidance.done =
        done == std::string::npos || summary.find("true", done) < summary.find(',', done);
  }
  for (const auto& row : read_rows(targets_file)) {
    const int rank = static_cast<int>(number(row, 0, targets_file));
    auto& targets = layers.guidance.targets;
    auto it = std::find_if(targets.begin(), targets.end(),
                           [&](const GuidanceTarget& t) { return t.rank == rank; });
    if (it == targets.end()) {
      GuidanceTarget t;
      t.rank = rank;
      t.order = static_cast<int>(number(row, 1, targets_file));
      t.area = number(row, 2, targets_file);
      t.cells = static_cast<std::size_t>(number(row, 3, targets_file));
      t.center = {number(row, 4, targets_file), number(row, 5, targets_file)};
      t.seen = number(row, 6, targets_file);
      targets.push_back(t);
      it = targets.end() - 1;
    }
    if (!field(row, 7, targets_file).empty()) {
      LookSpot s;
      s.position = {number(row, 8, targets_file), number(row, 9, targets_file)};
      s.heading = number(row, 10, targets_file) * kPi / 180.0;
      s.distance = number(row, 11, targets_file);
      s.sees = number(row, 12, targets_file);
      it->spots.push_back(s);
    }
  }
  std::sort(layers.guidance.targets.begin(), layers.guidance.targets.end(),
            [](const GuidanceTarget& a, const GuidanceTarget& b) { return a.rank < b.rank; });

  // Edits, and the area edits' reviews (edit_reviews.csv:
  // id,kind,cells,disagreeing_cells,walks with walks ;-separated).
  layers.edits = load_ground_edits(ground_dir / ground_layer_file::kEdits);
  for (GroundEdit& e : load_ground_edits(boundary_dir / boundary_layer_file::kEdits)) {
    layers.edits.push_back(std::move(e));
  }
  const auto reviews_file = ground_dir / ground_layer_file::kEditReviews;
  for (const auto& row : read_rows(reviews_file)) {
    GroundLayer::EditReview r;
    r.id = field(row, 0, reviews_file);
    r.kind = parse_edit_kind(field(row, 1, reviews_file)).value_or(GroundEdit::Kind::Ground);
    r.cells = static_cast<std::size_t>(number(row, 2, reviews_file));
    r.disagreeing = static_cast<std::size_t>(number(row, 3, reviews_file));
    std::istringstream walks(row.size() > 4 ? row[4] : std::string());
    for (std::string walk; std::getline(walks, walk, ';');) {
      if (!walk.empty()) {
        r.walks.push_back(walk);
      }
    }
    layers.reviews.push_back(std::move(r));
  }
  return layers;
}

}  // namespace vg
