#include "vg_core/terrain.hpp"

#include <Eigen/Sparse>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace vg {
namespace {

// A very weak pull of each node toward its neighbors' height (sigma of a first
// difference, in meters): keeps the problem solvable where only the bending
// penalty reaches, without flattening real slopes.
constexpr double kFlattenSigma = 2.0;
// How far the solved phone height may stray from its first estimate, in meters.
constexpr double kPhoneHeightSigma = 0.5;

struct Sample {
  Eigen::Vector2d xy;
  double z;  // Ground height, or camera height for a path point.
};

// One linear observation: sum of coefficients x unknowns = value.
struct Row {
  std::array<int, 5> index{};
  std::array<double, 5> coefficient{};
  int size = 0;
  double value = 0.0;
  double weight = 0.0;

  void add(int i, double c) {
    index[static_cast<std::size_t>(size)] = i;
    coefficient[static_cast<std::size_t>(size)] = c;
    ++size;
  }
};

double median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  return *mid;
}

// Bilinear interpolation: the four nodes around p and their weights.
void bilinear(const TerrainModel& model, const Eigen::Vector2d& p, Row& row) {
  const Eigen::Vector2d f = (p - model.origin) / model.spacing;
  const int x0 = std::clamp(static_cast<int>(std::floor(f.x())), 0, std::max(0, model.width - 2));
  const int y0 = std::clamp(static_cast<int>(std::floor(f.y())), 0, std::max(0, model.height - 2));
  const double tx = std::clamp(f.x() - x0, 0.0, 1.0);
  const double ty = std::clamp(f.y() - y0, 0.0, 1.0);
  const int x1 = std::min(x0 + 1, model.width - 1);
  const int y1 = std::min(y0 + 1, model.height - 1);
  row.add(y0 * model.width + x0, (1 - tx) * (1 - ty));
  row.add(y0 * model.width + x1, tx * (1 - ty));
  row.add(y1 * model.width + x0, (1 - tx) * ty);
  row.add(y1 * model.width + x1, tx * ty);
}

// Solves the weighted least squares of rows over n unknowns.
Eigen::VectorXd solve(const std::vector<Row>& rows, int n) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(rows.size() * 16);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n);
  for (const Row& row : rows) {
    if (row.weight <= 0.0) {
      continue;
    }
    for (int a = 0; a < row.size; ++a) {
      const auto ia = static_cast<std::size_t>(a);
      rhs[row.index[ia]] += row.weight * row.coefficient[ia] * row.value;
      for (int b = 0; b < row.size; ++b) {
        const auto ib = static_cast<std::size_t>(b);
        triplets.emplace_back(row.index[ia], row.index[ib],
                              row.weight * row.coefficient[ia] * row.coefficient[ib]);
      }
    }
  }
  Eigen::SparseMatrix<double> normal(n, n);
  normal.setFromTriplets(triplets.begin(), triplets.end());
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(normal);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("terrain: the fit is singular");
  }
  return solver.solve(rhs);
}

}  // namespace

double TerrainModel::at(const Eigen::Vector2d& p) const {
  Row row;
  bilinear(*this, p, row);
  double z = 0.0;
  for (int i = 0; i < row.size; ++i) {
    const auto k = static_cast<std::size_t>(i);
    z += row.coefficient[k] * nodes[static_cast<std::size_t>(row.index[k])];
  }
  return z;
}

Terrain fit_terrain(const HeightMap& ground, const std::vector<std::uint8_t>& patched,
                    const std::vector<Eigen::Vector3d>& path, const TerrainConfig& config) {
  Terrain terrain;
  const double cell = ground.cell_size > 0.0 ? ground.cell_size : config.node_spacing;

  // Seen ground cells, by center.
  std::vector<Sample> seen;
  for (int y = 0; y < ground.height; ++y) {
    for (int x = 0; x < ground.width; ++x) {
      const std::size_t i =
          static_cast<std::size_t>(y) * static_cast<std::size_t>(ground.width) +
          static_cast<std::size_t>(x);
      if (std::isnan(ground.heights[i]) || (!patched.empty() && patched[i] != 0)) {
        continue;
      }
      seen.push_back({ground.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * cell,
                      static_cast<double>(ground.heights[i])});
    }
  }
  // Path points, path_spacing apart.
  std::vector<Sample> walk;
  if (config.use_path) {
    for (const Eigen::Vector3d& p : path) {
      if (walk.empty() || (p.head<2>() - walk.back().xy).norm() >= config.path_spacing) {
        walk.push_back({p.head<2>(), p.z()});
      }
    }
  }
  terrain.ground_cells = seen.size();
  terrain.path_points = walk.size();
  terrain.heights.cell_size = cell;
  terrain.phone_height = config.phone_height.value_or(config.default_phone_height);
  if (seen.empty() && walk.empty()) {
    return terrain;
  }

  // The output grid: the ground map's cells, extended over the path.
  Eigen::Vector2d lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d hi = -lo;
  if (ground.width > 0 && ground.height > 0) {
    lo = ground.origin;
    hi = ground.origin + Eigen::Vector2d(ground.width, ground.height) * cell;
  }
  for (const Sample& s : walk) {
    lo = lo.cwiseMin(s.xy - Eigen::Vector2d::Constant(config.reach));
    hi = hi.cwiseMax(s.xy + Eigen::Vector2d::Constant(config.reach));
  }
  // Aligned to the ground map's grid (or the global grid without one).
  const Eigen::Vector2d anchor = ground.width > 0 ? ground.origin : Eigen::Vector2d::Zero();
  const Eigen::Vector2d first = anchor + ((lo - anchor) / cell).array().floor().matrix() * cell;
  HeightMap& out = terrain.heights;
  out.origin = first;
  out.cell_size = cell;
  out.width = static_cast<int>(std::ceil((hi.x() - first.x()) / cell - 1e-9));
  out.height = static_cast<int>(std::ceil((hi.y() - first.y()) / cell - 1e-9));

  // Nodes spanning the output grid.
  TerrainModel& model = terrain.model;
  model.spacing = config.node_spacing;
  model.origin = (first / model.spacing).array().floor().matrix() * model.spacing;
  const Eigen::Vector2d end = first + Eigen::Vector2d(out.width, out.height) * cell;
  model.width = std::max(2, static_cast<int>(std::ceil((end.x() - model.origin.x()) /
                                                       model.spacing - 1e-9)) + 1);
  model.height = std::max(2, static_cast<int>(std::ceil((end.y() - model.origin.y()) /
                                                        model.spacing - 1e-9)) + 1);
  const int nodes = model.width * model.height;
  model.nodes.assign(static_cast<std::size_t>(nodes), 0.0);

  // The phone's height: given, or first estimated where the path crosses seen
  // ground, then solved for along with the terrain.
  std::vector<double> over_ground;
  for (const Sample& s : walk) {
    const Eigen::Vector2d f = (s.xy - ground.origin) / cell;
    const int x = static_cast<int>(std::floor(f.x()));
    const int y = static_cast<int>(std::floor(f.y()));
    if (x < 0 || y < 0 || x >= ground.width || y >= ground.height) {
      continue;
    }
    const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(ground.width) +
                          static_cast<std::size_t>(x);
    if (!std::isnan(ground.heights[i]) && (patched.empty() || patched[i] == 0)) {
      over_ground.push_back(s.z - ground.heights[i]);
    }
  }
  double phone_height = config.phone_height.value_or(
      over_ground.empty() ? config.default_phone_height : median(over_ground));
  const bool solve_phone = !config.phone_height && !over_ground.empty() && !seen.empty();
  const int phone = nodes;  // Its unknown's index, when solved for.

  // Rows that don't change: bending and flattening.
  std::vector<Row> fixed;
  const auto node = [&](int x, int y) { return y * model.width + x; };
  const double bend_weight = 1.0 / (config.bend * config.bend);
  const double flatten_weight = 1.0 / (kFlattenSigma * kFlattenSigma);
  for (int y = 0; y < model.height; ++y) {
    for (int x = 0; x < model.width; ++x) {
      if (x + 2 < model.width) {
        Row r;
        r.add(node(x, y), 1);
        r.add(node(x + 1, y), -2);
        r.add(node(x + 2, y), 1);
        r.weight = bend_weight;
        fixed.push_back(r);
      }
      if (y + 2 < model.height) {
        Row r;
        r.add(node(x, y), 1);
        r.add(node(x, y + 1), -2);
        r.add(node(x, y + 2), 1);
        r.weight = bend_weight;
        fixed.push_back(r);
      }
      if (x + 1 < model.width && y + 1 < model.height) {
        // Twist, weighted as in a thin plate.
        Row r;
        r.add(node(x, y), 1);
        r.add(node(x + 1, y), -1);
        r.add(node(x, y + 1), -1);
        r.add(node(x + 1, y + 1), 1);
        r.weight = 2.0 * bend_weight;
        fixed.push_back(r);
      }
      for (const auto& [dx, dy] : {std::pair{1, 0}, std::pair{0, 1}}) {
        if (x + dx < model.width && y + dy < model.height) {
          Row r;
          r.add(node(x, y), 1);
          r.add(node(x + dx, y + dy), -1);
          r.weight = flatten_weight;
          fixed.push_back(r);
        }
      }
    }
  }

  // Interpolation rows of the evidence (weights set per round).
  std::vector<Row> ground_rows(seen.size());
  for (std::size_t i = 0; i < seen.size(); ++i) {
    bilinear(model, seen[i].xy, ground_rows[i]);
  }
  std::vector<Row> path_rows(walk.size());
  for (std::size_t i = 0; i < walk.size(); ++i) {
    bilinear(model, walk[i].xy, path_rows[i]);
  }
  const auto fitted = [&](const Row& row) {
    double z = 0.0;
    for (int k = 0; k < row.size; ++k) {
      const auto j = static_cast<std::size_t>(k);
      z += row.coefficient[j] * model.nodes[static_cast<std::size_t>(row.index[j])];
    }
    return z;
  };
  // Geman-McClure weights, scaled to be a half at r = scale: they fall off
  // fast enough that a whole patch of foliage can't outvote the walk.
  const auto robust = [](double r, double scale) {
    const double k = (std::sqrt(2.0) - 1.0) * (r / scale) * (r / scale);
    return 1.0 / ((1.0 + k) * (1.0 + k));
  };
  // A node's area of seen ground counts as one observation: the cells' errors
  // (foliage, a missed bump) are shared, not independent.
  const double ground_weight = (cell / model.spacing) * (cell / model.spacing) /
                               (config.ground_sigma * config.ground_sigma);
  const double path_weight = 1.0 / (config.path_sigma * config.path_sigma);
  std::vector<double> ground_robust(seen.size(), 1.0);
  std::vector<double> path_robust(walk.size(), 1.0);

  // One fit with the current robust weights; with_ground false fits the path
  // alone (at the first estimate of the phone height).
  const auto fit = [&](bool with_ground) {
    const bool solving_phone = solve_phone && with_ground;
    const int unknowns = nodes + (solving_phone ? 1 : 0);
    std::vector<Row> rows = fixed;
    rows.reserve(fixed.size() + seen.size() + walk.size() + 1);
    if (with_ground) {
      for (std::size_t i = 0; i < seen.size(); ++i) {
        Row r = ground_rows[i];
        r.value = seen[i].z;
        r.weight = ground_weight * ground_robust[i];
        rows.push_back(r);
      }
    }
    for (std::size_t i = 0; i < walk.size(); ++i) {
      // terrain + phone height = camera height.
      Row r = path_rows[i];
      r.value = walk[i].z - (solving_phone ? 0.0 : phone_height);
      if (solving_phone) {
        r.add(phone, 1);
      }
      r.weight = path_weight * path_robust[i];
      rows.push_back(r);
    }
    if (solving_phone) {
      Row r;
      r.add(phone, 1);
      r.value = phone_height;
      r.weight = 1.0 / (kPhoneHeightSigma * kPhoneHeightSigma);
      rows.push_back(r);
    }
    const Eigen::VectorXd x = solve(rows, unknowns);
    for (int i = 0; i < nodes; ++i) {
      model.nodes[static_cast<std::size_t>(i)] = x[i];
    }
    if (solving_phone) {
      phone_height = x[phone];
    }
  };
  const auto reweight = [&]() {
    for (std::size_t i = 0; i < seen.size(); ++i) {
      const double r = seen[i].z - fitted(ground_rows[i]);
      ground_robust[i] = robust(r, r > 0.0 ? config.above_scale : config.below_scale);
    }
    for (std::size_t i = 0; i < walk.size(); ++i) {
      const double r = walk[i].z - phone_height - fitted(path_rows[i]);
      path_robust[i] = robust(r, config.path_scale);
    }
  };

  // Start from the path, so ground is judged against where the walk puts it.
  if (!walk.empty() && !seen.empty()) {
    fit(false);
    reweight();
  }
  fit(true);
  for (int round = 0; round < config.iterations; ++round) {
    reweight();
    fit(true);
  }
  terrain.phone_height = phone_height;
  terrain.phone_height_solved = solve_phone;

  // How well the evidence fits.
  std::vector<double> residuals;
  for (std::size_t i = 0; i < seen.size(); ++i) {
    const double r = seen[i].z - fitted(ground_rows[i]);
    residuals.push_back(std::abs(r));
    const double w = robust(r, r > 0.0 ? config.above_scale : config.below_scale);
    if (w < 0.5) {
      ++terrain.ground_cells_discounted;
      terrain.ground_cells_discounted_above += r > 0.0 ? 1 : 0;
    }
  }
  terrain.ground_residual = median(residuals);
  residuals.clear();
  for (std::size_t i = 0; i < walk.size(); ++i) {
    residuals.push_back(std::abs(walk[i].z - phone_height - fitted(path_rows[i])));
  }
  terrain.path_residual = median(residuals);

  // The terrain on the output cells within reach of evidence.
  const auto w = static_cast<std::size_t>(out.width);
  std::vector<std::uint8_t> near(w * static_cast<std::size_t>(out.height), 0);
  std::vector<std::uint8_t> stamped(near.size(), 0);
  const int radius = static_cast<int>(std::ceil(config.reach / cell));
  const auto stamp = [&](const Eigen::Vector2d& p) {
    const Eigen::Vector2d f = (p - out.origin) / cell;
    const int cx = static_cast<int>(std::floor(f.x()));
    const int cy = static_cast<int>(std::floor(f.y()));
    if (cx < 0 || cy < 0 || cx >= out.width || cy >= out.height ||
        stamped[static_cast<std::size_t>(cy) * w + static_cast<std::size_t>(cx)] != 0) {
      return;
    }
    stamped[static_cast<std::size_t>(cy) * w + static_cast<std::size_t>(cx)] = 1;
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const int x = cx + dx;
        const int y = cy + dy;
        if (x >= 0 && y >= 0 && x < out.width && y < out.height &&
            std::hypot(dx, dy) * cell <= config.reach) {
          near[static_cast<std::size_t>(y) * w + static_cast<std::size_t>(x)] = 1;
        }
      }
    }
  };
  for (const Sample& s : seen) {
    stamp(s.xy);
  }
  for (const Sample& s : walk) {
    stamp(s.xy);
  }
  out.heights.assign(near.size(), std::numeric_limits<float>::quiet_NaN());
  for (int y = 0; y < out.height; ++y) {
    for (int x = 0; x < out.width; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * w + static_cast<std::size_t>(x);
      if (near[i] != 0) {
        out.heights[i] = static_cast<float>(
            model.at(out.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * cell));
      }
    }
  }
  return terrain;
}

}  // namespace vg
