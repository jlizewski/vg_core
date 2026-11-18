#include "vg_core/boundary.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "vg_core/map_package.hpp"

namespace vg {
namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Twice the signed area of a polygon (positive: counter-clockwise).
double twice_area(const std::vector<Eigen::Vector2d>& p) {
  double sum = 0.0;
  for (std::size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
    sum += p[j].x() * p[i].y() - p[i].x() * p[j].y();
  }
  return sum;
}

// The ground layer's state at a site-frame point (Outside beyond the grid).
GroundState state_at(const GroundLayer& ground, const Eigen::Vector2d& p) {
  const HeightMap& map = ground.heights;
  if (map.width == 0) {
    return GroundState::Outside;
  }
  const Eigen::Vector2d f = (p - map.origin) / map.cell_size;
  const int x = static_cast<int>(std::floor(f.x()));
  const int y = static_cast<int>(std::floor(f.y()));
  if (x < 0 || y < 0 || x >= map.width || y >= map.height) {
    return GroundState::Outside;
  }
  return ground.state[ground.index(x, y)];
}

// Distance from p to a closed polygon's outline.
double distance_to_outline(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& p) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Eigen::Vector2d a = polygon[j];
    const Eigen::Vector2d ab = polygon[i] - a;
    const double len2 = ab.squaredNorm();
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    best = std::min(best, (a + t * ab - p).norm());
  }
  return best;
}

std::vector<Eigen::Vector2d> positions(const Boundary& b) {
  std::vector<Eigen::Vector2d> out;
  out.reserve(b.vertices.size());
  for (const BoundaryVertex& v : b.vertices) {
    out.push_back(v.position);
  }
  return out;
}

double quantile(std::vector<double> v, double q) {
  if (v.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto k = static_cast<std::size_t>(q * static_cast<double>(v.size() - 1));
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
  return v[k];
}

// Where segments ab and cd cross, if they do (not at shared ends).
bool crossing(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c,
              const Eigen::Vector2d& d, Eigen::Vector2d& at) {
  const Eigen::Vector2d r = b - a;
  const Eigen::Vector2d s = d - c;
  const double denom = r.x() * s.y() - r.y() * s.x();
  if (std::abs(denom) < 1e-12) {
    return false;
  }
  const Eigen::Vector2d q = c - a;
  const double t = (q.x() * s.y() - q.y() * s.x()) / denom;
  const double u = (q.x() * r.y() - q.y() * r.x()) / denom;
  if (t <= 1e-9 || t >= 1.0 - 1e-9 || u <= 1e-9 || u >= 1.0 - 1e-9) {
    return false;
  }
  at = a + t * r;
  return true;
}

// Cuts the small loops where an outline crosses itself (pushing points out
// round a tight corner can make them): the vertices between the crossing
// edges go, the crossing point takes their place. Only loops of up to
// kMaxLoop edges are cut: a big one is the walk's own shape.
void remove_loops(std::vector<BoundaryVertex>& v) {
  constexpr std::size_t kMaxLoop = 24;
  bool cut = true;
  while (cut && v.size() > 4) {
    cut = false;
    const std::size_t n = v.size();
    for (std::size_t i = 0; i < n && !cut; ++i) {
      for (std::size_t k = 2; k <= kMaxLoop && k + 1 < n && !cut; ++k) {
        const std::size_t j = (i + k) % n;
        Eigen::Vector2d at;
        if (!crossing(v[i].position, v[(i + 1) % n].position, v[j].position,
                      v[(j + 1) % n].position, at)) {
          continue;
        }
        // Keep i, put the crossing after it, drop i+1..j.
        BoundaryVertex x = v[(i + 1) % n];
        x.position = at;
        std::vector<BoundaryVertex> kept;
        kept.reserve(n - k + 1);
        for (std::size_t m = 0; m < n; ++m) {
          const std::size_t offset = (m + n - i) % n;  // Steps after i.
          if (offset >= 1 && offset <= k) {
            if (offset == 1) {
              kept.push_back(x);
            }
            continue;
          }
          kept.push_back(v[m]);
        }
        v = std::move(kept);
        cut = true;
      }
    }
  }
}

}  // namespace

std::string_view boundary_edge_name(BoundaryEdge edge) {
  switch (edge) {
    case BoundaryEdge::Structure:
      return "structure";
    case BoundaryEdge::GroundEnd:
      return "ground_end";
    case BoundaryEdge::Assumed:
      return "assumed";
    case BoundaryEdge::Edited:
      return "edited";
  }
  return "assumed";
}

LoopClosure close_loop(const std::vector<Eigen::Vector2d>& path, const BoundaryConfig& config) {
  LoopClosure loop;
  if (path.size() < 3) {
    return loop;
  }
  std::vector<double> along(path.size(), 0.0);  // Distance walked to each point.
  for (std::size_t i = 1; i < path.size(); ++i) {
    along[i] = along[i - 1] + (path[i] - path[i - 1]).norm();
  }
  // Closest to the start, once the walk has been well away from it.
  double away = 0.0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double d = (path[i] - path.front()).norm();
    away = std::max(away, d);
    // Coming back: at least min_away nearer the start than the farthest so far.
    if (away - d >= config.min_away && d < best) {
      best = d;
      loop.end = i;
    }
  }
  if (!(best <= config.close_distance)) {
    loop.end = path.size() - 1;
    loop.length = along.back();
    return loop;
  }
  loop.closed = true;
  loop.gap = best;
  loop.length = along[loop.end];
  // The walk's start, against its overlap with it at the end (from a little
  // before the closest point on).
  double sum = 0.0;
  std::size_t n = 0;
  for (std::size_t i = 0; i < path.size() && along[i] <= config.overlap_length; ++i) {
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t j = loop.end;
         j-- > 0 && along[j] >= along[loop.end] - config.overlap_length;) {
      nearest = std::min(nearest, (path[j] - path[i]).norm());
    }
    for (std::size_t j = loop.end; j < path.size(); ++j) {
      nearest = std::min(nearest, (path[j] - path[i]).norm());
    }
    sum += nearest;
    ++n;
  }
  loop.overlap_offset = n > 0 ? sum / static_cast<double>(n) : loop.gap;
  loop.drift_per_100m = loop.length > 0.0 ? 100.0 * loop.gap / loop.length : 0.0;
  return loop;
}

Boundary estimate_boundary(const std::vector<Eigen::Vector2d>& path, const GroundLayer& ground,
                           const BoundaryConfig& config) {
  Boundary boundary;
  boundary.loop = close_loop(path, config);
  if (!boundary.loop.closed) {
    return boundary;
  }

  // The loop, resampled every sample_spacing, closed back to its start.
  std::vector<Eigen::Vector2d> loop(
      path.begin(), path.begin() + static_cast<std::ptrdiff_t>(boundary.loop.end) + 1);
  loop.push_back(path.front());
  std::vector<Eigen::Vector2d> ring{loop.front()};
  double carried = 0.0;  // Walked since the last sample.
  for (std::size_t i = 1; i < loop.size(); ++i) {
    Eigen::Vector2d a = loop[i - 1];
    const Eigen::Vector2d b = loop[i];
    double left = (b - a).norm();
    while (carried + left >= config.sample_spacing) {
      const double step = config.sample_spacing - carried;
      a += (b - a).normalized() * step;
      left -= step;
      carried = 0.0;
      ring.push_back(a);
    }
    carried += left;
  }
  if (ring.size() > 1 && (ring.back() - ring.front()).norm() < 0.5 * config.sample_spacing) {
    ring.pop_back();
  }
  if (ring.size() < 3) {
    boundary.loop.closed = false;
    return boundary;
  }
  if (twice_area(ring) < 0.0) {
    std::reverse(ring.begin(), ring.end());
  }

  // Each point pushed out to the edge the walker followed.
  const std::size_t m = ring.size();
  const double step = std::max(0.05, 0.5 * ground.heights.cell_size);
  std::vector<Eigen::Vector2d> outward(m);
  std::vector<std::pair<double, BoundaryEdge>> found(m);
  for (std::size_t j = 0; j < m; ++j) {
    const auto k = static_cast<std::size_t>(
        std::max(1.0, std::round(config.normal_span / config.sample_spacing)));
    const Eigen::Vector2d t = (ring[(j + k) % m] - ring[(j + m - k % m) % m]).normalized();
    outward[j] = Eigen::Vector2d(t.y(), -t.x());  // Right of the way round: outside.
    found[j] = {config.assumed_offset, BoundaryEdge::Assumed};
    for (double d = 0.0; d <= config.max_push + 1e-9; d += step) {
      const GroundState s = state_at(ground, ring[j] + outward[j] * d);
      if (s == GroundState::Covered) {
        found[j] = {d, BoundaryEdge::Structure};
        break;
      }
      if (s == GroundState::Outside || s == GroundState::Unknown) {
        found[j] = {std::max(0.0, d - step), BoundaryEdge::GroundEnd};
        break;
      }
    }
  }
  // A running median of the pushes (with how each was found) smooths spikes.
  const int half = std::max(0, config.smooth_window / 2);
  std::vector<std::pair<double, BoundaryEdge>> window;
  for (std::size_t j = 0; j < m; ++j) {
    window.clear();
    for (int k = -half; k <= half; ++k) {
      window.push_back(found[(j + m + static_cast<std::size_t>(k + static_cast<int>(m))) % m]);
    }
    std::nth_element(window.begin(), window.begin() + half, window.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
    BoundaryVertex v;
    v.push = window[static_cast<std::size_t>(half)].first;
    v.edge = window[static_cast<std::size_t>(half)].second;
    v.position = ring[j] + outward[j] * v.push;
    boundary.vertices.push_back(v);
  }

  remove_loops(boundary.vertices);
  const std::vector<Eigen::Vector2d> outline = positions(boundary);
  const std::size_t m2 = outline.size();
  boundary.area = 0.5 * std::abs(twice_area(outline));
  for (std::size_t i = 0, j = m2 - 1; i < m2; j = i++) {
    boundary.perimeter += (outline[i] - outline[j]).norm();
  }
  return boundary;
}

BoundaryAgreement compare_boundaries(const Boundary& a, const Boundary& b, double cell) {
  BoundaryAgreement agreement;
  const std::vector<Eigen::Vector2d> pa = positions(a);
  const std::vector<Eigen::Vector2d> pb = positions(b);
  if (pa.size() < 3 || pb.size() < 3) {
    agreement.median_distance = agreement.p90_distance = std::numeric_limits<double>::quiet_NaN();
    return agreement;
  }
  // Distances from points along each outline to the other's.
  std::vector<double> distances;
  for (const auto& [from, to] : {std::pair{&pa, &pb}, std::pair{&pb, &pa}}) {
    for (std::size_t i = 0, j = from->size() - 1; i < from->size(); j = i++) {
      const Eigen::Vector2d p = (*from)[j];
      const Eigen::Vector2d d = (*from)[i] - p;
      const int steps = std::max(1, static_cast<int>(std::ceil(d.norm() / cell)));
      for (int s = 0; s < steps; ++s) {
        distances.push_back(distance_to_outline(*to, p + d * (static_cast<double>(s) / steps)));
      }
    }
  }
  agreement.median_distance = quantile(distances, 0.5);
  agreement.p90_distance = quantile(distances, 0.9);

  // Overlap of the areas, on a raster.
  Eigen::Vector2d lo = pa.front();
  Eigen::Vector2d hi = lo;
  for (const auto* poly : {&pa, &pb}) {
    for (const Eigen::Vector2d& p : *poly) {
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
  }
  std::size_t both = 0;
  std::size_t either = 0;
  for (double y = lo.y() + cell / 2; y < hi.y(); y += cell) {
    for (double x = lo.x() + cell / 2; x < hi.x(); x += cell) {
      const bool in_a = inside_polygon(pa, {x, y});
      const bool in_b = inside_polygon(pb, {x, y});
      both += in_a && in_b ? 1 : 0;
      either += in_a || in_b ? 1 : 0;
    }
  }
  agreement.overlap = either > 0 ? static_cast<double>(both) / static_cast<double>(either) : 0.0;
  return agreement;
}

std::size_t edit_boundary(Boundary& boundary, const std::vector<GroundEdit>& edits) {
  std::size_t applied = 0;
  for (const GroundEdit& edit : edits) {
    std::vector<BoundaryVertex>& v = boundary.vertices;
    if (edit.kind != GroundEdit::Kind::Boundary || edit.polygon.size() < 2 || v.size() < 3) {
      continue;
    }
    const std::vector<Eigen::Vector2d>& line = edit.polygon;
    const auto nearest = [&](const Eigen::Vector2d& p) {
      std::size_t best = 0;
      for (std::size_t i = 1; i < v.size(); ++i) {
        if ((v[i].position - p).squaredNorm() < (v[best].position - p).squaredNorm()) {
          best = i;
        }
      }
      return best;
    };
    const std::size_t a = nearest(line.front());
    const std::size_t b = nearest(line.back());
    const std::size_t n = v.size();
    if (a == b) {
      continue;
    }
    // The two ways round from a to b (forward, a..b) and back, by how near
    // their vertices lie to the line, on average.
    const auto from_line = [&](std::size_t i) {
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t k = 1; k < line.size(); ++k) {
        const Eigen::Vector2d ab = line[k] - line[k - 1];
        const double len2 = ab.squaredNorm();
        const double t =
            len2 > 0.0 ? std::clamp((v[i].position - line[k - 1]).dot(ab) / len2, 0.0, 1.0) : 0.0;
        best = std::min(best, (line[k - 1] + t * ab - v[i].position).norm());
      }
      return best;
    };
    const auto mean_from_line = [&](std::size_t from, std::size_t to) {
      double sum = 0.0;
      std::size_t count = 0;
      for (std::size_t i = from;; i = (i + 1) % n) {
        sum += from_line(i);
        ++count;
        if (i == to) {
          break;
        }
      }
      return sum / static_cast<double>(count);
    };
    const bool forward = mean_from_line(a, b) <= mean_from_line(b, a);
    // Keep the other way round, then the line closes the ring.
    std::vector<BoundaryVertex> kept;
    std::vector<Eigen::Vector2d> drawn = line;
    if (forward) {  // Replace a..b: keep b..a, then the line from a's end to b's.
      for (std::size_t i = b;; i = (i + 1) % n) {
        kept.push_back(v[i]);
        if (i == a) {
          break;
        }
      }
    } else {  // Replace b..a: keep a..b, then the line back from b's end to a's.
      for (std::size_t i = a;; i = (i + 1) % n) {
        kept.push_back(v[i]);
        if (i == b) {
          break;
        }
      }
      std::reverse(drawn.begin(), drawn.end());
    }
    // The kept stretch's end vertices give way to the line's own ends.
    kept.pop_back();
    kept.erase(kept.begin());
    for (const Eigen::Vector2d& p : drawn) {
      BoundaryVertex x;
      x.position = p;
      x.edge = BoundaryEdge::Edited;
      kept.push_back(x);
    }
    v = std::move(kept);
    std::vector<Eigen::Vector2d> ring = positions(boundary);
    if (twice_area(ring) < 0.0) {
      std::reverse(v.begin(), v.end());
      ring = positions(boundary);
    }
    boundary.area = 0.5 * std::abs(twice_area(ring));
    boundary.perimeter = 0.0;
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
      boundary.perimeter += (ring[i] - ring[j]).norm();
    }
    ++applied;
  }
  return applied;
}

void apply_boundary(GroundLayer& ground, const Boundary& boundary) {
  const std::vector<Eigen::Vector2d> outline = positions(boundary);
  if (outline.size() < 3) {
    return;
  }
  const HeightMap& map = ground.heights;
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      const Eigen::Vector2d c = map.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * map.cell_size;
      if (inside_polygon(outline, c)) {
        continue;
      }
      const std::size_t i = ground.index(x, y);
      ground.state[i] = GroundState::Outside;
      ground.provenance[i] = Provenance::None;
      ground.heights.heights[i] = kNaN;
      ground.source[i] = -1;
      ground.spread[i] = kNaN;
    }
  }
}

std::vector<Eigen::Vector2d> load_path(const std::filesystem::path& package) {
  std::ifstream in(package / package_file::kTrajectory);
  if (!in) {
    throw std::runtime_error("boundary: no " + (package / package_file::kTrajectory).string());
  }
  std::vector<Eigen::Vector2d> path;
  std::string line;
  std::getline(in, line);  // Header: t,x,y,z.
  while (std::getline(in, line)) {
    std::istringstream fields(line);
    fields.imbue(std::locale::classic());
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
    char comma = 0;
    if (fields >> t >> comma >> x >> comma >> y) {
      path.emplace_back(x, y);
    }
  }
  return path;
}

}  // namespace vg
