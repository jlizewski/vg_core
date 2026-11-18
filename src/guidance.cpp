#include "vg_core/guidance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace vg {
namespace {

struct Cell {
  int x = 0;
  int y = 0;
};

}  // namespace

Guidance plan_guidance(const GroundLayer& ground, const Eigen::Vector2d& start,
                       const GuidanceConfig& config) {
  Guidance guidance;
  const HeightMap& map = ground.heights;
  if (map.width == 0 || map.height == 0) {
    return guidance;
  }
  const double cell = map.cell_size;
  const auto state = [&](int x, int y) { return ground.state[ground.index(x, y)]; };
  const auto center = [&](int x, int y) {
    return Eigen::Vector2d(map.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * cell);
  };

  // The holes: 4-connected regions of unknown cells.
  std::vector<std::vector<Cell>> holes;
  std::vector<char> visited(ground.state.size(), 0);
  const std::array<std::array<int, 2>, 4> steps = {{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      if (visited[ground.index(x, y)] || state(x, y) != GroundState::Unknown) {
        continue;
      }
      std::vector<Cell> hole{{x, y}};
      visited[ground.index(x, y)] = 1;
      for (std::size_t m = 0; m < hole.size(); ++m) {
        for (const auto& d : steps) {
          const int nx = hole[m].x + d[0];
          const int ny = hole[m].y + d[1];
          if (nx >= 0 && ny >= 0 && nx < map.width && ny < map.height &&
              !visited[ground.index(nx, ny)] && state(nx, ny) == GroundState::Unknown) {
            visited[ground.index(nx, ny)] = 1;
            hole.push_back({nx, ny});
          }
        }
      }
      holes.push_back(std::move(hole));
    }
  }
  std::stable_sort(holes.begin(), holes.end(),
                   [](const auto& a, const auto& b) { return a.size() > b.size(); });

  // Whether the straight line from a to b crosses a covered cell (not counting
  // its ends).
  const auto blocked = [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    const double length = (b - a).norm();
    const int samples = static_cast<int>(std::ceil(length / (0.5 * cell)));
    for (int s = 1; s < samples; ++s) {
      const Eigen::Vector2d p = a + (b - a) * (static_cast<double>(s) / samples);
      const Eigen::Vector2d f = (p - map.origin) / cell;
      const int x = static_cast<int>(std::floor(f.x()));
      const int y = static_cast<int>(std::floor(f.y()));
      if (x >= 0 && y >= 0 && x < map.width && y < map.height &&
          state(x, y) == GroundState::Covered) {
        return true;
      }
    }
    return false;
  };

  const int reach = static_cast<int>(std::ceil(config.max_view / cell));
  const int stride = std::max(1, static_cast<int>(std::round(0.3 / cell)));  // Spots ~0.3 m apart.
  const double ideal = 0.5 * (config.min_view + config.max_view);
  for (std::size_t h = 0; h < holes.size(); ++h) {
    const std::vector<Cell>& hole = holes[h];
    GuidanceTarget target;
    target.rank = static_cast<int>(h + 1);
    target.cells = hole.size();
    target.area = static_cast<double>(hole.size()) * cell * cell;
    std::vector<Eigen::Vector2d> points;
    points.reserve(hole.size());
    int x0 = map.width;
    int y0 = map.height;
    int x1 = -1;
    int y1 = -1;
    for (const Cell& c : hole) {
      points.push_back(center(c.x, c.y));
      target.center += points.back();
      x0 = std::min(x0, c.x);
      y0 = std::min(y0, c.y);
      x1 = std::max(x1, c.x);
      y1 = std::max(y1, c.y);
    }
    target.center /= static_cast<double>(hole.size());

    // Every spot to stand on near the hole, and the hole cells it sees.
    struct Candidate {
      Eigen::Vector2d position;
      double distance = 0.0;
      std::vector<std::size_t> sees;
    };
    std::vector<Candidate> candidates;
    for (int y = std::max(0, y0 - reach); y <= std::min(map.height - 1, y1 + reach); y += stride) {
      for (int x = std::max(0, x0 - reach); x <= std::min(map.width - 1, x1 + reach); x += stride) {
        if (state(x, y) != GroundState::Measured) {
          continue;
        }
        Candidate c;
        c.position = center(x, y);
        c.distance = std::numeric_limits<double>::infinity();
        for (const Eigen::Vector2d& p : points) {
          c.distance = std::min(c.distance, (p - c.position).norm());
        }
        if (c.distance < config.min_view || c.distance > config.max_view) {
          continue;
        }
        for (std::size_t i = 0; i < points.size(); ++i) {
          if ((points[i] - c.position).norm() <= config.max_view &&
              !blocked(c.position, points[i])) {
            c.sees.push_back(i);
          }
        }
        if (!c.sees.empty()) {
          candidates.push_back(std::move(c));
        }
      }
    }

    // Spots, one at a time: each the one seeing most of what's not yet seen.
    std::vector<char> seen(points.size(), 0);
    std::size_t seen_count = 0;
    while (static_cast<int>(target.spots.size()) < config.max_spots &&
           static_cast<double>(seen_count) <
               config.coverage_goal * static_cast<double>(points.size())) {
      const Candidate* best = nullptr;
      std::size_t best_gain = 0;
      for (const Candidate& c : candidates) {
        std::size_t gain = 0;
        for (const std::size_t i : c.sees) {
          gain += seen[i] ? 0 : 1;
        }
        if (gain > best_gain || (gain == best_gain && best != nullptr &&
                                 std::abs(c.distance - ideal) < std::abs(best->distance - ideal))) {
          best = &c;
          best_gain = gain;
        }
      }
      if (best == nullptr || best_gain == 0) {
        break;
      }
      LookSpot spot;
      spot.position = best->position;
      spot.distance = best->distance;
      spot.sees = static_cast<double>(best->sees.size()) / static_cast<double>(points.size());
      Eigen::Vector2d toward = Eigen::Vector2d::Zero();
      for (const std::size_t i : best->sees) {
        if (!seen[i]) {
          toward += points[i];
          seen[i] = 1;
          ++seen_count;
        }
      }
      toward /= static_cast<double>(best_gain);
      spot.heading = std::atan2(toward.y() - spot.position.y(), toward.x() - spot.position.x());
      target.spots.push_back(spot);
    }
    target.seen = static_cast<double>(seen_count) / static_cast<double>(points.size());
    if (target.area >= config.min_area) {
      guidance.done = false;
    }
    guidance.targets.push_back(std::move(target));
  }

  // Visiting order: nearest next, from start.
  std::vector<char> visited_target(guidance.targets.size(), 0);
  Eigen::Vector2d at = start;
  for (int order = 1; order <= static_cast<int>(guidance.targets.size()); ++order) {
    std::size_t next = 0;
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t t = 0; t < guidance.targets.size(); ++t) {
      const GuidanceTarget& target = guidance.targets[t];
      const Eigen::Vector2d where =
          target.spots.empty() ? target.center : target.spots.front().position;
      if (!visited_target[t] && (where - at).norm() < nearest) {
        nearest = (where - at).norm();
        next = t;
      }
    }
    visited_target[next] = 1;
    guidance.targets[next].order = order;
    const GuidanceTarget& chosen = guidance.targets[next];
    at = chosen.spots.empty() ? chosen.center : chosen.spots.back().position;
  }
  return guidance;
}

}  // namespace vg
