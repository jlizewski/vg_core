#include "vg_core/live_guidance.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "vg_core/height_map_io.hpp"
#include "vg_core/site.hpp"

namespace vg {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap(double a) {
  while (a > kPi) {
    a -= 2.0 * kPi;
  }
  while (a <= -kPi) {
    a += 2.0 * kPi;
  }
  return a;
}

}  // namespace

LiveGuidance::LiveGuidance(const GroundLayer& ground, const MapTransform& site_from_world,
                           const LiveGuidanceConfig& config)
    : config_(config), site_from_world_(site_from_world), grid_(ground.heights) {
  grid_.heights.clear();
  const HeightMap& map = ground.heights;
  hole_of_.assign(ground.state.size(), -1);
  seen_.assign(ground.state.size(), 0);
  const Guidance plan = plan_guidance(ground, Eigen::Vector2d::Zero(), config.guidance);
  const std::vector<std::vector<HoleCell>> holes = find_holes(ground);
  const auto center = [&](int x, int y) {
    return Eigen::Vector2d(map.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * map.cell_size);
  };
  for (std::size_t h = 0; h < holes.size() && h < plan.targets.size(); ++h) {
    const GuidanceTarget& t = plan.targets[h];
    LiveTarget live;
    live.rank = t.rank;
    live.order = t.order;
    live.area = t.area;
    live.center = t.center;
    live.spots = t.spots;
    live.cells = holes[h].size();
    for (const HoleCell& c : holes[h]) {
      hole_of_[ground.index(c.x, c.y)] = static_cast<int>(h);
    }
    // The expected ground: a plane through the measured ground around the
    // hole (within 2 cells of it), in coordinates relative to its center.
    std::vector<Eigen::Vector3d> ring;
    std::vector<char> taken(ground.state.size(), 0);
    for (const HoleCell& c : holes[h]) {
      for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
          const int x = c.x + dx;
          const int y = c.y + dy;
          if (x < 0 || y < 0 || x >= map.width || y >= map.height) {
            continue;
          }
          const std::size_t i = ground.index(x, y);
          if (taken[i] || ground.state[i] != GroundState::Measured || !map.has_data(x, y)) {
            continue;
          }
          taken[i] = 1;
          const Eigen::Vector2d p = center(x, y) - t.center;
          ring.emplace_back(p.x(), p.y(), map.at(x, y));
        }
      }
    }
    Eigen::Vector3d plane(0.0, 0.0, std::numeric_limits<double>::quiet_NaN());
    if (ring.size() >= 6) {
      Eigen::MatrixXd a(ring.size(), 3);
      Eigen::VectorXd b(ring.size());
      for (std::size_t r = 0; r < ring.size(); ++r) {
        a.row(static_cast<Eigen::Index>(r)) << ring[r].x(), ring[r].y(), 1.0;
        b(static_cast<Eigen::Index>(r)) = ring[r].z();
      }
      plane = a.colPivHouseholderQr().solve(b);
    } else if (!ring.empty()) {
      double sum = 0.0;
      for (const Eigen::Vector3d& r : ring) {
        sum += r.z();
      }
      plane.z() = sum / static_cast<double>(ring.size());
    }
    plane_.push_back(plane);
    targets_.push_back(std::move(live));
  }
}

LiveGuidance LiveGuidance::for_site(const std::filesystem::path& site_dir,
                                    const std::string& tracking_frame,
                                    const LiveGuidanceConfig& config) {
  const auto dir = site_dir / ground_layer_file::kDir;
  if (!std::filesystem::is_regular_file(dir / ground_layer_file::kState)) {
    throw std::runtime_error("live guidance: " + site_dir.string() +
                             " has no ground layer yet (vg site ground)");
  }
  const std::optional<MapTransform> placement =
      Site(site_dir).placement_of_tracking_frame(tracking_frame);
  if (!placement) {
    throw std::runtime_error("live guidance: no walk of " + site_dir.string() +
                             " was recorded in tracking frame '" + tracking_frame +
                             "', so this walk can't be placed in the site while it runs");
  }
  GroundLayer ground;
  ground.heights = load_height_map(dir / ground_layer_file::kHeights);
  const HeightMap states = load_height_map(dir / ground_layer_file::kState);
  if (states.width != ground.heights.width || states.height != ground.heights.height) {
    throw std::runtime_error("live guidance: the ground layer's grids don't match");
  }
  ground.state.reserve(states.heights.size());
  for (const float v : states.heights) {
    ground.state.push_back(std::isnan(v) ? GroundState::Outside
                                         : static_cast<GroundState>(static_cast<int>(v)));
  }
  return LiveGuidance(ground, *placement, config);
}

void LiveGuidance::see(const Eigen::Vector2d& site_xy, double site_z) {
  const Eigen::Vector2d f = (site_xy - grid_.origin) / grid_.cell_size;
  const int x = static_cast<int>(std::floor(f.x()));
  const int y = static_cast<int>(std::floor(f.y()));
  if (x < 0 || y < 0 || x >= grid_.width || y >= grid_.height) {
    return;
  }
  const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(grid_.width) +
                        static_cast<std::size_t>(x);
  const int h = hole_of_[i];
  if (h < 0 || seen_[i]) {
    return;
  }
  LiveTarget& t = targets_[static_cast<std::size_t>(h)];
  const Eigen::Vector3d& plane = plane_[static_cast<std::size_t>(h)];
  if (!std::isnan(plane.z())) {
    const Eigen::Vector2d p = site_xy - t.center;
    const double over = site_z - (plane.x() * p.x() + plane.y() * p.y() + plane.z());
    if (over > config_.max_height || over < -config_.max_depth) {
      return;
    }
  }
  seen_[i] = 1;
  ++t.seen_cells;
  t.seen = static_cast<double>(t.seen_cells) / static_cast<double>(t.cells);
  t.done = t.seen >= config_.done_share;
}

void LiveGuidance::update(const std::vector<HeightCell>& changed, double cell_size) {
  // The live grid is turned against the site's, so each live cell is sampled
  // at its center and near its corners to reach every site cell it covers.
  const double q = 0.45 * cell_size;
  for (const HeightCell& c : changed) {
    if (std::isnan(c.height)) {
      continue;  // A surface going away doesn't unsee what was seen.
    }
    const Eigen::Vector2d middle((c.x + 0.5) * cell_size, (c.y + 0.5) * cell_size);
    const double z = site_from_world_.apply(Eigen::Vector3d(middle.x(), middle.y(), c.height)).z();
    for (const Eigen::Vector2d& d :
         {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(q, q), Eigen::Vector2d(-q, q),
          Eigen::Vector2d(q, -q), Eigen::Vector2d(-q, -q)}) {
      see(site_from_world_.apply_xy(middle + d), z);
    }
  }
}

void LiveGuidance::set_camera_pose(const Eigen::Isometry3d& world_from_camera) {
  position_ = site_from_world_.apply_xy(world_from_camera.translation().head<2>());
  // Facing: the camera's view direction across the ground; looking almost
  // straight down, the image's up direction instead.
  Eigen::Vector3d forward = world_from_camera.linear().col(2);
  if (forward.head<2>().norm() < 0.2) {
    forward = -world_from_camera.linear().col(1);
  }
  facing_ = std::atan2(forward.y(), forward.x()) + site_from_world_.yaw;
}

std::optional<NextSpot> LiveGuidance::next() const {
  if (!position_) {
    return std::nullopt;
  }
  // The nearest spot of any hole not yet done (its center if it has no
  // spot): the plan's visiting order assumed a start at the site's origin,
  // and the walker goes where they go.
  std::optional<NextSpot> best;
  double nearest = std::numeric_limits<double>::infinity();
  for (const LiveTarget& t : targets_) {
    if (t.done) {
      continue;
    }
    const auto consider = [&](const Eigen::Vector2d& at, double heading) {
      const double d = (at - *position_).norm();
      if (d < nearest) {
        nearest = d;
        best = NextSpot{t.rank, at, heading, 0.0, 0.0};
      }
    };
    if (t.spots.empty()) {
      consider(t.center, std::atan2(t.center.y() - position_->y(), t.center.x() - position_->x()));
    }
    for (const LookSpot& s : t.spots) {
      consider(s.position, s.heading);
    }
  }
  if (!best) {
    return std::nullopt;
  }
  NextSpot next = *best;
  const Eigen::Vector2d to = next.position - *position_;
  next.distance = to.norm();
  next.turn = wrap(std::atan2(to.y(), to.x()) - facing_);
  return next;
}

bool LiveGuidance::done() const {
  return std::all_of(targets_.begin(), targets_.end(), [](const auto& t) { return t.done; });
}

double LiveGuidance::unseen_area() const {
  const double cell = grid_.cell_size * grid_.cell_size;
  double area = 0.0;
  for (const LiveTarget& t : targets_) {
    area += static_cast<double>(t.cells - t.seen_cells) * cell;
  }
  return area;
}

std::optional<Eigen::Vector2d> LiveGuidance::position() const { return position_; }

std::vector<LiveGuidance::HoleCellState> LiveGuidance::hole_cells() const {
  std::vector<HoleCellState> cells;
  for (int y = 0; y < grid_.height; ++y) {
    for (int x = 0; x < grid_.width; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(grid_.width) +
                            static_cast<std::size_t>(x);
      if (hole_of_[i] >= 0) {
        cells.push_back({grid_.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * grid_.cell_size,
                         hole_of_[i], seen_[i] != 0});
      }
    }
  }
  return cells;
}

}  // namespace vg
