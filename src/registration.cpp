#include "vg_core/registration.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace vg {
namespace {

constexpr double kPi = 3.14159265358979323846;
// A fit is a rival to the best (another place the walk could belong) if it's
// turned this much more or less, or puts the map's points this far from where
// the best does, on average (m). Nearer, it's the same alignment, stopped
// short by the fine fit: a few degrees off moves a yard's far side a meter.
constexpr double kRivalTurn = 20.0 * kPi / 180.0;
constexpr double kRivalDistance = 2.5;
// The fine fit's overlap test: a point with reference within this is in the
// overlap (m).
constexpr double kOverlapDistance = 1.0;
// The coarse search keeps this many shifts per heading, at least
// kPeakSeparation apart (m).
constexpr int kPeaksPerHeading = 3;
constexpr double kPeakSeparation = 2.0;
// Cell of the fine fit's small lookup grid, used once pairs are this close (m).
constexpr double kNarrowCell = 0.3;

using Complex = std::complex<double>;

Eigen::Matrix2d rotation(double yaw) {
  return Eigen::Rotation2Dd(yaw).toRotationMatrix();
}

double wrap_angle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  return (a < 0.0 ? a + 2.0 * kPi : a) - kPi;
}

// In-place radix-2 FFT of n (a power of two) values spaced stride apart.
void fft(Complex* data, int n, int stride, bool inverse) {
  for (int i = 1, j = 0; i < n; ++i) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) {
      j ^= bit;
    }
    j ^= bit;
    if (i < j) {
      std::swap(data[i * stride], data[j * stride]);
    }
  }
  for (int len = 2; len <= n; len <<= 1) {
    const double angle = (inverse ? 2.0 : -2.0) * kPi / len;
    const Complex step(std::cos(angle), std::sin(angle));
    for (int i = 0; i < n; i += len) {
      Complex w(1.0, 0.0);
      for (int k = 0; k < len / 2; ++k) {
        const Complex a = data[(i + k) * stride];
        const Complex b = data[(i + k + len / 2) * stride] * w;
        data[(i + k) * stride] = a + b;
        data[(i + k + len / 2) * stride] = a - b;
        w *= step;
      }
    }
  }
}

// 2D FFT of a rows x cols (powers of two) row-major image.
void fft2(std::vector<Complex>& image, int rows, int cols, bool inverse) {
  for (int r = 0; r < rows; ++r) {
    fft(image.data() + static_cast<std::ptrdiff_t>(r) * cols, cols, 1, inverse);
  }
  for (int c = 0; c < cols; ++c) {
    fft(image.data() + c, rows, cols, inverse);
  }
}

int next_power_of_two(int n) {
  int p = 1;
  while (p < n) {
    p <<= 1;
  }
  return p;
}

// Feature cells as (x, y, 1) points: where something stands, however tall, so
// a shed only one walk saw counts by its outline like a bed does.
std::vector<Eigen::Vector3d> feature_points(const HeightMap& map, const RegistrationConfig& config) {
  std::vector<Eigen::Vector3d> points;
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      const float h = map.at(x, y);
      if (!std::isnan(h) && h >= config.min_feature_height) {
        const Eigen::Vector2d c = map.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * map.cell_size;
        points.emplace_back(c.x(), c.y(), 1.0);
      }
    }
  }
  return points;
}

// The coarse search's raster and its FFT.
struct Raster {
  Eigen::Vector2d origin;
  double cell = 0.0;
  int rows = 0;
  int cols = 0;

  Eigen::Vector2d center() const { return origin + Eigen::Vector2d(cols, rows) * cell / 2.0; }

  // The outline of the points drawn: cells where the next one over differs,
  // zero mean. Outlines rather than areas, so a big thing only one walk saw
  // (a shed) counts by its outline, not its area. Returns the sum of squares.
  double draw(const std::vector<Eigen::Vector3d>& points, std::vector<Complex>& image) const {
    std::vector<double> values(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols),
                               0.0);
    for (const Eigen::Vector3d& p : points) {
      const int x = static_cast<int>(std::floor((p.x() - origin.x()) / cell));
      const int y = static_cast<int>(std::floor((p.y() - origin.y()) / cell));
      if (x >= 0 && y >= 0 && x < cols && y < rows) {
        double& v = values[static_cast<std::size_t>(y) * static_cast<std::size_t>(cols) +
                           static_cast<std::size_t>(x)];
        v = std::max(v, p.z());
      }
    }
    std::vector<double> edges(values.size(), 0.0);
    for (int y = 0; y + 1 < rows; ++y) {
      for (int x = 0; x + 1 < cols; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(cols) +
                              static_cast<std::size_t>(x);
        edges[i] = std::max(std::abs(values[i + 1] - values[i]),
                            std::abs(values[i + static_cast<std::size_t>(cols)] - values[i]));
      }
    }
    // Blurred (3 x 3 box), so an edge a cell off still counts.
    for (int y = 0; y < rows; ++y) {
      for (int x = 0; x < cols; ++x) {
        double sum = 0.0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int xx = std::clamp(x + dx, 0, cols - 1);
            const int yy = std::clamp(y + dy, 0, rows - 1);
            sum += edges[static_cast<std::size_t>(yy) * static_cast<std::size_t>(cols) +
                         static_cast<std::size_t>(xx)];
          }
        }
        values[static_cast<std::size_t>(y) * static_cast<std::size_t>(cols) +
               static_cast<std::size_t>(x)] = sum / 9.0;
      }
    }
    double mean = 0.0;
    for (const double v : values) {
      mean += v;
    }
    mean /= static_cast<double>(values.size());
    double energy = 0.0;
    image.resize(values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {
      image[i] = values[i] - mean;
      energy += (values[i] - mean) * (values[i] - mean);
    }
    return energy;
  }
};

struct Candidate {
  double score = -1.0;
  double rank = 0.0;  // score, weighed by agreement with GPS.
  double yaw = 0.0;
  Eigen::Vector2d translation = Eigen::Vector2d::Zero();
};

// Nearest-neighbor lookups among points, hashed into cells of a given size;
// finds neighbors within that size.
class PointHash {
 public:
  PointHash(const std::vector<Eigen::Vector3d>& points, double cell)
      : points_(points), cell_(cell) {
    for (std::size_t i = 0; i < points.size(); ++i) {
      cells_[key(cell_of(points[i]))].push_back(i);
    }
  }

  // Index of the nearest point within max_distance (<= cell), or -1.
  std::ptrdiff_t nearest(const Eigen::Vector3d& p, double max_distance, double* distance) const {
    const Eigen::Vector3i c = cell_of(p);
    double best = max_distance * max_distance;
    std::ptrdiff_t found = -1;
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const auto it = cells_.find(key(c + Eigen::Vector3i(dx, dy, dz)));
          if (it == cells_.end()) {
            continue;
          }
          for (const std::size_t i : it->second) {
            const double d = (points_[i] - p).squaredNorm();
            if (d <= best) {
              best = d;
              found = static_cast<std::ptrdiff_t>(i);
            }
          }
        }
      }
    }
    if (distance != nullptr) {
      *distance = std::sqrt(best);
    }
    return found;
  }

 private:
  Eigen::Vector3i cell_of(const Eigen::Vector3d& p) const {
    return (p / cell_).array().floor().cast<int>();
  }
  static std::int64_t key(const Eigen::Vector3i& c) {
    return (static_cast<std::int64_t>(c.x() & 0x1FFFFF) << 42) |
           (static_cast<std::int64_t>(c.y() & 0x1FFFFF) << 21) |
           static_cast<std::int64_t>(c.z() & 0x1FFFFF);
  }

  const std::vector<Eigen::Vector3d>& points_;
  double cell_;
  std::unordered_map<std::int64_t, std::vector<std::size_t>> cells_;
};

double median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  return *mid;
}

// The points at least min_height over the terrain (where it's known).
std::vector<Eigen::Vector3d> standing(const std::vector<Eigen::Vector3d>& points,
                                      const HeightMap& terrain, double min_height) {
  std::vector<Eigen::Vector3d> out;
  for (const Eigen::Vector3d& p : points) {
    const Eigen::Vector2d f = (p.head<2>() - terrain.origin) / terrain.cell_size;
    const int x = static_cast<int>(std::floor(f.x()));
    const int y = static_cast<int>(std::floor(f.y()));
    if (x >= 0 && y >= 0 && x < terrain.width && y < terrain.height && terrain.has_data(x, y) &&
        p.z() - terrain.at(x, y) >= min_height) {
      out.push_back(p);
    }
  }
  return out;
}

// One point per cell of the given size (the first in it): fewer points to
// search, as dense voxels add little to a fit.
std::vector<Eigen::Vector3d> thin(const std::vector<Eigen::Vector3d>& points, double cell) {
  std::unordered_map<std::int64_t, bool> taken;
  std::vector<Eigen::Vector3d> out;
  for (const Eigen::Vector3d& p : points) {
    const Eigen::Vector3i c = (p / cell).array().floor().cast<int>();
    const std::int64_t key = (static_cast<std::int64_t>(c.x() & 0x1FFFFF) << 42) |
                             (static_cast<std::int64_t>(c.y() & 0x1FFFFF) << 21) |
                             static_cast<std::int64_t>(c.z() & 0x1FFFFF);
    if (taken.emplace(key, true).second) {
      out.push_back(p);
    }
  }
  return out;
}

// Height of reference's terrain minus map's, where they overlap under t.
std::optional<double> terrain_offset(const HeightMap& reference, const HeightMap& map,
                                     const MapTransform& t) {
  std::vector<double> differences;
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      const float h = map.at(x, y);
      if (std::isnan(h)) {
        continue;
      }
      const Eigen::Vector2d p =
          t.apply_xy(map.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * map.cell_size);
      const Eigen::Vector2d f = (p - reference.origin) / reference.cell_size;
      const int rx = static_cast<int>(std::floor(f.x()));
      const int ry = static_cast<int>(std::floor(f.y()));
      if (rx >= 0 && ry >= 0 && rx < reference.width && ry < reference.height &&
          reference.has_data(rx, ry)) {
        differences.push_back(reference.at(rx, ry) - h);
      }
    }
  }
  if (differences.empty()) {
    return std::nullopt;
  }
  return median(differences);
}

}  // namespace

Eigen::Vector3d MapTransform::apply(const Eigen::Vector3d& p) const {
  Eigen::Vector3d q;
  q.head<2>() = rotation(yaw) * p.head<2>();
  q.z() = p.z();
  return q + translation;
}

Eigen::Vector2d MapTransform::apply_xy(const Eigen::Vector2d& p) const {
  return rotation(yaw) * p + translation.head<2>();
}

MapTransform MapTransform::after(const MapTransform& other) const {
  MapTransform t;
  t.yaw = wrap_angle(yaw + other.yaw);
  t.translation = apply(other.translation);
  return t;
}

MapTransform MapTransform::inverse() const {
  MapTransform t;
  t.yaw = -yaw;
  t.translation.head<2>() = -(rotation(-yaw) * translation.head<2>());
  t.translation.z() = -translation.z();
  return t;
}

RegistrationResult register_map(const RegistrationInput& reference, const RegistrationInput& map,
                                const RegistrationConfig& config,
                                const RegistrationPrior& prior) {
  RegistrationResult result;
  const auto fail = [&](const std::string& problem) {
    result.problem = problem;
    return result;
  };
  const auto ref_features = feature_points(reference.features, config);
  const auto map_features = feature_points(map.features, config);
  if (ref_features.size() < 10 || map_features.size() < 10) {
    return fail("too few features to match (run the terrain stage; walks need beds, plants, "
                "walls or posts in view)");
  }
  // Every standing voxel of the reference judges the fit; a thinned set
  // drives it (thinning stretches the distances a fit is judged by).
  const auto ref_standing = standing(reference.points, reference.terrain, config.icp_min_height);
  const auto ref_points = thin(ref_standing, config.icp_thin);
  const auto map_points =
      thin(standing(map.points, map.terrain, config.icp_min_height), config.icp_thin);
  if (ref_points.size() < 100 || map_points.size() < 100) {
    return fail("too few voxels standing over the terrain to fit (beds, plants, walls)");
  }

  // Coarse: the raster covers the reference's features plus the search radius.
  Eigen::Vector2d lo = ref_features.front().head<2>();
  Eigen::Vector2d hi = lo;
  for (const auto& p : ref_features) {
    lo = lo.cwiseMin(p.head<2>());
    hi = hi.cwiseMax(p.head<2>());
  }
  Raster raster;
  raster.cell = config.raster;
  raster.origin = lo - Eigen::Vector2d::Constant(config.search_radius);
  const Eigen::Vector2d span = hi - lo + Eigen::Vector2d::Constant(2.0 * config.search_radius);
  raster.cols = next_power_of_two(static_cast<int>(std::ceil(span.x() / raster.cell)));
  raster.rows = next_power_of_two(static_cast<int>(std::ceil(span.y() / raster.cell)));
  std::vector<Complex> ref_image;
  const double ref_energy = raster.draw(ref_features, ref_image);
  fft2(ref_image, raster.rows, raster.cols, false);

  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& p : map_features) {
    centroid += p.head<2>();
  }
  centroid /= static_cast<double>(map_features.size());

  // Headings to try.
  std::vector<double> yaws;
  const double step = config.yaw_step_deg * kPi / 180.0;
  if (prior.yaw && std::isfinite(prior.yaw_sigma)) {
    const double window = std::max(config.prior_sigmas * prior.yaw_sigma,
                                   config.min_yaw_window_deg * kPi / 180.0);
    if (window < kPi) {
      result.used_prior = true;
      for (double d = -window; d <= window + 1e-9; d += step) {
        yaws.push_back(wrap_angle(*prior.yaw + d));
      }
    }
  }
  if (yaws.empty()) {
    for (double a = 0.0; a < 2.0 * kPi - 1e-9; a += step) {
      yaws.push_back(a);
    }
  }

  // How likely an alignment is given GPS (1 without it): its heading, and
  // where it puts the middle of the walk, whose GPS spread grows with its
  // distance from the walk's origin by the heading's uncertainty.
  Eigen::Vector2d middle = Eigen::Vector2d::Zero();
  for (const Eigen::Vector3d& p : map_points) {
    middle += p.head<2>();
  }
  middle /= static_cast<double>(map_points.size());
  const auto gps_likelihood = [&](const MapTransform& t) {
    double log = 0.0;
    if (result.used_prior) {
      const double z = wrap_angle(t.yaw - *prior.yaw) / prior.yaw_sigma;
      log -= 0.5 * z * z;
    }
    if (prior.origin && prior.origin_sigma > 0.0) {
      const double yaw = prior.yaw ? *prior.yaw : t.yaw;
      const double yaw_sigma = prior.yaw ? prior.yaw_sigma : 0.0;
      const Eigen::Vector2d expected = rotation(yaw) * middle + *prior.origin;
      const double sigma2 = prior.origin_sigma * prior.origin_sigma +
                            middle.squaredNorm() * yaw_sigma * yaw_sigma;
      log -= 0.5 * (t.apply_xy(middle) - expected).squaredNorm() / sigma2;
    }
    return std::max(1e-9, std::exp(log));
  };

  // Each heading's best few shifts (a walk seeing one of two alike beds fits
  // both), as candidate alignments.
  std::vector<Candidate> candidates;
  std::vector<Complex> image;
  std::vector<Eigen::Vector3d> moved(map_features.size());
  const int suppress = static_cast<int>(std::ceil(kPeakSeparation / raster.cell));
  for (const double yaw : yaws) {
    const Eigen::Matrix2d r = rotation(yaw);
    for (std::size_t i = 0; i < map_features.size(); ++i) {
      moved[i].head<2>() = r * (map_features[i].head<2>() - centroid) + raster.center();
      moved[i].z() = map_features[i].z();
    }
    const double energy = raster.draw(moved, image);
    fft2(image, raster.rows, raster.cols, false);
    for (std::size_t i = 0; i < image.size(); ++i) {
      image[i] = ref_image[i] * std::conj(image[i]);
    }
    fft2(image, raster.rows, raster.cols, true);
    const double norm = static_cast<double>(image.size()) * std::sqrt(ref_energy * energy);
    for (int peak = 0; peak < kPeaksPerHeading; ++peak) {
      std::size_t best = 0;
      for (std::size_t i = 1; i < image.size(); ++i) {
        if (image[i].real() > image[best].real()) {
          best = i;
        }
      }
      const int by = static_cast<int>(best / static_cast<std::size_t>(raster.cols));
      const int bx = static_cast<int>(best % static_cast<std::size_t>(raster.cols));
      const int ky = by < raster.rows / 2 ? by : by - raster.rows;
      const int kx = bx < raster.cols / 2 ? bx : bx - raster.cols;
      Candidate c;
      c.score = norm > 0.0 ? image[best].real() / norm : 0.0;
      c.yaw = yaw;
      // map p -> r (p - centroid) + center + shift.
      c.translation = raster.center() + Eigen::Vector2d(kx, ky) * raster.cell - r * centroid;
      candidates.push_back(c);
      // Clear around it (wrapping, as the correlation does) for the next.
      for (int dy = -suppress; dy <= suppress; ++dy) {
        for (int dx = -suppress; dx <= suppress; ++dx) {
          const int y = ((by + dy) % raster.rows + raster.rows) % raster.rows;
          const int x = ((bx + dx) % raster.cols + raster.cols) % raster.cols;
          image[static_cast<std::size_t>(y) * static_cast<std::size_t>(raster.cols) +
                static_cast<std::size_t>(x)] = -std::numeric_limits<double>::infinity();
        }
      }
    }
  }
  // The best few distinct candidates go on to the fine fit, which picks among
  // them: the coarse score alone can take a match for its mirror image (a
  // rectangular yard turned 180 degrees) or for a look-alike place. They're
  // ranked with GPS where there is some, so the place it points at is tried.
  const auto as_transform = [](const Candidate& c) {
    MapTransform t;
    t.yaw = c.yaw;
    t.translation.head<2>() = c.translation;
    return t;
  };
  for (Candidate& c : candidates) {
    c.rank = c.score * gps_likelihood(as_transform(c));
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.rank > b.rank; });
  std::vector<Candidate> picks;
  for (const Candidate& c : candidates) {
    if (static_cast<int>(picks.size()) >= std::max(1, config.coarse_candidates)) {
      break;
    }
    if (std::all_of(picks.begin(), picks.end(), [&](const Candidate& p) {
          return std::abs(wrap_angle(c.yaw - p.yaw)) >=
                     config.candidate_separation_deg * kPi / 180.0 - 1e-9 ||
                 (as_transform(c).apply_xy(middle) - as_transform(p).apply_xy(middle)).norm() >=
                     kPeakSeparation;
        })) {
      picks.push_back(c);
    }
  }

  // Fine: trimmed ICP over (an even pick of) the map's points.
  std::vector<Eigen::Vector3d> sample;
  const auto cap = static_cast<std::size_t>(std::max(1, config.max_icp_points));
  const std::size_t every = std::max<std::size_t>(1, (map_points.size() + cap - 1) / cap);
  for (std::size_t i = 0; i < map_points.size(); i += every) {
    sample.push_back(map_points[i]);
  }
  // Lookups in cells of the pairing distance: large while it's large, small
  // once it shrinks (far fewer points to check per cell).
  const PointHash wide(ref_points, std::max(config.icp_start_distance, kOverlapDistance));
  const PointHash narrow(ref_points, std::max(config.icp_end_distance, kNarrowCell));
  const PointHash far(ref_standing, kOverlapDistance);

  struct Fit {
    MapTransform t;
    double score = 0.0;
    std::size_t inliers = 0;  // Within 0.3 m.
    double overlap = 0.0;
    double inliers_10cm = 0.0;
    double inliers_30cm = 0.0;
    double median_distance = 0.0;
    // Points lining up, times how well its heading agrees with GPS.
    double weight = 0.0;
  };
  std::vector<Fit> fits;
  for (const Candidate& c : picks) {
    MapTransform t;
    t.yaw = c.yaw;
    t.translation.head<2>() = c.translation;
    t.translation.z() = terrain_offset(reference.terrain, map.terrain, t).value_or(0.0);
    for (int round = 0; round < config.icp_rounds; ++round) {
      const double limit = std::max(config.icp_end_distance,
                                    config.icp_start_distance * std::pow(0.85, round));
      Eigen::Vector2d sum_src = Eigen::Vector2d::Zero();
      Eigen::Vector2d sum_dst = Eigen::Vector2d::Zero();
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> pairs;
      for (const Eigen::Vector3d& p : sample) {
        const Eigen::Vector3d q = t.apply(p);
        const auto j = (limit <= kNarrowCell ? narrow : wide).nearest(q, limit, nullptr);
        if (j >= 0) {
          pairs.emplace_back(q, ref_points[static_cast<std::size_t>(j)]);
          sum_src += q.head<2>();
          sum_dst += ref_points[static_cast<std::size_t>(j)].head<2>();
        }
      }
      if (pairs.size() < 100) {
        break;
      }
      const Eigen::Vector2d cs = sum_src / static_cast<double>(pairs.size());
      const Eigen::Vector2d cd = sum_dst / static_cast<double>(pairs.size());
      double sxx = 0.0;  // Sums for the best turn about z (2D Kabsch).
      double sxy = 0.0;
      std::vector<double> dz;
      dz.reserve(pairs.size());
      for (const auto& [src, dst] : pairs) {
        const Eigen::Vector2d a = src.head<2>() - cs;
        const Eigen::Vector2d b = dst.head<2>() - cd;
        sxx += a.dot(b);
        sxy += a.x() * b.y() - a.y() * b.x();
        dz.push_back(dst.z() - src.z());
      }
      MapTransform step_t;
      step_t.yaw = std::atan2(sxy, sxx);
      step_t.translation.head<2>() = cd - rotation(step_t.yaw) * cs;
      step_t.translation.z() = median(std::move(dz));
      t = step_t.after(t);
    }

    // How well it fits.
    Fit fit;
    fit.t = t;
    fit.score = c.score;
    std::vector<double> distances;
    std::size_t within_10 = 0;
    for (const Eigen::Vector3d& p : sample) {
      double d = 0.0;
      if (far.nearest(t.apply(p), kOverlapDistance, &d) >= 0) {
        distances.push_back(d);
        within_10 += d <= 0.1 ? 1 : 0;
        fit.inliers += d <= 0.3 ? 1 : 0;
      }
    }
    fit.overlap = static_cast<double>(distances.size()) / static_cast<double>(sample.size());
    if (!distances.empty()) {
      const auto n = static_cast<double>(distances.size());
      fit.inliers_10cm = static_cast<double>(within_10) / n;
      fit.inliers_30cm = static_cast<double>(fit.inliers) / n;
      fit.median_distance = median(distances);
    }
    fit.weight = static_cast<double>(fit.inliers) * gps_likelihood(t);
    fits.push_back(fit);
  }

  // Fits ending near the same alignment (one stopped short of the other) are
  // one alignment, represented by the fit lining up the most points: the
  // geometry settles the details. Between distinct alignments, GPS weighs in
  // where there is some (it breaks near ties: a short walk's GPS heading can
  // be off by tens of degrees, too loose to pick by itself).
  const auto apart = [&](const MapTransform& a, const MapTransform& b) {
    // How far apart two alignments put the map's points, on average.
    double sum = 0.0;
    for (const Eigen::Vector3d& p : sample) {
      sum += (a.apply(p) - b.apply(p)).head<2>().norm();
    }
    return sum / static_cast<double>(sample.size());
  };
  const auto distinct = [&](const MapTransform& a, const MapTransform& b) {
    return std::abs(wrap_angle(a.yaw - b.yaw)) > kRivalTurn || apart(a, b) > kRivalDistance;
  };
  std::sort(fits.begin(), fits.end(),
            [](const Fit& a, const Fit& b) { return a.inliers > b.inliers; });
  std::vector<const Fit*> alignments;
  for (const Fit& fit : fits) {
    if (std::all_of(alignments.begin(), alignments.end(),
                    [&](const Fit* a) { return distinct(fit.t, a->t); })) {
      alignments.push_back(&fit);
    }
  }
  std::sort(alignments.begin(), alignments.end(),
            [](const Fit* a, const Fit* b) { return a->weight > b->weight; });
  const Fit& best = *alignments.front();
  result.reference_from_map = best.t;
  result.peak = best.score;
  result.overlap = best.overlap;
  result.inliers_10cm = best.inliers_10cm;
  result.inliers_30cm = best.inliers_30cm;
  result.median_distance = best.median_distance;
  if (alignments.size() > 1) {
    result.rival_yaw = alignments[1]->t.yaw;
    result.rival_ratio = best.weight / std::max(1e-9, alignments[1]->weight);
  }

  std::ostringstream problem;
  if (result.rival_ratio > 0.0 && result.rival_ratio < config.min_rival_ratio) {
    problem << "ambiguous: turned " << best.t.yaw * 180.0 / kPi << " deg it fits only "
            << result.rival_ratio << " times as well as turned " << result.rival_yaw * 180.0 / kPi
            << " deg" << (result.used_prior || prior.origin ? ", even with GPS" : " (no GPS used)")
            << " (a walk seeing more of the site, or more of one place's surroundings, helps)";
  } else if (result.overlap < config.min_overlap) {
    problem << "too little overlap: " << 100.0 * result.overlap << "% of the walk is within "
            << kOverlapDistance << " m of the reference";
  } else if (result.inliers_30cm < config.min_inliers) {
    problem << "doesn't line up: only " << 100.0 * result.inliers_30cm
            << "% of the overlap within 0.3 m";
  }
  result.problem = problem.str();
  result.accepted = result.problem.empty();
  return result;
}

}  // namespace vg
