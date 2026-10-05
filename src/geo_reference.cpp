#include "vg_core/geo_reference.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

namespace vg {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84E2 = 6.69437999014e-3;

// Meridional and prime vertical radii of curvature at a latitude.
void earth_radii(double latitude, double& meridional, double& prime_vertical) {
  const double s = std::sin(latitude * kDeg);
  const double w = 1.0 - kWgs84E2 * s * s;
  prime_vertical = kWgs84A / std::sqrt(w);
  meridional = kWgs84A * (1.0 - kWgs84E2) / (w * std::sqrt(w));
}

double horizontal_variance(const GpsFix& fix, double default_sigma) {
  const double v = 0.5 * (fix.position_covariance(0, 0) + fix.position_covariance(1, 1));
  return v > 0.0 ? v : default_sigma * default_sigma;
}

// Body position at time t, interpolated between the poses either side of it.
std::optional<Eigen::Vector3d> position_at(const std::vector<PoseStamped>& poses, Timestamp t,
                                           Timestamp max_gap) {
  const auto after =
      std::lower_bound(poses.begin(), poses.end(), t,
                       [](const PoseStamped& p, Timestamp time) { return p.timestamp < time; });
  if (after != poses.end() && after->timestamp == t) {
    return after->world_from_frame.translation();
  }
  if (after == poses.begin() || after == poses.end()) {
    return std::nullopt;
  }
  const auto before = after - 1;
  if (t - before->timestamp > max_gap || after->timestamp - t > max_gap) {
    return std::nullopt;
  }
  const double a = static_cast<double>(t - before->timestamp) /
                   static_cast<double>(after->timestamp - before->timestamp);
  return (1.0 - a) * before->world_from_frame.translation() +
         a * after->world_from_frame.translation();
}

}  // namespace

Eigen::Vector3d GeoReference::world_from_enu(const Eigen::Vector3d& enu) const {
  return Eigen::AngleAxisd(-heading, Eigen::Vector3d::UnitZ()) * enu;
}

Eigen::Vector3d GeoReference::enu_from_world(const Eigen::Vector3d& world) const {
  return Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()) * world;
}

Eigen::Vector3d enu_offset(double latitude, double longitude, double altitude, double ref_latitude,
                           double ref_longitude, double ref_altitude) {
  double rm = 0.0;
  double rn = 0.0;
  earth_radii(ref_latitude, rm, rn);
  double dlon = longitude - ref_longitude;
  dlon -= 360.0 * std::round(dlon / 360.0);
  return {dlon * kDeg * (rn + ref_altitude) * std::cos(ref_latitude * kDeg),
          (latitude - ref_latitude) * kDeg * (rm + ref_altitude), altitude - ref_altitude};
}

std::optional<GeoReference> estimate_geo_reference(const std::vector<GpsFix>& fixes,
                                                   const std::vector<PoseStamped>& poses,
                                                   const GeoReferenceConfig& config) {
  if (fixes.empty()) {
    return std::nullopt;
  }

  // Reference point: the weighted mean fix.
  double weight_sum = 0.0;
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  const double lon0 = fixes.front().longitude;
  for (const GpsFix& fix : fixes) {
    const double w = 1.0 / horizontal_variance(fix, config.default_gps_sigma);
    double dlon = fix.longitude - lon0;
    dlon -= 360.0 * std::round(dlon / 360.0);
    weight_sum += w;
    lat += w * fix.latitude;
    lon += w * dlon;
    alt += w * fix.altitude;
  }
  GeoReference geo;
  geo.latitude = lat / weight_sum;
  geo.longitude = lon0 + lon / weight_sum;
  geo.altitude = alt / weight_sum;
  geo.heading = 0.0;
  geo.heading_sigma = std::numeric_limits<double>::infinity();

  std::vector<PoseStamped> sorted = poses;
  std::sort(sorted.begin(), sorted.end(),
            [](const PoseStamped& a, const PoseStamped& b) { return a.timestamp < b.timestamp; });

  // Pair each fix (in ENU around the reference) with the body position (in
  // the world) at the same time.
  struct Pair {
    Eigen::Vector3d world;
    Eigen::Vector3d enu;
    double w;
  };
  std::vector<Pair> pairs;
  for (const GpsFix& fix : fixes) {
    const auto p = position_at(sorted, fix.timestamp, config.max_pose_gap);
    if (p) {
      pairs.push_back({*p,
                       enu_offset(fix.latitude, fix.longitude, fix.altitude, geo.latitude,
                                  geo.longitude, geo.altitude),
                       1.0 / horizontal_variance(fix, config.default_gps_sigma)});
    }
  }
  if (pairs.size() < 3) {
    return geo;
  }

  // Weighted 2D Procrustes: enu = Rz(heading) * world + t.
  double w_total = 0.0;
  Eigen::Vector3d world_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d enu_mean = Eigen::Vector3d::Zero();
  for (const Pair& p : pairs) {
    w_total += p.w;
    world_mean += p.w * p.world;
    enu_mean += p.w * p.enu;
  }
  world_mean /= w_total;
  enu_mean /= w_total;
  double sxx = 0.0;
  double sxy = 0.0;
  double spread = 0.0;
  for (const Pair& p : pairs) {
    const Eigen::Vector2d a = (p.world - world_mean).head<2>();
    const Eigen::Vector2d b = (p.enu - enu_mean).head<2>();
    sxx += p.w * a.dot(b);
    sxy += p.w * (a.x() * b.y() - a.y() * b.x());
    spread += p.w * a.squaredNorm();
  }
  if (spread <= 0.0) {
    return geo;
  }
  const double heading = std::atan2(sxy, sxx);
  const Eigen::AngleAxisd rot(heading, Eigen::Vector3d::UnitZ());

  // Heading uncertainty from the track's spread, inflated when the fixes
  // scatter more than their reported accuracy.
  double chi2 = 0.0;
  for (const Pair& p : pairs) {
    const Eigen::Vector2d r = ((p.enu - enu_mean) - rot * (p.world - world_mean)).head<2>();
    chi2 += p.w * r.squaredNorm();
  }
  const double dof = 2.0 * static_cast<double>(pairs.size()) - 3.0;
  const double scale = std::max(1.0, chi2 / dof);
  geo.heading = heading;
  geo.heading_sigma = std::sqrt(scale / spread);

  // Move the reference to the world origin.
  const Eigen::Vector3d origin_enu = enu_mean - rot * world_mean;
  double rm = 0.0;
  double rn = 0.0;
  earth_radii(geo.latitude, rm, rn);
  geo.longitude += origin_enu.x() / ((rn + geo.altitude) * std::cos(geo.latitude * kDeg)) / kDeg;
  geo.latitude += origin_enu.y() / (rm + geo.altitude) / kDeg;
  geo.altitude += origin_enu.z();
  return geo;
}

}  // namespace vg
