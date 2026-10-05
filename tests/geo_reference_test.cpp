#include "vg_core/geo_reference.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <random>

namespace {

constexpr double kPi = 3.14159265358979323846;

// GPS fixes and poses for a walk around a square, with the world frame rotated
// by heading from East-North-Up and its origin at origin_enu.
struct Walk {
  std::vector<vg::GpsFix> fixes;
  std::vector<vg::PoseStamped> poses;
};

Walk make_walk(double heading, const Eigen::Vector3d& origin_enu, double ref_lat, double ref_lon,
               double gps_noise, double side) {
  Walk walk;
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, gps_noise);
  const Eigen::AngleAxisd enu_from_world(heading, Eigen::Vector3d::UnitZ());
  const double meters_per_deg_lat =
      vg::enu_offset(ref_lat + 1e-3, ref_lon, 0.0, ref_lat, ref_lon, 0.0).y() / 1e-3;
  const double meters_per_deg_lon =
      vg::enu_offset(ref_lat, ref_lon + 1e-3, 0.0, ref_lat, ref_lon, 0.0).x() / 1e-3;
  // Position along the walk at step i (0 to 400, may be fractional).
  const auto position = [side](double i) {
    const double s = 4.0 * side * i / 400.0;
    if (s < side) return Eigen::Vector3d(s, 0.0, 1.2);
    if (s < 2 * side) return Eigen::Vector3d(side, s - side, 1.2);
    if (s < 3 * side) return Eigen::Vector3d(3 * side - s, side, 1.2);
    return Eigen::Vector3d(0.0, 4 * side - s, 1.2);
  };
  for (int i = 0; i <= 400; ++i) {
    const vg::Timestamp t = static_cast<vg::Timestamp>(i) * 100'000'000;
    vg::PoseStamped pose;
    pose.timestamp = t;
    pose.world_from_frame.translation() = position(i);
    walk.poses.push_back(pose);
    if (i % 10 == 5) {
      // A fix between two poses.
      const Eigen::Vector3d enu = origin_enu + enu_from_world * position(i + 0.3);
      vg::GpsFix fix;
      fix.timestamp = t + 30'000'000;
      fix.latitude = ref_lat + (enu.y() + noise(rng)) / meters_per_deg_lat;
      fix.longitude = ref_lon + (enu.x() + noise(rng)) / meters_per_deg_lon;
      fix.altitude = 100.0 + enu.z();
      const double var = std::max(gps_noise * gps_noise, 1e-4);
      fix.position_covariance = Eigen::Vector3d(var, var, 4 * var).asDiagonal();
      walk.fixes.push_back(fix);
    }
  }
  return walk;
}

TEST(GeoReference, NoFixes) { EXPECT_FALSE(vg::estimate_geo_reference({}, {}).has_value()); }

TEST(GeoReference, FixesWithoutPosesGiveLocationOnly) {
  vg::GpsFix a;
  a.latitude = 45.0;
  a.longitude = -122.0;
  vg::GpsFix b = a;
  b.latitude = 45.001;
  const auto geo = vg::estimate_geo_reference({a, b}, {});
  ASSERT_TRUE(geo);
  EXPECT_NEAR(geo->latitude, 45.0005, 1e-9);
  EXPECT_NEAR(geo->longitude, -122.0, 1e-9);
  EXPECT_EQ(geo->heading, 0.0);
  EXPECT_TRUE(std::isinf(geo->heading_sigma));
}

TEST(GeoReference, RecoversHeadingAndOrigin) {
  const double heading = 0.6;
  const Walk walk = make_walk(heading, {12.0, -7.0, 0.0}, 45.0, -122.0, 0.0, 20.0);
  const auto geo = vg::estimate_geo_reference(walk.fixes, walk.poses);
  ASSERT_TRUE(geo);
  EXPECT_NEAR(geo->heading, heading, 1e-3);
  EXPECT_LT(geo->heading_sigma, 1e-3);
  // The world origin is 12 m east and 7 m south of the reference, at its
  // altitude.
  const Eigen::Vector3d origin =
      vg::enu_offset(geo->latitude, geo->longitude, geo->altitude, 45.0, -122.0, 100.0);
  EXPECT_NEAR(origin.x(), 12.0, 0.1);
  EXPECT_NEAR(origin.y(), -7.0, 0.1);
  EXPECT_NEAR(origin.z(), 0.0, 1e-6);

  const Eigen::Vector3d east_in_world = geo->world_from_enu(Eigen::Vector3d::UnitX());
  EXPECT_NEAR(east_in_world.x(), std::cos(heading), 1e-3);
  EXPECT_NEAR(east_in_world.y(), -std::sin(heading), 1e-3);
  EXPECT_TRUE(geo->enu_from_world(east_in_world).isApprox(Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(GeoReference, NoisyGpsReportsHeadingUncertainty) {
  const double heading = -2.0;
  const Walk wide = make_walk(heading, {0.0, 0.0, 0.0}, -33.9, 151.2, 3.0, 40.0);
  const Walk small = make_walk(heading, {0.0, 0.0, 0.0}, -33.9, 151.2, 3.0, 4.0);
  const auto geo_wide = vg::estimate_geo_reference(wide.fixes, wide.poses);
  const auto geo_small = vg::estimate_geo_reference(small.fixes, small.poses);
  ASSERT_TRUE(geo_wide && geo_small);
  const double error = std::remainder(geo_wide->heading - heading, 2 * kPi);
  EXPECT_LT(std::abs(error), 3.0 * geo_wide->heading_sigma);
  EXPECT_LT(geo_wide->heading_sigma, 0.1);
  EXPECT_GT(geo_small->heading_sigma, 5.0 * geo_wide->heading_sigma);
}

TEST(GeoReference, EnuOffset) {
  const Eigen::Vector3d north = vg::enu_offset(45.001, 10.0, 5.0, 45.0, 10.0, 0.0);
  EXPECT_NEAR(north.x(), 0.0, 1e-9);
  EXPECT_NEAR(north.y(), 111.13, 0.05);
  EXPECT_NEAR(north.z(), 5.0, 1e-12);
  const Eigen::Vector3d east = vg::enu_offset(45.0, 10.001, 0.0, 45.0, 10.0, 0.0);
  EXPECT_NEAR(east.x(), 78.85, 0.05);
  // Across the antimeridian.
  const Eigen::Vector3d wrap = vg::enu_offset(0.0, -179.9995, 0.0, 0.0, 179.9995, 0.0);
  EXPECT_NEAR(wrap.x(), 111.32, 0.05);
}

}  // namespace
