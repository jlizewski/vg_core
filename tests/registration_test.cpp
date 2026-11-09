#include "vg_core/registration.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCell = 0.1524;
constexpr double kVoxel = 0.0762;

// A yard in world coordinates: flat ground at z = 0 over [0, 16]^2 and beds,
// a shed and posts standing on it, laid out with no symmetry.
double yard(double x, double y) {
  const auto in = [&](double x0, double y0, double x1, double y1) {
    return x >= x0 && x < x1 && y >= y0 && y < y1;
  };
  if (in(2, 2, 6, 3.2)) return 0.4;     // Long bed.
  if (in(9, 1, 10.2, 5)) return 0.3;    // Bed along y.
  if (in(11, 10, 14, 13.5)) return 2.2;  // Shed.
  if (in(3, 9, 3.3, 9.3)) return 1.2;   // Post.
  if (in(6, 12, 6.3, 12.3)) return 1.2;  // Post.
  if (in(4.5, 6, 7.5, 7)) return 0.6;   // Raised bed.
  return 0.0;
}

// The yard as seen by a walk whose frame is world_from_map.inverse()(world):
// its features, terrain and voxels in its own coordinates.
using Yard = std::function<double(double, double)>;

// Two identical L-shaped beds 8 m apart, and nothing else: a walk seeing one
// fits either.
double twin_beds(double x, double y) {
  const auto bed = [&](double x0) {
    return (x >= x0 && x < x0 + 3 && y >= 6 && y < 7) ||
           (x >= x0 && x < x0 + 1 && y >= 7 && y < 9);
  };
  return bed(2.0) || bed(10.0) ? 0.5 : 0.0;
}

vg::RegistrationInput walk(const vg::MapTransform& world_from_map,
                           const std::function<bool(double, double)>& seen = nullptr,
                           const Yard& yard = ::yard) {
  const vg::MapTransform map_from_world = world_from_map.inverse();
  vg::RegistrationInput in;
  // Grids in the map's frame covering the yard.
  Eigen::Vector2d lo(1e9, 1e9);
  Eigen::Vector2d hi(-1e9, -1e9);
  for (const Eigen::Vector2d c : {Eigen::Vector2d(0, 0), Eigen::Vector2d(16, 0),
                                  Eigen::Vector2d(0, 16), Eigen::Vector2d(16, 16)}) {
    const Eigen::Vector2d p = map_from_world.apply_xy(c);
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  for (vg::HeightMap* m : {&in.features, &in.terrain}) {
    m->origin = lo;
    m->cell_size = kCell;
    m->width = static_cast<int>((hi.x() - lo.x()) / kCell);
    m->height = static_cast<int>((hi.y() - lo.y()) / kCell);
    m->heights.assign(static_cast<std::size_t>(m->width * m->height), NAN);
  }
  for (int y = 0; y < in.features.height; ++y) {
    for (int x = 0; x < in.features.width; ++x) {
      const Eigen::Vector2d w =
          world_from_map.apply_xy(lo + Eigen::Vector2d(x + 0.5, y + 0.5) * kCell);
      if (w.x() < 0 || w.y() < 0 || w.x() >= 16 || w.y() >= 16 || (seen && !seen(w.x(), w.y()))) {
        continue;
      }
      const auto i = static_cast<std::size_t>(y * in.features.width + x);
      in.features.heights[i] = static_cast<float>(yard(w.x(), w.y()));
      in.terrain.heights[i] = static_cast<float>(map_from_world.translation.z());
    }
  }
  // Voxels: the top surface, and the sides of what stands up.
  for (double x = 0.0; x < 16.0; x += kVoxel) {
    for (double y = 0.0; y < 16.0; y += kVoxel) {
      if (seen && !seen(x, y)) {
        continue;
      }
      const double h = yard(x, y);
      for (double z = 0.0; z <= h + 1e-9; z += kVoxel) {
        const bool edge = h > 0 && (yard(x + kVoxel, y) < h || yard(x - kVoxel, y) < h ||
                                    yard(x, y + kVoxel) < h || yard(x, y - kVoxel) < h);
        if (z + kVoxel > h || edge) {
          in.points.push_back(map_from_world.apply({x, y, z}));
        }
      }
    }
  }
  return in;
}

vg::MapTransform transform(double yaw_deg, double x, double y, double z) {
  vg::MapTransform t;
  t.yaw = yaw_deg * kPi / 180.0;
  t.translation = {x, y, z};
  return t;
}

double yaw_error_deg(double a, double b) {
  return std::abs(std::remainder(a - b, 2.0 * kPi)) * 180.0 / kPi;
}

TEST(MapTransformTest, ComposesAndInverts) {
  const auto a = transform(30, 1, 2, 0.5);
  const auto b = transform(-75, -3, 0.5, 0.1);
  const Eigen::Vector3d p(0.3, -1.2, 2.0);
  EXPECT_TRUE(a.after(b).apply(p).isApprox(a.apply(b.apply(p)), 1e-12));
  EXPECT_TRUE(a.inverse().apply(a.apply(p)).isApprox(p, 1e-12));
}

TEST(RegistrationTest, FindsAWalkTurnedAndShiftedWithoutGps) {
  const auto reference = walk({});
  // The second walk's frame: turned 137 degrees, 3 m away, 0.4 m lower.
  const auto world_from_map = transform(137, 3.0, -2.0, 0.4);
  const vg::RegistrationResult r = vg::register_map(reference, walk(world_from_map));
  ASSERT_TRUE(r.accepted) << r.problem;
  EXPECT_FALSE(r.used_prior);
  EXPECT_LT(yaw_error_deg(r.reference_from_map.yaw, world_from_map.yaw), 0.5);
  EXPECT_LT((r.reference_from_map.translation - world_from_map.translation).norm(), 0.05);
  EXPECT_GT(r.inliers_10cm, 0.9);
  EXPECT_TRUE(r.rival_ratio == 0.0 || r.rival_ratio > 1.3);
}

TEST(RegistrationTest, MatchesPartlyOverlappingWalks) {
  // Each walk saw a different, overlapping part of the yard.
  const auto reference = walk({}, [](double x, double) { return x < 11.0; });
  const auto world_from_map = transform(-60, -4.0, 5.0, -0.2);
  const auto r = vg::register_map(reference, walk(world_from_map, [](double x, double) {
                                    return x > 4.0;
                                  }));
  ASSERT_TRUE(r.accepted) << r.problem;
  EXPECT_LT(yaw_error_deg(r.reference_from_map.yaw, world_from_map.yaw), 0.5);
  // Within about a voxel: less of the yard to pin it down.
  EXPECT_LT((r.reference_from_map.translation - world_from_map.translation).norm(), 0.1);
  EXPECT_LT(r.overlap, 0.9);
}

TEST(RegistrationTest, SearchesNearAGpsHeading) {
  const auto world_from_map = transform(200, 1.0, 1.0, 0.0);
  vg::RegistrationPrior prior;
  prior.yaw = world_from_map.yaw + 0.05;
  prior.yaw_sigma = 0.05;
  const auto r = vg::register_map(walk({}), walk(world_from_map), {}, prior);
  ASSERT_TRUE(r.accepted) << r.problem;
  EXPECT_TRUE(r.used_prior);
  EXPECT_LT(yaw_error_deg(r.reference_from_map.yaw, world_from_map.yaw), 0.5);
}

TEST(RegistrationTest, GpsPositionPicksBetweenPlacesThatFitAlike) {
  const auto reference = walk({}, nullptr, twin_beds);
  // A walk around the second bed only.
  const auto world_from_map = transform(30, 9.0, 4.0, 0.0);
  const auto second_bed = [](double x, double) { return x > 8.0; };
  const auto map = walk(world_from_map, second_bed, twin_beds);

  // Without GPS: two places fit alike.
  const auto blind = vg::register_map(reference, map);
  EXPECT_FALSE(blind.accepted);
  EXPECT_NE(blind.problem.find("ambiguous"), std::string::npos) << blind.problem;

  // GPS puts the walk's origin within a few meters of the truth.
  vg::RegistrationPrior prior;
  prior.yaw = world_from_map.yaw + 0.03;
  prior.yaw_sigma = 0.05;
  prior.origin = world_from_map.translation.head<2>() + Eigen::Vector2d(2.0, -1.5);
  prior.origin_sigma = 4.0;
  const auto r = vg::register_map(reference, map, {}, prior);
  ASSERT_TRUE(r.accepted) << r.problem;
  // One bed pins it down less tightly than a yard.
  EXPECT_LT((r.reference_from_map.translation - world_from_map.translation).norm(), 0.15);
}

TEST(RegistrationTest, RefusesWalksThatDontOverlap) {
  const auto reference = walk({}, [](double x, double) { return x < 7.0; });
  const auto r = vg::register_map(reference, walk({}, [](double x, double) { return x > 9.0; }));
  EXPECT_FALSE(r.accepted);
  EXPECT_FALSE(r.problem.empty());
}

TEST(RegistrationTest, RefusesAMapWithoutFeatures) {
  vg::RegistrationInput flat;
  flat.points = {{0, 0, 0}};
  const auto r = vg::register_map(walk({}), flat);
  EXPECT_FALSE(r.accepted);
  EXPECT_NE(r.problem.find("features"), std::string::npos);
}

}  // namespace
