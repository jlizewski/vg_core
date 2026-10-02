#include "vg_core/ground_segmentation.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <tuple>

#include "test_scene.hpp"

namespace {

constexpr double kVoxel = 0.02;

using Voxels = std::vector<Eigen::Vector3i>;

// A flat ground shell (k = -1, so the surface is at z = 0) over
// [x0, x1) by [y0, y1) voxels, leaving out cells for which skip returns true.
template <typename Skip>
void add_ground(Voxels& voxels, int x0, int x1, int y0, int y1, int k, Skip skip) {
  for (int y = y0; y < y1; ++y) {
    for (int x = x0; x < x1; ++x) {
      if (!skip(x, y)) {
        voxels.emplace_back(x, y, k);
      }
    }
  }
}
void add_ground(Voxels& voxels, int x0, int x1, int y0, int y1, int k = -1) {
  add_ground(voxels, x0, x1, y0, y1, k, [](int, int) { return false; });
}

// A raised bed's shell as seen by a camera: walls on the perimeter of
// [lo, hi] up to top, and a top layer, with no ground visible inside.
void add_bed(Voxels& voxels, int lo, int hi, int top) {
  for (int y = lo; y <= hi; ++y) {
    for (int x = lo; x <= hi; ++x) {
      const bool wall = x == lo || x == hi || y == lo || y == hi;
      for (int k = wall ? 0 : top; k <= top; ++k) {
        voxels.emplace_back(x, y, k);
      }
    }
  }
}

float ground_at(const vg::GroundMap& ground, double x, double y) {
  const auto& map = ground.heights;
  const int cx = static_cast<int>(std::floor((x - map.origin.x()) / map.cell_size));
  const int cy = static_cast<int>(std::floor((y - map.origin.y()) / map.cell_size));
  return map.at(cx, cy);
}

bool filled_at(const vg::GroundMap& ground, double x, double y) {
  const auto& map = ground.heights;
  const int cx = static_cast<int>(std::floor((x - map.origin.x()) / map.cell_size));
  const int cy = static_cast<int>(std::floor((y - map.origin.y()) / map.cell_size));
  return ground.is_filled(cx, cy);
}

TEST(GroundSegmentation, EmptyMap) {
  const auto result = vg::segment_ground(Voxels{}, kVoxel);
  EXPECT_TRUE(result.ground_voxels.empty());
  EXPECT_TRUE(result.segments.empty());
  EXPECT_EQ(result.ground.heights.width, 0);
}

TEST(GroundSegmentation, SeparatesBedAndCanopyFromGround) {
  // A 2 x 2 m yard with a 0.6 m raised bed (top at 0.3 m) whose inside hides
  // the ground, and a branch hanging 1 m over seen ground.
  Voxels voxels;
  add_ground(voxels, 0, 100, 0, 100, -1,
             [](int x, int y) { return x > 30 && x < 60 && y > 30 && y < 60; });
  add_bed(voxels, 30, 60, 14);
  for (int y = 10; y < 30; ++y) {
    for (int x = 70; x < 90; ++x) {
      voxels.emplace_back(x, y, 50);
    }
  }

  const auto result = vg::segment_ground(voxels, kVoxel);

  // Ground: everything at k = -1 plus the bottom of the bed walls.
  std::set<std::tuple<int, int, int>> ground;
  for (const auto& v : result.ground_voxels) {
    ground.emplace(v.x(), v.y(), v.z());
    EXPECT_LE(v.z(), 1) << v.transpose();
  }
  for (const auto& v : voxels) {
    if (v.z() == -1) {
      EXPECT_EQ(ground.count({v.x(), v.y(), v.z()}), 1u) << v.transpose();
    }
  }

  ASSERT_EQ(result.segments.size(), 2u);
  const auto& bed = result.segments[0];
  EXPECT_TRUE(bed.grounded);
  EXPECT_EQ(bed.min, Eigen::Vector3i(30, 30, 2));
  EXPECT_EQ(bed.max, Eigen::Vector3i(60, 60, 14));
  EXPECT_NEAR(bed.top_above_ground, 0.30, 1e-3);

  const auto& branch = result.segments[1];
  EXPECT_FALSE(branch.grounded);
  EXPECT_EQ(branch.voxels.size(), 400u);
  EXPECT_NEAR(branch.bottom_above_ground, 1.0, 1e-3);
  EXPECT_NEAR(branch.top_above_ground, 1.02, 1e-3);

  // The ground map is flat at z = 0, including the patch under the bed.
  const auto& map = result.ground.heights;
  EXPECT_DOUBLE_EQ(map.cell_size, 0.05);
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      ASSERT_TRUE(map.has_data(x, y)) << x << "," << y;
      EXPECT_NEAR(map.at(x, y), 0.0, 1e-4) << x << "," << y;
    }
  }
  EXPECT_TRUE(filled_at(result.ground, 0.9, 0.9));
  EXPECT_FALSE(filled_at(result.ground, 0.1, 0.1));
  EXPECT_FALSE(filled_at(result.ground, 1.6, 0.4));  // Under the branch, but seen.
}

TEST(GroundSegmentation, PatchUnderObjectFollowsSurroundingSlope) {
  // Ground rising 0.2 m per meter along x, with a box hiding a 0.5 m patch.
  Voxels voxels;
  auto ground_k = [](int x) {
    return static_cast<int>(std::floor(0.2 * (x + 0.5) * kVoxel / kVoxel)) - 1;
  };
  for (int y = 0; y < 100; ++y) {
    for (int x = 0; x < 100; ++x) {
      const bool hidden = x >= 40 && x < 65 && y >= 40 && y < 65;
      if (hidden) {
        voxels.emplace_back(x, y, 30);  // The box's top.
      } else {
        voxels.emplace_back(x, y, ground_k(x));
      }
    }
  }

  const auto result = vg::segment_ground(voxels, kVoxel);

  ASSERT_EQ(result.segments.size(), 1u);
  EXPECT_FALSE(result.segments[0].grounded);  // Its sides weren't seen.
  // The patch carries on the slope: z = 0.2 x, within a voxel.
  for (const double x : {0.85, 1.0, 1.25}) {
    EXPECT_TRUE(filled_at(result.ground, x, 1.0));
    EXPECT_NEAR(ground_at(result.ground, x, 1.0), 0.2 * x, 0.02) << x;
  }
}

TEST(GroundSegmentation, JoinsGroundAcrossUnseenGap) {
  // Two level patches of ground 0.3 m apart, e.g. either side of a shadow.
  Voxels voxels;
  add_ground(voxels, 0, 40, 0, 50);
  add_ground(voxels, 55, 80, 0, 50);

  const auto result = vg::segment_ground(voxels, kVoxel);

  EXPECT_EQ(result.ground_voxels.size(), voxels.size());
  EXPECT_TRUE(result.segments.empty());
  // The gap is open to the map's edge, so it's not patched.
  EXPECT_TRUE(std::isnan(ground_at(result.ground, 0.95, 0.5)));
}

TEST(GroundSegmentation, RaisedSurfaceAcrossGapIsNotGround) {
  // A deck 0.4 m up seen beyond a gap, with no ground visible under it.
  Voxels voxels;
  add_ground(voxels, 0, 60, 0, 50);
  add_ground(voxels, 70, 100, 0, 50, 19);

  const auto result = vg::segment_ground(voxels, kVoxel);

  EXPECT_EQ(result.ground_voxels.size(), 60u * 50u);
  ASSERT_EQ(result.segments.size(), 1u);
  EXPECT_EQ(result.segments[0].voxels.size(), 30u * 50u);
  EXPECT_FALSE(result.segments[0].grounded);
}

TEST(GroundSegmentation, PatchesOnlySmallEnclosedHoles) {
  // Unseen holes in the ground: 0.2 x 0.2 m (patched) and 1.2 x 1.2 m (left).
  Voxels voxels;
  add_ground(voxels, 0, 150, 0, 100, -1, [](int x, int y) {
    const bool small = x >= 20 && x < 30 && y >= 20 && y < 30;
    const bool large = x >= 70 && x < 130 && y >= 20 && y < 80;
    return small || large;
  });

  const auto result = vg::segment_ground(voxels, kVoxel);

  EXPECT_TRUE(filled_at(result.ground, 0.5, 0.5));
  EXPECT_NEAR(ground_at(result.ground, 0.5, 0.5), 0.0, 1e-4);
  EXPECT_TRUE(std::isnan(ground_at(result.ground, 2.0, 1.0)));
  EXPECT_FALSE(filled_at(result.ground, 2.0, 1.0));
}

TEST(GroundSegmentation, SeparatesRaisedBedInFusedMap) {
  vg::test::Scene scene;
  scene.box = vg::test::Box{{0.2, -0.3, 0.0}, {0.8, 0.3, 0.3}};
  vg::TsdfConfig config;
  config.max_depth = 2.5;
  vg::TsdfVolume volume(config);
  const Eigen::Vector3d target(0.5, 0.0, 0.0);
  for (const Eigen::Vector3d& eye :
       {Eigen::Vector3d(0.5, 0.0, 1.5), Eigen::Vector3d(-0.7, 0.5, 1.2),
        Eigen::Vector3d(1.7, -0.5, 1.2), Eigen::Vector3d(0.5, 1.2, 1.2),
        Eigen::Vector3d(0.5, -1.2, 1.2)}) {
    const auto pose = vg::test::look_at(eye, target, Eigen::Vector3d::UnitZ());
    volume.integrate(vg::test::render_depth(scene, pose), pose);
  }

  const auto result = vg::segment_ground(volume);

  ASSERT_FALSE(result.segments.empty());
  const auto& bed = result.segments[0];
  EXPECT_TRUE(bed.grounded);
  EXPECT_NEAR(bed.top_above_ground, 0.30, 0.03);
  for (const auto& v : result.ground_voxels) {
    const double x = (v.x() + 0.5) * kVoxel;
    const double y = (v.y() + 0.5) * kVoxel;
    EXPECT_FALSE(x > 0.24 && x < 0.76 && y > -0.26 && y < 0.26) << v.transpose();
    // Fusion rounds the corner where the walls meet the ground, so the ground
    // runs a few voxels up the foot of the walls, but never up the bed.
    EXPECT_LE(v.z(), 7) << v.transpose();
  }
  EXPECT_TRUE(filled_at(result.ground, 0.5, 0.0));
  EXPECT_NEAR(ground_at(result.ground, 0.5, 0.0), 0.0, 0.03);
}

}  // namespace
