#include "vg_core/tsdf_volume.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "test_scene.hpp"
#include "vg_core/height_map.hpp"

namespace {

using vg::test::look_at;
using vg::test::render_depth;
using vg::test::Scene;

const Eigen::Isometry3d kTopDown =
    look_at({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0}, Eigen::Vector3d::UnitY());

TEST(TsdfVolume, FlatGroundSurfaceIsAtZero) {
  vg::TsdfVolume volume;
  volume.integrate(render_depth(Scene{}, kTopDown), kTopDown);

  const auto points = volume.extract_surface_points();
  ASSERT_GT(points.size(), 1000u);
  for (const auto& p : points) {
    EXPECT_NEAR(p.z(), 0.0, volume.config().voxel_size / 2);
  }
}

TEST(TsdfVolume, DistanceIsSignedAroundSurface) {
  vg::TsdfVolume volume;
  volume.integrate(render_depth(Scene{}, kTopDown), kTopDown);

  const double tolerance = volume.config().voxel_size;
  const auto above = volume.distance_at({0.1, 0.1, 0.05});
  const auto below = volume.distance_at({0.1, 0.1, -0.05});
  ASSERT_TRUE(above && below);
  EXPECT_NEAR(*above, 0.05, tolerance);
  EXPECT_NEAR(*below, -0.05, tolerance);

  // Far above the ground there is no allocated block.
  EXPECT_FALSE(volume.distance_at({0.1, 0.1, 1.0}));
}

TEST(TsdfVolume, HeightMapShowsRaisedBed) {
  Scene scene;
  scene.box = vg::test::Box{{0.2, -0.3, 0.0}, {0.8, 0.3, 0.3}};

  // Keep the oblique views to the area of interest, so the test stays fast in
  // unoptimized builds.
  vg::TsdfConfig config;
  config.max_depth = 2.5;
  vg::TsdfVolume volume(config);
  const Eigen::Vector3d target(0.3, 0.0, 0.0);
  for (const Eigen::Vector3d& eye :
       {Eigen::Vector3d(0.0, 0.0, 1.5), Eigen::Vector3d(-1.0, 0.5, 1.4),
        Eigen::Vector3d(1.5, -0.5, 1.2)}) {
    const auto pose = look_at(eye, target, Eigen::Vector3d::UnitZ());
    volume.integrate(render_depth(scene, pose), pose);
  }

  const auto map = vg::make_height_map(volume.extract_surface_points(), 0.05);
  auto height_at = [&map](double x, double y) {
    const int cx = static_cast<int>(std::floor((x - map.origin.x()) / map.cell_size));
    const int cy = static_cast<int>(std::floor((y - map.origin.y()) / map.cell_size));
    return map.at(cx, cy);
  };
  EXPECT_NEAR(height_at(0.5, 0.0), 0.3, 0.03);
  EXPECT_NEAR(height_at(-0.4, 0.0), 0.0, 0.03);
  EXPECT_NEAR(height_at(1.2, 0.1), 0.0, 0.03);
}

TEST(TsdfVolume, SkipsInvalidAndLowConfidenceDepth) {
  auto frame = render_depth(Scene{}, kTopDown);

  vg::TsdfConfig config;
  config.min_confidence = 128;
  frame.confidence.assign(frame.depth.size(), 50);
  vg::TsdfVolume low_confidence(config);
  low_confidence.integrate(frame, kTopDown);
  EXPECT_EQ(low_confidence.num_blocks(), 0u);

  frame.confidence.clear();
  for (float& d : frame.depth) {
    d = std::numeric_limits<float>::quiet_NaN();
  }
  vg::TsdfVolume no_depth;
  no_depth.integrate(frame, kTopDown);
  EXPECT_EQ(no_depth.num_blocks(), 0u);
}

TEST(TsdfVolume, ReportsChangedBlocksOnce) {
  vg::TsdfVolume volume;
  volume.integrate(render_depth(Scene{}, kTopDown), kTopDown);

  const auto changed = volume.take_changed_blocks();
  EXPECT_EQ(changed.size(), volume.num_blocks());
  EXPECT_TRUE(volume.take_changed_blocks().empty());

  // Per-block points add up to the full extraction.
  std::size_t total = 0;
  for (const auto& block : changed) {
    total += volume.extract_surface_points(block).size();
  }
  EXPECT_EQ(total, volume.extract_surface_points().size());
  EXPECT_TRUE(volume.extract_surface_points(Eigen::Vector3i(1000, 1000, 1000)).empty());
}

TEST(TsdfVolume, OccupiedVoxelsFormShellBelowGround) {
  vg::TsdfVolume volume;
  volume.integrate(render_depth(Scene{}, kTopDown), kTopDown);

  const auto voxels = volume.occupied_voxels();
  ASSERT_GT(voxels.size(), 1000u);
  // The ground at z = 0 is the top face of the voxel layer k = -1.
  for (const auto& v : voxels) {
    EXPECT_EQ(v.z(), -1);
  }

  std::size_t per_block = 0;
  for (const auto& block : volume.take_changed_blocks()) {
    per_block += volume.occupied_voxels(block).size();
  }
  EXPECT_EQ(per_block, voxels.size());
}

TEST(TsdfVolume, OccupiedVoxelsShowRaisedBed) {
  Scene scene;
  scene.box = vg::test::Box{{0.2, -0.3, 0.0}, {0.8, 0.3, 0.3}};
  vg::TsdfVolume volume;
  volume.integrate(render_depth(scene, kTopDown), kTopDown);

  // Seen from above: a layer just under the bed's top (k = 14, z 0.28 to 0.30)
  // over the bed, and just under the ground (k = -1) elsewhere.
  std::size_t top = 0;
  for (const auto& v : volume.occupied_voxels()) {
    const double x = (v.x() + 0.5) * 0.02;
    const double y = (v.y() + 0.5) * 0.02;
    const bool inside = x > 0.22 && x < 0.78 && y > -0.28 && y < 0.28;
    const bool outside = x < 0.18 || x > 0.82 || y < -0.32 || y > 0.32;
    if (inside) {
      EXPECT_EQ(v.z(), 14) << v.transpose();
      ++top;
    } else if (outside) {
      EXPECT_LE(v.z(), 14) << v.transpose();
    }
  }
  EXPECT_GT(top, 500u);  // The bed's top is 0.6 x 0.6 m: ~28 x 28 voxels.
}

TEST(HeightMap, KeepsHighestPointPerCell) {
  const auto map =
      vg::make_height_map({{0.05, 0.05, 1.0}, {0.07, 0.02, 2.0}, {0.25, 0.05, 0.5}}, 0.1);
  EXPECT_DOUBLE_EQ(map.origin.x(), 0.0);
  EXPECT_DOUBLE_EQ(map.origin.y(), 0.0);
  ASSERT_EQ(map.width, 3);
  ASSERT_EQ(map.height, 1);
  EXPECT_FLOAT_EQ(map.at(0, 0), 2.0f);
  EXPECT_FALSE(map.has_data(1, 0));
  EXPECT_FLOAT_EQ(map.at(2, 0), 0.5f);
}

}  // namespace
