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

// Fuses the raised bed from a few views into volume.
void fuse_bed(vg::TsdfVolume& volume) {
  Scene scene;
  scene.box = vg::test::Box{{0.2, -0.3, 0.0}, {0.8, 0.3, 0.3}};
  const Eigen::Vector3d target(0.3, 0.0, 0.0);
  for (const Eigen::Vector3d& eye :
       {Eigen::Vector3d(0.0, 0.0, 1.5), Eigen::Vector3d(-1.0, 0.5, 1.4),
        Eigen::Vector3d(1.5, -0.5, 1.2)}) {
    const auto pose = look_at(eye, target, Eigen::Vector3d::UnitZ());
    volume.integrate(render_depth(scene, pose), pose);
  }
}

TEST(TsdfVolume, ThreadsGiveTheSameVolume) {
  vg::TsdfConfig config;
  config.max_depth = 2.5;
  vg::TsdfVolume serial(config);
  config.threads = 4;
  vg::TsdfVolume threaded(config);
  fuse_bed(serial);
  fuse_bed(threaded);

  const auto indices = serial.block_indices();
  ASSERT_EQ(indices, threaded.block_indices());
  for (const auto& index : indices) {
    const auto& a = *serial.find_block(index);
    const auto& b = *threaded.find_block(index);
    for (std::size_t i = 0; i < a.size(); ++i) {
      ASSERT_EQ(a[i].tsdf, b[i].tsdf);
      ASSERT_EQ(a[i].weight, b[i].weight);
    }
  }
}

TEST(TsdfVolume, PixelStrideKeepsCoarseMap) {
  // At 3 inch voxels, placing blocks from every other pixel finds the same
  // surface as every pixel.
  vg::TsdfConfig config;
  config.max_depth = 2.5;
  config.voxel_size = 0.0762;
  config.truncation_distance = 3 * config.voxel_size;
  vg::TsdfVolume every(config);
  config.pixel_stride = 2;
  vg::TsdfVolume strided(config);
  fuse_bed(every);
  fuse_bed(strided);

  const auto all = every.occupied_voxels();
  const auto some = strided.occupied_voxels();
  ASSERT_GT(all.size(), 100u);
  EXPECT_GE(static_cast<double>(some.size()), 0.98 * static_cast<double>(all.size()));
  EXPECT_LE(strided.num_blocks(), every.num_blocks());
}

TEST(TsdfVolume, QualityWeightsFavorNearHeadOnReadings) {
  vg::TsdfConfig config;
  config.quality_weights = true;
  const auto weight_after_one_frame = [&](double height) {
    vg::TsdfVolume volume(config);
    const auto pose = look_at({0.0, 0.0, height}, {0.0, 0.0, 0.0}, Eigen::Vector3d::UnitY());
    volume.integrate(render_depth(Scene{}, pose), pose);
    // The voxel just under the ground straight below the camera, seen head-on.
    const auto* block = volume.find_block(Eigen::Vector3i(0, 0, -1));
    return block == nullptr ? 0.0f : (*block)[static_cast<std::size_t>(7 * 64)].weight;
  };
  // Head-on at 1 m weighs 1; at 2 m, a quarter.
  EXPECT_NEAR(weight_after_one_frame(1.0), 1.0f, 0.01f);
  EXPECT_NEAR(weight_after_one_frame(2.0), 0.25f, 0.01f);
}

TEST(TsdfVolume, MinWeightNeedsRepeatedObservations) {
  // Seen from 3 m up, each frame adds about 0.11 per voxel.
  vg::TsdfConfig config;
  config.quality_weights = true;
  config.min_weight = 0.5f;
  vg::TsdfVolume volume(config);
  const auto pose = look_at({0.0, 0.0, 3.0}, {0.0, 0.0, 0.0}, Eigen::Vector3d::UnitY());
  const auto frame = render_depth(Scene{}, pose);
  volume.integrate(frame, pose);
  EXPECT_EQ(volume.occupied_voxels().size(), 0u);
  EXPECT_EQ(volume.extract_surface_points().size(), 0u);
  for (int i = 0; i < 5; ++i) {
    volume.integrate(frame, pose);
  }
  EXPECT_GT(volume.occupied_voxels().size(), 1000u);
  EXPECT_GT(volume.extract_surface_points().size(), 1000u);
  // Lowering the bar on a built map brings back what a single frame saw.
  vg::TsdfVolume once(config);
  once.integrate(frame, pose);
  once.set_min_weight(0.0f);
  EXPECT_GT(once.occupied_voxels().size(), 1000u);
}

// A frame from above with a bad patch: a blob 0.8 m over the ground, as
// smeared or hallucinated depth makes. Then views from around it that see
// the ground through where the blob is.
std::size_t blob_voxels_after_looking_through(bool carve) {
  vg::TsdfConfig config;
  config.voxel_size = 0.05;
  config.truncation_distance = 0.15;
  config.carve = carve;
  vg::TsdfVolume volume(config);
  const auto above = look_at({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0}, Eigen::Vector3d::UnitY());
  auto bad = render_depth(Scene{}, above);
  for (int v = 50; v < 70; ++v) {
    for (int u = 70; u < 90; ++u) {
      bad.depth[bad.index(u, v)] = 0.7f;
    }
  }
  volume.integrate(bad, above);
  for (int i = 0; i < 8; ++i) {
    const double a = i * 3.14159265358979323846 / 4.0;
    const Eigen::Vector3d eye(1.2 * std::cos(a), 1.2 * std::sin(a), 1.2);
    const Eigen::Vector3d through(-1.2 * std::cos(a), -1.2 * std::sin(a), 0.4);
    const auto pose = look_at(eye, through, Eigen::Vector3d::UnitZ());
    volume.integrate(render_depth(Scene{}, pose), pose);
  }
  std::size_t blob = 0;
  for (const auto& v : volume.occupied_voxels()) {
    blob += v.z() * config.voxel_size > 0.4 ? 1 : 0;
  }
  return blob;
}

TEST(TsdfVolume, CarvingRemovesWhatLaterFramesSeeThrough) {
  EXPECT_GT(blob_voxels_after_looking_through(false), 20u);
  EXPECT_EQ(blob_voxels_after_looking_through(true), 0u);
}

TEST(TsdfVolume, CarvingKeepsTheSurface) {
  vg::TsdfConfig config;
  config.max_depth = 2.5;
  config.voxel_size = 0.0762;
  config.truncation_distance = 3 * config.voxel_size;
  vg::TsdfVolume plain(config);
  config.carve = true;
  vg::TsdfVolume carved(config);
  fuse_bed(plain);
  fuse_bed(carved);
  // The same surface, give or take a few voxels at its edges.
  const auto a = plain.occupied_voxels().size();
  const auto b = carved.occupied_voxels().size();
  EXPECT_NEAR(static_cast<double>(b), static_cast<double>(a), 0.02 * static_cast<double>(a));
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
