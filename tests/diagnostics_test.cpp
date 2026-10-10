#include "vg_core/diagnostics.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr vg::Timestamp kMs = 1'000'000;

vg::PoseStamped pose(vg::Timestamp t, double x, double yaw = 0.0) {
  vg::PoseStamped p;
  p.timestamp = t;
  p.world_from_frame.translation() = Eigen::Vector3d(x, 0.0, 1.2);
  p.world_from_frame.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return p;
}

TEST(Diagnostics, FindsPoseJumpsAndGaps) {
  // Walking at 1 m/s at 30 Hz, with a 20 cm snap, a 30 degree snap and a gap.
  std::vector<vg::PoseStamped> poses;
  double x = 0.0;
  double yaw = 0.0;
  for (int i = 0; i < 100; ++i) {
    x += 1.0 / 30.0;
    if (i == 40) x += 0.2;
    if (i == 60) yaw += 30.0 * 3.14159265358979323846 / 180.0;
    const vg::Timestamp t = i * 33 * kMs + (i >= 80 ? 1000 * kMs : 0);
    poses.push_back(pose(t, x, yaw));
  }

  const auto stats = vg::analyze_pose_track(poses);
  EXPECT_EQ(stats.poses, 100u);
  EXPECT_EQ(stats.gaps, 1u);
  EXPECT_NEAR(stats.longest_gap, 1.033, 1e-3);
  ASSERT_EQ(stats.jumps.size(), 2u);
  // Largest first: the 30 degree turn (0.52 rad) beats 0.23 m.
  EXPECT_EQ(stats.jumps[0].timestamp, poses[60].timestamp);
  EXPECT_NEAR(stats.jumps[0].rotation, 0.5236, 1e-3);
  EXPECT_EQ(stats.jumps[1].timestamp, poses[40].timestamp);
  EXPECT_NEAR(stats.jumps[1].translation, 0.2333, 1e-3);
  // Path: the walk without the jumps and the step across the gap.
  EXPECT_NEAR(stats.path_length, 96.0 / 30.0, 1e-3);
}

TEST(Diagnostics, DepthStatsCountRangesEdgesAndConfidence) {
  vg::DepthFrame frame;
  frame.intrinsics.width = 4;
  frame.intrinsics.height = 1;
  // A step from 1.5 m to 3.5 m, and a pixel with no reading.
  frame.depth = {1.5f, 1.5f, 3.5f, 0.0f};
  frame.confidence = {255, 200, 10, 0};

  vg::DepthStats stats;
  stats.add(frame);
  EXPECT_EQ(stats.frames(), 1u);
  EXPECT_EQ(stats.pixels(), 4u);
  EXPECT_EQ(stats.valid(), 3u);
  EXPECT_EQ(stats.valid_by_range()[1], 2u);
  EXPECT_EQ(stats.valid_by_range()[3], 1u);
  // Both sides of the step are edges; the first pixel isn't.
  EXPECT_EQ(stats.edges_by_range()[1], 1u);
  EXPECT_EQ(stats.edges_by_range()[3], 1u);
  EXPECT_EQ(stats.by_confidence()[3], 2u);
  EXPECT_EQ(stats.by_confidence()[0], 1u);
}

TEST(Diagnostics, FindsStackedSurfaces) {
  // 0.1 m voxels, 0.2 m cells. Ground at k = -1 over 4 x 4 columns; a ghost
  // ground 0.3 m up (k = 2) over the first column of cell (0, 0), and a
  // canopy 2 m up over cell (1, 1).
  std::vector<Eigen::Vector3i> voxels;
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      voxels.emplace_back(x, y, -1);
    }
  }
  voxels.emplace_back(0, 0, 2);
  voxels.emplace_back(0, 0, 3);
  voxels.emplace_back(3, 3, 19);
  // One empty voxel between runs isn't a gap.
  voxels.emplace_back(2, 0, 1);

  const auto layers = vg::find_surface_layers(voxels, 0.1, 0.2);
  EXPECT_EQ(layers.columns, 16u);
  EXPECT_EQ(layers.layered_columns, 2u);
  EXPECT_EQ(layers.by_gap[1], 1u);  // 0.2 m: k = 0 and 1 empty.
  EXPECT_EQ(layers.by_gap[4], 1u);  // 1.9 m.
  const auto& map = layers.gap;
  ASSERT_EQ(map.width, 2);
  ASSERT_EQ(map.height, 2);
  EXPECT_NEAR(map.at(0, 0), 0.2, 1e-6);
  EXPECT_TRUE(std::isnan(map.at(1, 0)));
  EXPECT_NEAR(map.at(1, 1), 1.9, 1e-6);
}

}  // namespace
