#include "vg_core/map_builder.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <utility>

#include "test_scene.hpp"
#include "vg_core/height_map.hpp"

namespace {

using vg::test::look_at;
using vg::test::render_depth;

constexpr vg::Timestamp kMs = 1'000'000;

vg::test::Scene raised_bed() {
  vg::test::Scene scene;
  scene.box = vg::test::Box{{0.2, -0.3, 0.0}, {0.8, 0.3, 0.3}};
  return scene;
}

std::vector<Eigen::Isometry3d> walk() {
  const Eigen::Vector3d target(0.3, 0.0, 0.0);
  std::vector<Eigen::Isometry3d> poses;
  for (const Eigen::Vector3d& eye :
       {Eigen::Vector3d(0.0, 0.0, 1.5), Eigen::Vector3d(-1.0, 0.5, 1.4),
        Eigen::Vector3d(1.5, -0.5, 1.2), Eigen::Vector3d(0.3, 1.2, 1.3)}) {
    poses.push_back(look_at(eye, target, Eigen::Vector3d::UnitZ()));
  }
  return poses;
}

vg::MapBuilderConfig test_config() {
  vg::MapBuilderConfig config;
  config.tsdf.max_depth = 2.5;
  return config;
}

vg::PoseStamped pose_at(vg::Timestamp t, const Eigen::Isometry3d& world_from_body) {
  vg::PoseStamped pose;
  pose.timestamp = t;
  pose.world_from_frame = world_from_body;
  return pose;
}

vg::DepthFrame depth_at(vg::Timestamp t, const Eigen::Isometry3d& world_from_camera) {
  vg::DepthFrame frame = render_depth(raised_bed(), world_from_camera);
  frame.timestamp = t;
  return frame;
}

void expect_same_map(const vg::HeightMap& a, const vg::HeightMap& b) {
  ASSERT_EQ(a.width, b.width);
  ASSERT_EQ(a.height, b.height);
  EXPECT_NEAR(a.origin.x(), b.origin.x(), 1e-9);
  EXPECT_NEAR(a.origin.y(), b.origin.y(), 1e-9);
  for (int y = 0; y < a.height; ++y) {
    for (int x = 0; x < a.width; ++x) {
      ASSERT_EQ(a.has_data(x, y), b.has_data(x, y)) << x << "," << y;
      if (a.has_data(x, y)) {
        EXPECT_FLOAT_EQ(a.at(x, y), b.at(x, y)) << x << "," << y;
      }
    }
  }
}

TEST(MapBuilder, LiveHeightMapMatchesFullRebuild) {
  vg::MapBuilder builder(test_config());
  // Mirror of the live map built only from the reported changes, as a view would.
  std::map<std::pair<int, int>, float> view;

  vg::Timestamp t = 0;
  for (const auto& pose : walk()) {
    builder.add_pose(pose_at(t, pose));
    builder.add_depth("rgb", depth_at(t, pose));
    builder.flush();
    const auto changed = builder.update_height_map();
    EXPECT_FALSE(changed.empty());
    for (const vg::HeightCell& cell : changed) {
      if (std::isnan(cell.height)) {
        view.erase({cell.x, cell.y});
      } else {
        view[{cell.x, cell.y}] = cell.height;
      }
    }
    // After every frame, the incremental map equals one rebuilt from scratch.
    expect_same_map(builder.height_map(),
                    vg::make_height_map(builder.volume().extract_surface_points(), 0.05));
    t += 33 * kMs;
  }
  EXPECT_EQ(builder.stats().integrated, 4u);

  // The mirror kept from change lists matches the final map too.
  const auto map = builder.height_map();
  EXPECT_EQ(view.size(), builder.height_map_cells());
  const double cell = map.cell_size;
  for (const auto& [index, height] : view) {
    const int x = index.first - static_cast<int>(std::lround(map.origin.x() / cell));
    const int y = index.second - static_cast<int>(std::lround(map.origin.y() / cell));
    EXPECT_FLOAT_EQ(map.at(x, y), height);
  }

  // Nothing new integrated: nothing changes.
  EXPECT_TRUE(builder.update_height_map().empty());

  // The raised bed and the ground are where they should be.
  auto height_at = [&map](double x, double y) {
    const int cx = static_cast<int>(std::floor((x - map.origin.x()) / map.cell_size));
    const int cy = static_cast<int>(std::floor((y - map.origin.y()) / map.cell_size));
    return map.at(cx, cy);
  };
  EXPECT_NEAR(height_at(0.5, 0.0), 0.3, 0.03);
  EXPECT_NEAR(height_at(-0.4, 0.0), 0.0, 0.03);
}

TEST(MapBuilder, PairsDepthWithClosestPose) {
  const auto poses = walk();
  vg::MapBuilder builder(test_config());

  // Pose after its depth frame.
  builder.add_depth("rgb", depth_at(100 * kMs, poses[0]));
  builder.add_pose(pose_at(100 * kMs, poses[0]));
  // Pose before its depth frame, then a later pose that's further away.
  builder.add_pose(pose_at(200 * kMs, poses[1]));
  builder.add_depth("rgb", depth_at(205 * kMs, poses[1]));
  builder.add_pose(pose_at(240 * kMs, poses[2]));
  // No pose within 50 ms.
  builder.add_depth("rgb", depth_at(400 * kMs, poses[2]));
  builder.flush();

  EXPECT_EQ(builder.stats().integrated, 2u);
  EXPECT_EQ(builder.stats().skipped_no_pose, 1u);

  // With the right poses, the ground stays flat at z = 0.
  builder.update_height_map();
  const auto map = builder.height_map();
  const int x = static_cast<int>(std::floor((-0.4 - map.origin.x()) / map.cell_size));
  const int y = static_cast<int>(std::floor((0.0 - map.origin.y()) / map.cell_size));
  EXPECT_NEAR(map.at(x, y), 0.0, 0.03);
}

TEST(MapBuilder, SkipsFramesWhileTrackingIsLost) {
  const auto poses = walk();
  vg::MapBuilder builder(test_config());
  builder.set_tracking_ok(false);
  builder.add_pose(pose_at(0, poses[0]));
  builder.add_depth("rgb", depth_at(0, poses[0]));
  builder.set_tracking_ok(true);
  builder.add_pose(pose_at(33 * kMs, poses[1]));
  builder.add_depth("rgb", depth_at(33 * kMs, poses[1]));
  builder.flush();
  EXPECT_EQ(builder.stats().skipped_tracking, 1u);
  EXPECT_EQ(builder.stats().integrated, 1u);
}

TEST(MapBuilder, AppliesCameraExtrinsics) {
  // Camera 10 cm to the side of the body; the pose stream is the body's.
  Eigen::Isometry3d body_from_camera = Eigen::Isometry3d::Identity();
  body_from_camera.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);

  vg::MapBuilder builder(test_config());
  builder.set_camera_extrinsics("rgb", body_from_camera);
  vg::Timestamp t = 0;
  for (const auto& world_from_camera : walk()) {
    builder.add_pose(pose_at(t, world_from_camera * body_from_camera.inverse()));
    builder.add_depth("rgb", depth_at(t, world_from_camera));
    t += 33 * kMs;
  }
  builder.flush();
  builder.update_height_map();

  // Views would disagree by 10 cm if the offset were ignored; the bed's edge
  // at x = 0.2 stays sharp.
  const auto map = builder.height_map();
  auto height_at = [&map](double x, double y) {
    const int cx = static_cast<int>(std::floor((x - map.origin.x()) / map.cell_size));
    const int cy = static_cast<int>(std::floor((y - map.origin.y()) / map.cell_size));
    return map.at(cx, cy);
  };
  EXPECT_NEAR(height_at(0.28, 0.0), 0.3, 0.03);
  EXPECT_NEAR(height_at(0.12, 0.0), 0.0, 0.03);
}

}  // namespace
