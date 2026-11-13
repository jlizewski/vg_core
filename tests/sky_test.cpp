#include "vg_core/sky.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>

#include "vg_core/semantics.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

// A canopy: a slab 6-8 m up over [-3, 3] x [-3, 3], flat ground at z = 0.
bool in_canopy(const Eigen::Vector3d& p) {
  return std::abs(p.x()) <= 3.0 && std::abs(p.y()) <= 3.0 && p.z() >= 6.0 && p.z() <= 8.0;
}

// What a camera at eye looking toward yaw, tilted up by pitch, sees: sky
// where its rays miss the canopy, tree (with no depth reading: too far) where
// they hit it.
std::pair<vg::DepthFrame, Eigen::Isometry3d> look_up(const Eigen::Vector3d& eye, double yaw,
                                                     double pitch) {
  vg::DepthFrame frame;
  frame.intrinsics = {32, 24, 20.0, 20.0, 16.0, 12.0};
  frame.depth.assign(32 * 24, 0.0f);
  frame.labels.assign(32 * 24, 0);
  // Optical frame: x right, y down, z forward.
  const Eigen::Vector3d forward(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw),
                                std::sin(pitch));
  const Eigen::Vector3d right = forward.cross(Eigen::Vector3d::UnitZ()).normalized();
  const Eigen::Vector3d down = forward.cross(right);
  Eigen::Isometry3d world_from_camera = Eigen::Isometry3d::Identity();
  world_from_camera.linear() << right, down, forward;
  world_from_camera.translation() = eye;
  for (int v = 0; v < 24; ++v) {
    for (int u = 0; u < 32; ++u) {
      const Eigen::Vector3d d =
          world_from_camera.linear() *
          Eigen::Vector3d((u + 0.5 - 16.0) / 20.0, (v + 0.5 - 12.0) / 20.0, 1.0).normalized();
      bool hit = false;
      for (double s = 0.1; s < 40.0 && !hit; s += 0.1) {
        hit = in_canopy(eye + d * s);
      }
      frame.labels[static_cast<std::size_t>(v * 32 + u)] = static_cast<std::uint8_t>(
          hit ? vg::SemanticClass::Tree : vg::SemanticClass::Sky);
    }
  }
  return {frame, world_from_camera};
}

vg::SkyEvidence walk_under_canopy() {
  vg::SkyEvidence sky(0.5);
  vg::SkyConfig config;
  config.pixel_stride = 1;
  for (double x = -8.0; x <= 8.0; x += 2.0) {
    for (double y = -6.0; y <= 6.0; y += 6.0) {
      for (int k = 0; k < 8; ++k) {
        const auto [frame, pose] = look_up({x, y, 1.5}, k * kPi / 4.0, 50.0 * kPi / 180.0);
        sky.add(frame, pose, config);
      }
    }
  }
  return sky;
}

vg::HeightMap flat_terrain() {
  vg::HeightMap t;
  t.origin = {-20.0, -20.0};
  t.cell_size = 0.5;
  t.width = t.height = 80;
  t.heights.assign(80 * 80, 0.0f);
  return t;
}

TEST(SkyTest, CarvesTheCanopyFromSkyAndBlockedRays) {
  const vg::SkyEvidence sky = walk_under_canopy();
  EXPECT_EQ(sky.frames(), 9u * 3u * 8u);
  vg::CanopyConfig config;
  config.min_blocked = 5;
  const vg::Canopy canopy = vg::make_canopy(sky, flat_terrain(), config);
  ASSERT_FALSE(canopy.cells.empty());
  // Nearly every canopy cell is over the slab's footprint, from its bottom
  // up. (Above the slab is never seen from below, so it stays in the hull:
  // the canopy's top is only an upper bound.)
  std::size_t inside = 0;
  for (const auto& c : canopy.cells) {
    const Eigen::Vector3d center = (c.cast<double>().array() + 0.5) * 0.5;
    inside += std::abs(center.x()) <= 4.0 && std::abs(center.y()) <= 4.0 && center.z() >= 5.0;
  }
  EXPECT_GT(static_cast<double>(inside) / static_cast<double>(canopy.cells.size()), 0.95);
  // Over the middle, the canopy starts about 6 m up.
  const auto& b = canopy.bottom;
  const int mx = static_cast<int>(std::floor((0.25 - b.origin.x()) / b.cell_size));
  const int my = static_cast<int>(std::floor((0.25 - b.origin.y()) / b.cell_size));
  ASSERT_TRUE(mx >= 0 && my >= 0 && mx < b.width && my < b.height);
  ASSERT_TRUE(b.has_data(mx, my));
  EXPECT_NEAR(b.at(mx, my), 6.0, 1.0);
}

TEST(SkyTest, FramesWithoutLabelsAddNothing) {
  vg::SkyEvidence sky(0.5);
  auto [frame, pose] = look_up({0, 0, 1.5}, 0.0, 0.8);
  frame.labels.clear();
  sky.add(frame, pose, {});
  EXPECT_EQ(sky.frames(), 0u);
  EXPECT_TRUE(sky.empty());
}

TEST(SkyTest, NearReadingsAreNotBlockedBeyond) {
  // A tree trunk in reach of the depth: mapped by it, not evidence of canopy.
  vg::SkyEvidence sky(0.5);
  auto [frame, pose] = look_up({0, 0, 1.5}, 0.0, 0.8);
  for (std::size_t i = 0; i < frame.labels.size(); ++i) {
    frame.labels[i] = static_cast<std::uint8_t>(vg::SemanticClass::Tree);
    frame.depth[i] = 2.0f;
  }
  sky.add(frame, pose, {});
  EXPECT_EQ(sky.frames(), 1u);
  EXPECT_TRUE(sky.empty());
}

TEST(SkyTest, RoundTripsAndCombinesInAnotherFrame) {
  const vg::SkyEvidence sky = walk_under_canopy();
  const auto path = std::filesystem::temp_directory_path() / "vg_sky_test.vgs";
  vg::save_sky(sky, path);
  const vg::SkyEvidence loaded = vg::load_sky(path);
  std::filesystem::remove(path);
  EXPECT_EQ(loaded.frames(), sky.frames());
  EXPECT_EQ(loaded.size(), sky.size());

  // The same evidence moved 10 m east lands its canopy 10 m east.
  vg::MapTransform east;
  east.translation = {10.0, 0.0, 0.0};
  vg::SkyEvidence moved(0.5);
  moved.add(sky, east);
  vg::CanopyConfig config;
  config.min_blocked = 5;
  vg::HeightMap terrain = flat_terrain();
  terrain.width = 120;  // Out to x = 40.
  terrain.heights.assign(120 * 80, 0.0f);
  const auto a = vg::make_canopy(sky, terrain, config);
  const auto b = vg::make_canopy(moved, terrain, config);
  EXPECT_EQ(a.cells.size(), b.cells.size());
  EXPECT_NEAR(b.bottom.origin.x() - a.bottom.origin.x(), 10.0, 1e-9);
}

}  // namespace
