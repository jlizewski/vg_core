#include "vg_core/live_guidance.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCell = 0.1;

// A 4 x 4 m yard of measured ground sloping up 0.1 m per m along x, with a
// 1 x 1 m unknown hole from (1.5, 1.5) to (2.5, 2.5).
vg::GroundLayer yard() {
  vg::GroundLayer g;
  g.heights.origin = Eigen::Vector2d::Zero();
  g.heights.cell_size = kCell;
  g.heights.width = 40;
  g.heights.height = 40;
  g.heights.heights.resize(1600);
  g.state.resize(1600);
  for (int y = 0; y < 40; ++y) {
    for (int x = 0; x < 40; ++x) {
      const bool hole = x >= 15 && x < 25 && y >= 15 && y < 25;
      g.state[g.index(x, y)] = hole ? vg::GroundState::Unknown : vg::GroundState::Measured;
      g.heights.heights[g.index(x, y)] =
          hole ? std::nanf("") : static_cast<float>(0.1 * (x + 0.5) * kCell);
    }
  }
  return g;
}

// Live cells covering the square [x0, x1) x [y0, y1) (world, m), at height z
// over the yard's slope (plus a fixed offset).
std::vector<vg::HeightCell> cells(double x0, double y0, double x1, double y1, double over,
                                  double offset = 0.0) {
  std::vector<vg::HeightCell> out;
  for (int y = static_cast<int>(std::floor(y0 / kCell)); (y + 0.5) * kCell < y1; ++y) {
    for (int x = static_cast<int>(std::floor(x0 / kCell)); (x + 0.5) * kCell < x1; ++x) {
      out.push_back({x, y, static_cast<float>(0.1 * (x + 0.5) * kCell + over + offset)});
    }
  }
  return out;
}

TEST(LiveGuidance, GroundAndLowThingsCloseAHoleACanopyDoesnt) {
  vg::LiveGuidance live(yard(), vg::MapTransform{});
  ASSERT_EQ(live.targets().size(), 1u);
  EXPECT_EQ(live.targets()[0].cells, 100u);
  EXPECT_NEAR(live.unseen_area(), 1.0, 1e-9);
  EXPECT_FALSE(live.done());

  // A canopy 3 m up over all of it: the ground under it is still unseen.
  live.update(cells(1.5, 1.5, 2.5, 2.5, 3.0), kCell);
  EXPECT_EQ(live.targets()[0].seen_cells, 0u);
  // Ground over half of it, and a 0.6 m box over a quarter.
  live.update(cells(1.5, 1.5, 2.0, 2.5, 0.02), kCell);
  live.update(cells(2.0, 1.5, 2.5, 2.0, 0.6), kCell);
  EXPECT_NEAR(live.targets()[0].seen, 0.75, 1e-9);
  EXPECT_FALSE(live.targets()[0].done);
  // A surface going away (NaN) doesn't unsee it; the rest closes it.
  live.update({{18, 18, std::nanf("")}}, kCell);
  live.update(cells(2.0, 2.0, 2.5, 2.5, 0.0), kCell);
  EXPECT_TRUE(live.targets()[0].done);
  EXPECT_TRUE(live.done());
  EXPECT_NEAR(live.unseen_area(), 0.0, 1e-9);
}

TEST(LiveGuidance, AWalkTurnedInTheSiteStillReachesEveryCell) {
  // The walk's world frame turned 30 degrees and shifted in the site's (and
  // 1 m lower): live cells along its own grid still cover the whole hole.
  vg::MapTransform site_from_world;
  site_from_world.yaw = 30.0 * kPi / 180.0;
  site_from_world.translation = Eigen::Vector3d(0.7, -0.4, 1.0);
  vg::LiveGuidance live(yard(), site_from_world);
  // The hole's corners in the world frame, and a generous box around them.
  const vg::MapTransform world_from_site = site_from_world.inverse();
  Eigen::Vector2d lo = Eigen::Vector2d::Constant(1e9);
  Eigen::Vector2d hi = -lo;
  for (const Eigen::Vector2d& c : {Eigen::Vector2d(1.5, 1.5), Eigen::Vector2d(2.5, 1.5),
                                   Eigen::Vector2d(1.5, 2.5), Eigen::Vector2d(2.5, 2.5)}) {
    lo = lo.cwiseMin(world_from_site.apply_xy(c));
    hi = hi.cwiseMax(world_from_site.apply_xy(c));
  }
  // Ground in the site at about 0.2 m: in the world, 1 m lower.
  std::vector<vg::HeightCell> seen;
  for (int y = static_cast<int>(std::floor(lo.y() / kCell)) - 1; y * kCell < hi.y() + kCell; ++y) {
    for (int x = static_cast<int>(std::floor(lo.x() / kCell)) - 1; x * kCell < hi.x() + kCell;
         ++x) {
      seen.push_back({x, y, 0.2f - 1.0f});
    }
  }
  live.update(seen, kCell);
  EXPECT_EQ(live.targets()[0].seen_cells, 100u);
}

TEST(LiveGuidance, SaysWhereToGoAndWhichWayToTurn) {
  vg::LiveGuidance live(yard(), vg::MapTransform{});
  EXPECT_FALSE(live.next().has_value());  // No camera pose yet.
  // The camera at (0.5, 2.0), 1.5 m up, looking along +y and down a little.
  Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();
  camera.linear() << 1, 0, 0,  //
      0, 0, 1,                 //
      0, -1, 0;                // x right, y down (-z), z forward (+y).
  camera.linear() =
      camera.linear() * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()).toRotationMatrix();
  camera.translation() = Eigen::Vector3d(0.5, 2.0, 1.5);
  live.set_camera_pose(camera);
  const auto next = live.next();
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->rank, 1);
  const auto& spots = live.targets()[0].spots;
  ASSERT_FALSE(spots.empty());
  EXPECT_NEAR(next->distance, (next->position - Eigen::Vector2d(0.5, 2.0)).norm(), 1e-9);
  EXPECT_GE(next->distance, 0.0);
  // Facing +y; a spot to the right (larger x) means turning right (negative).
  const Eigen::Vector2d to = next->position - Eigen::Vector2d(0.5, 2.0);
  const double expected = std::atan2(to.y(), to.x()) - kPi / 2.0;
  EXPECT_NEAR(next->turn, std::remainder(expected, 2.0 * kPi), 1e-6);
  EXPECT_NEAR(live.position()->y(), 2.0, 1e-9);
  // Once the hole is seen there's nowhere left to go.
  live.update(cells(1.5, 1.5, 2.5, 2.5, 0.0), kCell);
  EXPECT_FALSE(live.next().has_value());
}

TEST(LiveGuidance, ASiteWithoutTheWalksTrackingFrameCantPlaceIt) {
  const auto dir = std::filesystem::temp_directory_path() / "vg_live_guidance_no_site";
  std::filesystem::remove_all(dir);
  EXPECT_THROW(vg::LiveGuidance::for_site(dir, "abc"), std::runtime_error);
  std::filesystem::remove_all(dir);
}

}  // namespace
