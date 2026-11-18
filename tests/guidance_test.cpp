#include "vg_core/guidance.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

using vg::GroundState;

constexpr double kPi = 3.14159265358979323846;

// A 12 x 12 m site of measured ground, 0.2 m cells.
vg::GroundLayer site() {
  vg::GroundLayer g;
  g.heights.cell_size = 0.2;
  g.heights.width = g.heights.height = 60;
  const std::size_t n = 60 * 60;
  g.heights.heights.assign(n, 0.0f);
  g.state.assign(n, GroundState::Measured);
  g.provenance.assign(n, vg::Provenance::Measured);
  g.observations.assign(n, 1);
  g.source.assign(n, 0);
  g.spread.assign(n, 0.0f);
  return g;
}

// Cells whose centers lie in [x0, x1) x [y0, y1) (m) get state s.
void paint(vg::GroundLayer& g, double x0, double y0, double x1, double y1, GroundState s) {
  for (int y = 0; y < g.heights.height; ++y) {
    for (int x = 0; x < g.heights.width; ++x) {
      const double cx = (x + 0.5) * 0.2;
      const double cy = (y + 0.5) * 0.2;
      if (cx >= x0 && cx < x1 && cy >= y0 && cy < y1) {
        g.state[g.index(x, y)] = s;
      }
    }
  }
}

TEST(Guidance, NothingUnknownIsDone) {
  const vg::Guidance guidance = vg::plan_guidance(site());
  EXPECT_TRUE(guidance.done);
  EXPECT_TRUE(guidance.targets.empty());
}

TEST(Guidance, LooksAtAHoleFromTheOpenSideNotThroughAWall) {
  // A 1.2 x 1.2 m hole at x 6-7.2 m, with a wall just west of it (x 5.6-5.8)
  // running the site's height: a spot west of it can't see it.
  vg::GroundLayer g = site();
  paint(g, 6.0, 5.0, 7.2, 6.2, GroundState::Unknown);
  paint(g, 5.6, 0.0, 5.8, 12.0, GroundState::Covered);
  const vg::Guidance guidance = vg::plan_guidance(g);
  EXPECT_FALSE(guidance.done);
  ASSERT_EQ(guidance.targets.size(), 1u);
  const vg::GuidanceTarget& t = guidance.targets[0];
  EXPECT_EQ(t.cells, 36u);
  EXPECT_NEAR(t.area, 1.44, 1e-9);
  EXPECT_NEAR(t.center.x(), 6.6, 1e-9);
  ASSERT_FALSE(t.spots.empty());
  EXPECT_GE(t.seen, 0.9);
  for (const vg::LookSpot& s : t.spots) {
    EXPECT_GT(s.position.x(), 5.8) << s.position.transpose();  // East of the wall.
    EXPECT_GE(s.distance, 1.0);
    EXPECT_LE(s.distance, 3.0);
    // Facing the hole.
    const Eigen::Vector2d to = t.center - s.position;
    const double off = std::abs(std::remainder(std::atan2(to.y(), to.x()) - s.heading, 2.0 * kPi));
    EXPECT_LT(off, 0.5);
  }
}

TEST(Guidance, RanksBySizeAndVisitsNearestFirst) {
  vg::GroundLayer g = site();
  paint(g, 9.0, 9.0, 10.2, 10.2, GroundState::Unknown);  // Bigger, far from the origin.
  paint(g, 2.0, 2.0, 2.8, 2.8, GroundState::Unknown);    // Smaller, near it.
  const vg::Guidance guidance = vg::plan_guidance(g);
  ASSERT_EQ(guidance.targets.size(), 2u);
  EXPECT_EQ(guidance.targets[0].rank, 1);
  EXPECT_NEAR(guidance.targets[0].center.x(), 9.6, 1e-9);
  EXPECT_EQ(guidance.targets[0].order, 2);  // ... but visited second.
  EXPECT_EQ(guidance.targets[1].order, 1);

  vg::GuidanceConfig lenient;
  lenient.min_area = 2.0;  // Both are smaller: done.
  EXPECT_TRUE(vg::plan_guidance(g, {0, 0}, lenient).done);
}

TEST(Guidance, ABigHoleNeedsMoreThanOneSpot) {
  // 8 x 1.2 m: no spot sees it all within 3 m.
  vg::GroundLayer g = site();
  paint(g, 2.0, 5.0, 10.0, 6.2, GroundState::Unknown);
  const vg::Guidance guidance = vg::plan_guidance(g);
  ASSERT_EQ(guidance.targets.size(), 1u);
  EXPECT_GE(guidance.targets[0].spots.size(), 2u);
  EXPECT_GT(guidance.targets[0].seen, guidance.targets[0].spots[0].sees);
}

}  // namespace
