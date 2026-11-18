#include "vg_core/boundary.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace {

using vg::BoundaryEdge;
using vg::GroundState;

// A walk round the square [5, 15] x [5, 15], counter-clockwise from (5, 5),
// carrying on 3 m past the start; every 0.1 m.
std::vector<Eigen::Vector2d> square_walk(const Eigen::Vector2d& drift = Eigen::Vector2d::Zero()) {
  const std::vector<Eigen::Vector2d> corners = {{5, 5}, {15, 5}, {15, 15}, {5, 15}, {5, 5}, {8, 5}};
  std::vector<Eigen::Vector2d> path;
  double walked = 0.0;
  for (std::size_t c = 1; c < corners.size(); ++c) {
    const Eigen::Vector2d a = corners[c - 1];
    const Eigen::Vector2d d = corners[c] - a;
    const int steps = static_cast<int>(std::round(d.norm() / 0.1));
    for (int s = 0; s < steps; ++s) {
      // Drift grows with the distance walked, reaching drift after 40 m.
      path.push_back(a + d * (static_cast<double>(s) / steps) + drift * (walked / 40.0));
      walked += 0.1;
    }
  }
  return path;
}

// A 20 x 20 m yard of measured ground, 0.2 m cells: a fence (covered) at
// x = 3 and at y = 17; the ground ends at x = 16.5; open to the south.
vg::GroundLayer yard() {
  vg::GroundLayer g;
  g.heights.cell_size = 0.2;
  g.heights.width = g.heights.height = 100;
  const std::size_t n = 100 * 100;
  g.heights.heights.assign(n, 0.0f);
  g.state.assign(n, GroundState::Measured);
  g.provenance.assign(n, vg::Provenance::Measured);
  g.observations.assign(n, 1);
  g.source.assign(n, 0);
  g.spread.assign(n, 0.0f);
  for (int y = 0; y < 100; ++y) {
    for (int x = 0; x < 100; ++x) {
      const double cx = (x + 0.5) * 0.2;
      const double cy = (y + 0.5) * 0.2;
      const std::size_t i = g.index(x, y);
      if (cx > 16.5) {
        g.state[i] = GroundState::Outside;
        g.heights.heights[i] = std::numeric_limits<float>::quiet_NaN();
      } else if (std::abs(cx - 3.0) < 0.15 || std::abs(cy - 17.0) < 0.15) {
        g.state[i] = GroundState::Covered;
      }
    }
  }
  return g;
}

// The boundary vertex nearest to p.
const vg::BoundaryVertex& nearest(const vg::Boundary& b, const Eigen::Vector2d& p) {
  const vg::BoundaryVertex* best = &b.vertices.front();
  for (const vg::BoundaryVertex& v : b.vertices) {
    if ((v.position - p).norm() < (best->position - p).norm()) {
      best = &v;
    }
  }
  return *best;
}

TEST(Boundary, ClosesTheLoopAndMeasuresDrift) {
  const vg::LoopClosure loop = vg::close_loop(square_walk({0.3, 0.0}));
  ASSERT_TRUE(loop.closed);
  EXPECT_NEAR(loop.length, 40.0, 0.5);
  EXPECT_NEAR(loop.gap, 0.3, 0.05);  // The drift, back at the start.
  EXPECT_NEAR(loop.drift_per_100m, 0.75, 0.15);
}

TEST(Boundary, AnOpenWalkDoesntClose) {
  std::vector<Eigen::Vector2d> path;
  for (int i = 0; i < 200; ++i) {
    path.emplace_back(0.1 * i, 0.0);
  }
  EXPECT_FALSE(vg::close_loop(path).closed);
  EXPECT_TRUE(vg::estimate_boundary(path, yard()).vertices.empty());
}

TEST(Boundary, PushesOutToTheEdgeFollowed) {
  const vg::Boundary b = vg::estimate_boundary(square_walk(), yard());
  ASSERT_TRUE(b.loop.closed);
  ASSERT_GT(b.vertices.size(), 60u);

  const vg::BoundaryVertex& west = nearest(b, {3.0, 10.0});  // The fence.
  EXPECT_EQ(west.edge, BoundaryEdge::Structure);
  EXPECT_NEAR(west.position.x(), 3.0, 0.2);
  const vg::BoundaryVertex& north = nearest(b, {10.0, 17.0});  // The other fence.
  EXPECT_EQ(north.edge, BoundaryEdge::Structure);
  EXPECT_NEAR(north.position.y(), 17.0, 0.2);
  const vg::BoundaryVertex& east = nearest(b, {16.5, 10.0});  // The ground's end.
  EXPECT_EQ(east.edge, BoundaryEdge::GroundEnd);
  EXPECT_NEAR(east.position.x(), 16.4, 0.2);
  const vg::BoundaryVertex& south = nearest(b, {10.0, 4.0});  // Open: assumed 1 m out.
  EXPECT_EQ(south.edge, BoundaryEdge::Assumed);
  EXPECT_NEAR(south.position.y(), 4.0, 0.05);

  // About 13.4 x 13 m.
  EXPECT_NEAR(b.area, 13.4 * 13.0, 8.0);
}

TEST(Boundary, ComparesBoundaries) {
  const vg::Boundary a = vg::estimate_boundary(square_walk(), yard());
  const vg::BoundaryAgreement same = vg::compare_boundaries(a, a);
  EXPECT_NEAR(same.median_distance, 0.0, 1e-9);
  EXPECT_NEAR(same.overlap, 1.0, 1e-9);
  vg::Boundary shifted = a;
  for (vg::BoundaryVertex& v : shifted.vertices) {
    v.position.x() += 0.5;
  }
  const vg::BoundaryAgreement apart = vg::compare_boundaries(a, shifted);
  EXPECT_GT(apart.median_distance, 0.2);
  EXPECT_LT(apart.median_distance, 0.55);
  EXPECT_LT(apart.overlap, 0.95);
}

TEST(Boundary, CellsOutsideAreOutside) {
  vg::GroundLayer g = yard();
  const vg::Boundary b = vg::estimate_boundary(square_walk(), g);
  vg::apply_boundary(g, b);
  EXPECT_EQ(g.state[g.index(50, 50)], GroundState::Measured);  // (10, 10): inside.
  EXPECT_EQ(g.state[g.index(5, 50)], GroundState::Outside);    // (1, 10): beyond the fence.
  EXPECT_TRUE(std::isnan(g.heights.heights[g.index(5, 50)]));
  EXPECT_EQ(g.state[g.index(50, 5)], GroundState::Outside);  // (10, 1): south of it.
}

TEST(Boundary, AnEditRedrawsAStretch) {
  // The south side was assumed (open ground); the user draws it 0.5 m in.
  vg::Boundary b = vg::estimate_boundary(square_walk(), yard());
  vg::GroundEdit e;
  e.id = "gate";
  e.kind = vg::GroundEdit::Kind::Boundary;
  e.polygon = {{6.0, 4.5}, {10.0, 4.5}, {14.0, 4.5}};
  vg::GroundEdit area;  // Area edits aren't the boundary's.
  area.kind = vg::GroundEdit::Kind::NotGround;
  area.polygon = {{0, 0}, {1, 0}, {1, 1}};
  EXPECT_EQ(vg::edit_boundary(b, {area, e}), 1u);

  const vg::BoundaryVertex& mid = nearest(b, {10.0, 4.0});
  EXPECT_EQ(mid.edge, BoundaryEdge::Edited);
  EXPECT_DOUBLE_EQ(mid.position.y(), 4.5);
  for (const vg::BoundaryVertex& v : b.vertices) {  // The old stretch is gone...
    if (v.position.x() > 6.5 && v.position.x() < 13.5) {
      EXPECT_FALSE(std::abs(v.position.y() - 4.0) < 0.05) << v.position.transpose();
    }
  }
  EXPECT_EQ(nearest(b, {3.0, 10.0}).edge, BoundaryEdge::Structure);  // ... the rest stays.
  EXPECT_NEAR(b.area, vg::estimate_boundary(square_walk(), yard()).area - 0.5 * 8.0, 2.0);

  // Placed by where it is: the same edit on the boundary made again agrees.
  vg::Boundary again = vg::estimate_boundary(square_walk(), yard());
  vg::edit_boundary(again, {e});
  EXPECT_NEAR(vg::compare_boundaries(b, again).median_distance, 0.0, 1e-9);
}

}  // namespace
