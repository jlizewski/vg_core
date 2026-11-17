#include "vg_core/ground_layer.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "vg_core/ground_segmentation.hpp"

namespace {

namespace fs = std::filesystem;
using vg::GroundCell;
using vg::GroundEdit;
using vg::GroundState;
using vg::Provenance;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// A walk whose ground is one row of 1 m cells from x = 0.
vg::WalkGround walk(const std::string& id, std::vector<GroundCell> states,
                    std::vector<float> heights, Eigen::Vector3d shift = Eigen::Vector3d::Zero()) {
  vg::WalkGround w;
  w.id = id;
  w.heights.origin = {0.0, 0.0};
  w.heights.cell_size = 1.0;
  w.heights.width = static_cast<int>(states.size());
  w.heights.height = 1;
  w.heights.heights = std::move(heights);
  for (const GroundCell s : states) {
    w.state.push_back(static_cast<std::uint8_t>(s));
    w.weight.push_back(s == GroundCell::Measured ? 1.0f : 0.0f);
  }
  w.site_from_map.translation = shift;
  return w;
}

// Walk a, and walk b (later) one cell to the right and 0.5 m up:
//
//   site x:   0   1   2   3   4   5
//   a:        M   M   U   L   L   -      (heights 1, patched 1.2 under the object)
//   b:            M   S   M              (heights 2, so 2.5 in the site)
std::vector<vg::WalkGround> two_walks() {
  using C = GroundCell;
  return {walk("a", {C::Measured, C::Measured, C::UnderObject, C::LargeHole, C::LargeHole, C::None},
               {1.0f, 1.0f, 1.2f, kNaN, kNaN, kNaN}),
          walk("b", {C::Measured, C::SmallHole, C::Measured}, {2.0f, 2.0f, 2.0f}, {1.0, 0.0, 0.5})};
}

GroundEdit edit(GroundEdit::Kind kind, double x0, double x1, const std::string& id = "e1") {
  GroundEdit e;
  e.id = id;
  e.time = "2026-11-16T12:00:00Z";
  e.kind = kind;
  e.polygon = {{x0, -1.0}, {x1, -1.0}, {x1, 2.0}, {x0, 2.0}};
  return e;
}

TEST(GroundLayer, CombinesWalksTheLatestMeasuredWins) {
  const vg::GroundLayer layer = vg::estimate_ground(two_walks(), {});
  ASSERT_EQ(layer.heights.width, 6);
  ASSERT_EQ(layer.heights.height, 1);
  const auto& h = layer.heights.heights;

  EXPECT_EQ(layer.state[0], GroundState::Measured);  // Only a saw it.
  EXPECT_FLOAT_EQ(h[0], 1.0f);
  EXPECT_EQ(layer.source[0], 0);
  EXPECT_EQ(layer.observations[0], 1);

  EXPECT_EQ(layer.state[1], GroundState::Measured);  // Both: b is later.
  EXPECT_FLOAT_EQ(h[1], 2.5f);
  EXPECT_EQ(layer.source[1], 1);
  EXPECT_EQ(layer.observations[1], 2);
  EXPECT_EQ(layer.provenance[1], Provenance::Measured);

  EXPECT_EQ(layer.state[2], GroundState::Covered);  // Under an object beats a small hole.
  EXPECT_FLOAT_EQ(h[2], 1.2f);
  EXPECT_EQ(layer.provenance[2], Provenance::Inferred);
  EXPECT_EQ(layer.observations[2], 0);

  EXPECT_EQ(layer.state[3], GroundState::Measured);  // b saw what a missed.
  EXPECT_FLOAT_EQ(h[3], 2.5f);

  EXPECT_EQ(layer.state[4], GroundState::Unknown);  // Nobody saw it.
  EXPECT_TRUE(std::isnan(h[4]));
  EXPECT_EQ(layer.provenance[4], Provenance::None);

  EXPECT_EQ(layer.state[5], GroundState::Outside);
}

TEST(GroundLayer, EditsWinAndTheGroundRunsOnBeneath) {
  const std::vector<GroundEdit> edits = {
      edit(GroundEdit::Kind::NotGround, 0.0, 1.0, "deck"),  // Over measured cell 0.
      edit(GroundEdit::Kind::Ground, 4.0, 5.0, "lawn"),     // Over the unknown cell 4.
      edit(GroundEdit::Kind::Ground, 1.0, 2.0, "seen"),     // Over measured cell 1: no change.
  };
  const vg::GroundLayer layer = vg::estimate_ground(two_walks(), edits);
  const auto& h = layer.heights.heights;

  EXPECT_EQ(layer.state[0], GroundState::Covered);
  EXPECT_EQ(layer.provenance[0], Provenance::Edited);
  EXPECT_EQ(layer.source[0], -1);
  EXPECT_NEAR(h[0], 2.5f, 1e-4);  // Filled from the ground beside it, not the deck's top.

  EXPECT_EQ(layer.state[4], GroundState::Filled);
  EXPECT_EQ(layer.provenance[4], Provenance::Edited);
  EXPECT_NEAR(h[4], 2.5f, 1e-4);

  EXPECT_EQ(layer.state[1], GroundState::Measured);
  EXPECT_EQ(layer.provenance[1], Provenance::Measured);
  EXPECT_FLOAT_EQ(h[1], 2.5f);
}

TEST(GroundLayer, SameInputsSameLayer) {
  const auto walks = two_walks();
  const std::vector<GroundEdit> edits = {edit(GroundEdit::Kind::Ground, 4.0, 5.0)};
  const vg::GroundLayer a = vg::estimate_ground(walks, edits);
  const vg::GroundLayer b = vg::estimate_ground(walks, edits);
  ASSERT_EQ(a.heights.heights.size(), b.heights.heights.size());
  for (std::size_t i = 0; i < a.heights.heights.size(); ++i) {
    const float x = a.heights.heights[i];
    const float y = b.heights.heights[i];
    EXPECT_TRUE((std::isnan(x) && std::isnan(y)) || x == y) << i;
  }
  EXPECT_EQ(a.state, b.state);
  EXPECT_EQ(a.provenance, b.provenance);
  EXPECT_EQ(a.source, b.source);
}

TEST(GroundLayer, NoWalksNoLayer) {
  const vg::GroundLayer layer = vg::estimate_ground({}, {});
  EXPECT_EQ(layer.heights.width, 0);
  EXPECT_TRUE(layer.state.empty());
}

TEST(GroundLayer, InsidePolygon) {
  const std::vector<Eigen::Vector2d> triangle = {{0, 0}, {4, 0}, {0, 4}};
  EXPECT_TRUE(vg::inside_polygon(triangle, {1, 1}));
  EXPECT_FALSE(vg::inside_polygon(triangle, {3, 3}));
  EXPECT_FALSE(vg::inside_polygon(triangle, {-1, 1}));
}

TEST(GroundEdits, RoundTripAndRejectNonsense) {
  const fs::path dir =
      fs::temp_directory_path() /
      ("vg_ground_edits_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(dir);
  EXPECT_TRUE(vg::load_ground_edits(dir / "edits.csv").empty());  // No file: no edits.

  GroundEdit deck = edit(GroundEdit::Kind::NotGround, -1.25, 2.5, "deck");
  deck.note = "the deck, by the back door";
  const std::vector<GroundEdit> edits = {deck, edit(GroundEdit::Kind::Ground, 0.0, 1.0, "lawn")};
  vg::save_ground_edits(edits, dir / "edits.csv");
  const std::vector<GroundEdit> back = vg::load_ground_edits(dir / "edits.csv");
  ASSERT_EQ(back.size(), 2u);
  EXPECT_EQ(back[0].id, "deck");
  EXPECT_EQ(back[0].time, deck.time);
  EXPECT_EQ(back[0].kind, GroundEdit::Kind::NotGround);
  EXPECT_EQ(back[0].note, deck.note);  // Commas and all.
  ASSERT_EQ(back[0].polygon.size(), 4u);
  EXPECT_DOUBLE_EQ(back[0].polygon[0].x(), -1.25);
  EXPECT_EQ(back[1].kind, GroundEdit::Kind::Ground);

  {
    std::ofstream bad(dir / "bad.csv");
    bad << "id,time,kind,polygon,note\ne,2026-11-16,bedrock,0 0;1 0;1 1,\n";
  }
  EXPECT_THROW(vg::load_ground_edits(dir / "bad.csv"), std::runtime_error);
  {
    std::ofstream two(dir / "two.csv");
    two << "id,time,kind,polygon,note\ne,2026-11-16,ground,0 0;1 0,\n";
  }
  EXPECT_THROW(vg::load_ground_edits(dir / "two.csv"), std::runtime_error);
  fs::remove_all(dir);
}

}  // namespace
