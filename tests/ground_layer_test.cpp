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

vg::GroundLayerConfig latest_wins() {
  vg::GroundLayerConfig config;
  config.latest_wins = true;
  config.max_fill_area = 0.5;  // So the 1 m^2 hole at cell 4 isn't filled.
  return config;
}

TEST(GroundLayer, CombinesWalksTheLatestMeasuredWins) {
  const vg::GroundLayer layer = vg::estimate_ground(two_walks(), {}, latest_wins());
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
  EXPECT_FLOAT_EQ(layer.spread[1], 1.5f);  // a, 1.5 m below.
  EXPECT_FLOAT_EQ(layer.spread[0], 0.0f);  // One walk.

  // Under an object (not known what: covered) beats a small hole; the ground
  // runs on from the measured ground beside it, not the walk's own patch.
  EXPECT_EQ(layer.state[2], GroundState::Covered);
  EXPECT_FLOAT_EQ(h[2], 2.5f);
  EXPECT_EQ(layer.provenance[2], Provenance::Inferred);
  EXPECT_EQ(layer.observations[2], 0);
  EXPECT_TRUE(std::isnan(layer.spread[2]));

  EXPECT_EQ(layer.state[3], GroundState::Measured);  // b saw what a missed.
  EXPECT_FLOAT_EQ(h[3], 2.5f);

  EXPECT_EQ(layer.state[4], GroundState::Unknown);  // A hole, too large to fill.
  EXPECT_TRUE(std::isnan(h[4]));
  EXPECT_EQ(layer.provenance[4], Provenance::None);

  EXPECT_EQ(layer.state[5], GroundState::Outside);  // Nobody knew it, at the edge.
}

// Walks over the same two cells (heights per walk), with weights.
std::vector<vg::WalkGround> same_place(const std::vector<std::pair<float, float>>& height_weight) {
  std::vector<vg::WalkGround> walks;
  for (std::size_t k = 0; k < height_weight.size(); ++k) {
    const auto [h, w] = height_weight[k];
    walks.push_back(
        walk("w" + std::to_string(k), {GroundCell::Measured, GroundCell::Measured}, {h, h}));
    walks.back().weight = {w, w};
  }
  return walks;
}

TEST(GroundLayer, OneBadWalkDoesntMakeADip) {
  // Two walks agree; the latest reads 30 cm low (a bad reading).
  const auto walks = same_place({{1.00f, 1.0f}, {1.02f, 1.0f}, {0.70f, 1.0f}});
  const vg::GroundLayer robust = vg::estimate_ground(walks, {});
  EXPECT_NEAR(robust.heights.heights[0], 1.01f, 1e-5);  // The two that agree, averaged.
  EXPECT_NEAR(robust.spread[0], 0.31f, 1e-5);           // The bad walk shows as spread.
  EXPECT_EQ(robust.observations[0], 3);
  const vg::GroundLayer latest = vg::estimate_ground(walks, {}, latest_wins());
  EXPECT_FLOAT_EQ(latest.heights.heights[0], 0.70f);  // The dip.
}

TEST(GroundLayer, TheBetterSeenWalkCountsMore) {
  // Two walks disagree: the one that saw the cell ten times as much wins.
  const vg::GroundLayer layer = vg::estimate_ground(same_place({{1.0f, 10.0f}, {1.5f, 1.0f}}), {});
  EXPECT_FLOAT_EQ(layer.heights.heights[0], 1.0f);
  EXPECT_EQ(layer.source[0], 0);
  EXPECT_FLOAT_EQ(layer.spread[0], 0.5f);
}

TEST(GroundLayer, WalksThatAgreeAreAveragedByWeight) {
  const vg::GroundLayer layer = vg::estimate_ground(same_place({{1.00f, 1.0f}, {1.06f, 3.0f}}), {});
  EXPECT_NEAR(layer.heights.heights[0], (1.00f + 3 * 1.06f) / 4, 1e-5);
  EXPECT_EQ(layer.source[0], 1);  // The heavier one.
  vg::GroundLayerConfig strict;
  strict.agree_within = 0.05;  // Now they don't agree: the median walk alone.
  EXPECT_FLOAT_EQ(vg::estimate_ground(same_place({{1.00f, 1.0f}, {1.06f, 3.0f}}), {}, strict)
                      .heights.heights[0],
                  1.06f);
}

// A walk measuring an n x n grid of 0.2 m cells, ground at height(x, y).
template <typename F>
vg::WalkGround grid_walk(const std::string& id, int n, F height) {
  vg::WalkGround w;
  w.id = id;
  w.heights.cell_size = 0.2;
  w.heights.width = n;
  w.heights.height = n;
  for (int y = 0; y < n; ++y) {
    for (int x = 0; x < n; ++x) {
      w.heights.heights.push_back(height((x + 0.5) * 0.2, (y + 0.5) * 0.2));
      w.state.push_back(static_cast<std::uint8_t>(GroundCell::Measured));
      w.weight.push_back(1.0f);
    }
  }
  return w;
}

float slope_ground(double x, double y) { return static_cast<float>(0.1 * x - 0.03 * y); }

TEST(GroundLayer, AlignsAWalkThatSitsHighAndTilted) {
  // The second walk reads the same 6 x 6 m slope 15 cm high at its middle and
  // tilted 2 cm per meter along x, with 40 cells of plants 0.5 m tall.
  const vg::WalkGround a = grid_walk("a", 30, slope_ground);
  const vg::WalkGround b = grid_walk("b", 30, [](double x, double y) {
    const bool plant = x > 1.0 && x < 1.8 && y > 1.0 && y < 3.0;
    return slope_ground(x, y) + static_cast<float>(0.15 + 0.02 * (x - 3.0)) + (plant ? 0.5f : 0.0f);
  });
  const vg::GroundLayer layer = vg::estimate_ground({a, b}, {});

  const vg::WalkAlignment& fit = layer.alignment[1];
  EXPECT_EQ(fit.cells, 900u);
  EXPECT_TRUE(fit.tilted);
  EXPECT_NEAR(fit.correction_at({3.0, 3.0}), 0.15, 0.005);
  EXPECT_NEAR(fit.slope.x(), 0.02, 0.002);
  EXPECT_NEAR(fit.slope.y(), 0.0, 0.002);
  EXPECT_GT(fit.before, 0.1);
  EXPECT_LT(fit.after, 0.005);
  EXPECT_DOUBLE_EQ(layer.alignment[0].offset, 0.0);  // The first walk stays.

  // Aligned, the walks agree (but at the plants): the layer is the first
  // walk's ground.
  for (int y = 0; y < 30; ++y) {
    for (int x = 0; x < 30; ++x) {
      const std::size_t i = layer.index(x, y);
      EXPECT_NEAR(layer.heights.heights[i], slope_ground((x + 0.5) * 0.2, (y + 0.5) * 0.2), 0.01)
          << x << "," << y;
    }
  }

  // Without alignment, the walks don't agree: the median walk alone, 15 cm off at the middle.
  vg::GroundLayerConfig off;
  off.align = false;
  const vg::GroundLayer unaligned = vg::estimate_ground({a, b}, {}, off);
  EXPECT_GT(unaligned.spread[layer.index(15, 15)], 0.1f);
}

TEST(GroundLayer, LittleOverlapOnlyMovesAWalkUpOrDown) {
  // 100 shared cells: below align_min_cells, so no tilt.
  const vg::WalkGround a = grid_walk("a", 10, slope_ground);
  const vg::WalkGround b = grid_walk("b", 10, [](double x, double y) {
    return slope_ground(x, y) + static_cast<float>(0.1 + 0.02 * x);
  });
  const vg::GroundLayer layer = vg::estimate_ground({a, b}, {});
  EXPECT_FALSE(layer.alignment[1].tilted);
  EXPECT_NEAR(layer.alignment[1].offset, 0.12, 0.02);  // The median shift.
}

TEST(GroundLayer, RefusesASteepTilt) {
  const vg::WalkGround a = grid_walk("a", 30, slope_ground);
  const vg::WalkGround b = grid_walk("b", 30, [](double x, double y) {
    return slope_ground(x, y) + static_cast<float>(0.2 * (x - 3.0));  // 11 degrees.
  });
  const vg::GroundLayer layer = vg::estimate_ground({a, b}, {});
  EXPECT_FALSE(layer.alignment[1].tilted);
}

TEST(GroundLayer, ALaterWalkLeavesEarlierAlignmentsAlone) {
  const vg::WalkGround a = grid_walk("a", 30, slope_ground);
  const vg::WalkGround b = grid_walk("b", 30, [](double x, double y) {
    return slope_ground(x, y) + static_cast<float>(0.1 + 0.01 * x);
  });
  const vg::WalkGround c = grid_walk("c", 30, [](double x, double y) {
    return slope_ground(x, y) - static_cast<float>(0.2 + 0.01 * y);
  });
  const vg::GroundLayer two = vg::estimate_ground({a, b}, {});
  const vg::GroundLayer three = vg::estimate_ground({a, b, c}, {});
  EXPECT_DOUBLE_EQ(three.alignment[1].offset, two.alignment[1].offset);
  EXPECT_EQ(three.alignment[1].slope, two.alignment[1].slope);
  EXPECT_NEAR(three.alignment[2].correction_at({3.0, 3.0}), -0.23, 0.005);
  EXPECT_NEAR(three.heights.heights[three.index(15, 15)], slope_ground(3.1, 3.1), 0.01);
}

TEST(GroundLayer, AlignsToTheWalkThatSetsTheFrameNotTheOldest) {
  // An older recording added after the site was set up: it's moved to meet
  // the frame's walk, not the other way round.
  const vg::WalkGround older =
      grid_walk("older", 30, [](double x, double y) { return slope_ground(x, y) + 0.12f; });
  vg::WalkGround frame = grid_walk("frame", 30, slope_ground);
  frame.sets_frame = true;
  const vg::GroundLayer layer = vg::estimate_ground({older, frame}, {});
  EXPECT_NEAR(layer.alignment[0].offset, 0.12, 0.005);
  EXPECT_DOUBLE_EQ(layer.alignment[1].offset, 0.0);
  EXPECT_NEAR(layer.heights.heights[layer.index(15, 15)], slope_ground(3.1, 3.1), 0.005);
}

// The 6 x 6 m sloping walk, with what it saw on the ground in places: cells
// of each rectangle (in 0.2 m cells, inclusive) get the state and object.
struct Patch {
  int x0, y0, x1, y1;
  GroundCell state;
  int object;  // -1: none.
};
vg::WalkGround yard(const std::vector<vg::WalkObject>& objects, const std::vector<Patch>& patches) {
  vg::WalkGround w = grid_walk("yard", 30, slope_ground);
  w.object.assign(w.state.size(), -1);
  w.objects = objects;
  for (const Patch& p : patches) {
    for (int y = p.y0; y <= p.y1; ++y) {
      for (int x = p.x0; x <= p.x1; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * 30 + static_cast<std::size_t>(x);
        w.state[i] = static_cast<std::uint8_t>(p.state);
        w.object[i] = p.object;
        w.weight[i] = 0.0f;
        w.heights.heights[i] = p.state == GroundCell::LargeHole ? kNaN : 9.0f;  // Patches: ignored.
      }
    }
  }
  return w;
}

vg::WalkObject object(vg::SemanticClass label, bool grounded, double top) {
  vg::WalkObject o;
  o.label = label;
  o.grounded = grounded;
  o.top_above_ground = top;
  return o;
}

TEST(GroundLayer, ClassifiesWhatStandsOnTheGroundAndTheHoles) {
  using vg::SemanticClass;
  using C = GroundCell;
  const std::vector<vg::WalkObject> objects = {
      object(SemanticClass::Structure, true, 2.0),  // 0: a shed
      object(SemanticClass::Plant, true, 0.3),      // 1: low plants
      object(SemanticClass::Unknown, true, 0.8),    // 2: something tall
      object(SemanticClass::Tree, false, 3.0),      // 3: a canopy (overhang)
      object(SemanticClass::Ground, true, 0.4),     // 4: a stretch of ground not joined
  };
  const vg::WalkGround w =
      yard(objects, {
                        {2, 2, 6, 6, C::UnderObject, 0},      // shed, 1 x 1 m
                        {10, 2, 14, 6, C::UnderObject, 1},    // plants
                        {18, 2, 20, 4, C::UnderObject, 2},    // something tall
                        {2, 10, 7, 15, C::UnderObject, 3},    // canopy over unseen ground, 1.44 m^2
                        {10, 10, 12, 12, C::UnderObject, 4},  // ground piece, 0.36 m^2
                        {15, 15, 22, 22, C::LargeHole, -1},   // unseen, 2.56 m^2
                        {25, 10, 26, 11, C::SmallHole, -1},   // unseen, 0.16 m^2
                        {0, 25, 0, 29, C::None, -1},          // never seen, at the edge
                        {24, 24, 26, 26, C::None, -1},        // never seen, enclosed, 0.36 m^2
                    });
  const vg::GroundLayer layer = vg::estimate_ground({w}, {});
  const auto state = [&](int x, int y) { return layer.state[layer.index(x, y)]; };

  EXPECT_EQ(state(4, 4), GroundState::Covered);
  EXPECT_EQ(state(12, 4), GroundState::Filled);
  EXPECT_EQ(state(19, 3), GroundState::Covered);
  EXPECT_EQ(state(4, 12), GroundState::Unknown);  // The canopy doesn't stand on it.
  EXPECT_EQ(state(11, 11), GroundState::Filled);  // Small, and ground anyway.
  EXPECT_EQ(state(18, 18), GroundState::Unknown);
  EXPECT_EQ(state(25, 10), GroundState::Filled);
  EXPECT_EQ(state(0, 27), GroundState::Outside);
  EXPECT_EQ(state(25, 25), GroundState::Filled);  // Enclosed by what the walk saw.
  EXPECT_EQ(state(1, 1), GroundState::Measured);

  // Under the shed the slope runs on, from the measured ground around it.
  const float shed = layer.heights.heights[layer.index(4, 4)];
  EXPECT_NEAR(shed, slope_ground(0.9, 0.9), 0.01);
  EXPECT_EQ(layer.provenance[layer.index(4, 4)], Provenance::Inferred);
  EXPECT_TRUE(std::isnan(layer.heights.heights[layer.index(18, 18)]));  // Unknown: no height.

  // Lower the bar for covering: the plants (0.3 m) cover too.
  vg::GroundLayerConfig low;
  low.cover_min_height = 0.25;
  EXPECT_EQ(vg::estimate_ground({w}, {}, low).state[layer.index(12, 4)], GroundState::Covered);
}

TEST(GroundLayer, EditsWinAndTheGroundRunsOnBeneath) {
  const std::vector<GroundEdit> edits = {
      edit(GroundEdit::Kind::NotGround, 0.0, 1.0, "deck"),  // Over measured cell 0.
      edit(GroundEdit::Kind::Ground, 4.0, 5.0, "lawn"),     // Over the unknown cell 4.
      edit(GroundEdit::Kind::Ground, 1.0, 2.0, "seen"),     // Over measured cell 1: no change.
  };
  const vg::GroundLayer layer = vg::estimate_ground(two_walks(), edits);
  const vg::GroundLayer unedited = vg::estimate_ground(two_walks(), {});
  const auto& h = layer.heights.heights;

  EXPECT_EQ(layer.state[0], GroundState::Covered);
  EXPECT_EQ(layer.provenance[0], Provenance::Edited);
  EXPECT_EQ(layer.source[0], -1);
  EXPECT_TRUE(std::isnan(layer.spread[0]));
  EXPECT_NEAR(h[0], h[1], 1e-4);  // Filled from the ground beside it, not the deck's top.

  EXPECT_EQ(layer.state[4], GroundState::Filled);
  EXPECT_EQ(layer.provenance[4], Provenance::Edited);
  EXPECT_NEAR(h[4], 2.5f, 1e-4);

  EXPECT_EQ(layer.state[1], GroundState::Measured);
  EXPECT_EQ(layer.provenance[1], Provenance::Measured);
  EXPECT_FLOAT_EQ(h[1], unedited.heights.heights[1]);
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
