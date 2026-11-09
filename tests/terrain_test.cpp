#include "vg_core/terrain.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace {

constexpr double kCell = 0.1524;

// A ground map over [0, size)^2 with heights from height(x, y) at cell
// centers; NaN where seen(x, y) is false.
vg::HeightMap make_ground(double size, const std::function<double(double, double)>& height,
                          const std::function<bool(double, double)>& seen = nullptr) {
  vg::HeightMap map;
  map.cell_size = kCell;
  map.width = static_cast<int>(size / kCell);
  map.height = map.width;
  map.heights.assign(static_cast<std::size_t>(map.width * map.height), NAN);
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      const double cx = (x + 0.5) * kCell;
      const double cy = (y + 0.5) * kCell;
      if (!seen || seen(cx, cy)) {
        map.heights[static_cast<std::size_t>(y * map.width + x)] =
            static_cast<float>(height(cx, cy));
      }
    }
  }
  return map;
}

// A walk back and forth across [1, size - 1]^2, with the phone held phone
// meters over height(x, y), wobbling by up to +-wobble.
std::vector<Eigen::Vector3d> make_walk(double size, double phone,
                                       const std::function<double(double, double)>& height,
                                       double wobble = 0.0) {
  std::vector<Eigen::Vector3d> walk;
  int i = 0;
  for (double y = 1.0; y <= size - 1.0; y += 1.0) {
    for (double x = 1.0; x <= size - 1.0; x += 0.1, ++i) {
      const double z = height(x, y) + phone + wobble * std::sin(i * 1.7);
      walk.emplace_back(x, y, z);
    }
  }
  return walk;
}

double terrain_at(const vg::Terrain& t, double x, double y) {
  return t.model.at({x, y});
}

TEST(TerrainTest, FitsFlatSeenGroundAndSolvesThePhoneHeight) {
  const auto flat = [](double, double) { return -1.0; };
  const vg::Terrain t = fit_terrain(make_ground(8.0, flat), {}, make_walk(8.0, 1.5, flat, 0.15));
  EXPECT_TRUE(t.phone_height_solved);
  EXPECT_NEAR(t.phone_height, 1.5, 0.03);
  EXPECT_NEAR(terrain_at(t, 4.0, 4.0), -1.0, 0.02);
  EXPECT_NEAR(t.ground_residual, 0.0, 0.01);
  EXPECT_EQ(t.ground_cells_discounted, 0u);
}

TEST(TerrainTest, FollowsTheWalkWhereTheGroundIsHidden) {
  // A slope with a 3 x 3 m overgrown patch where no ground was seen.
  const auto slope = [](double x, double y) { return 0.1 * x + 0.05 * y; };
  const auto seen = [](double x, double y) { return !(x > 3 && x < 6 && y > 3 && y < 6); };
  const vg::Terrain t =
      fit_terrain(make_ground(9.0, slope, seen), {}, make_walk(9.0, 1.5, slope, 0.15));
  for (const double x : {3.5, 4.5, 5.5}) {
    EXPECT_NEAR(terrain_at(t, x, 4.5), slope(x, 4.5), 0.05) << x;
  }
}

TEST(TerrainTest, DiscountsFoliageTakenForGroundWhereTheWalkDisagrees) {
  // Flat ground, but in a 3 x 3 m patch what's seen is foliage 0.4 m up.
  const auto flat = [](double, double) { return 0.0; };
  const auto foliage = [](double x, double y) {
    return x > 3 && x < 6 && y > 3 && y < 6 ? 0.4 : 0.0;
  };
  const vg::Terrain t =
      fit_terrain(make_ground(9.0, foliage), {}, make_walk(9.0, 1.5, flat, 0.15));
  EXPECT_NEAR(terrain_at(t, 4.5, 4.5), 0.0, 0.1);
  EXPECT_NEAR(t.phone_height, 1.5, 0.05);
  EXPECT_GT(t.ground_cells_discounted_above, 200u);

  // Without the walk, only smoothing argues against the foliage: it's taken
  // for a rise in the ground.
  vg::TerrainConfig no_path;
  no_path.use_path = false;
  const vg::Terrain blind = fit_terrain(make_ground(9.0, foliage), {}, {}, no_path);
  EXPECT_GT(terrain_at(blind, 4.5, 4.5), terrain_at(t, 4.5, 4.5) + 0.08);
}

TEST(TerrainTest, IgnoresPatchedGround) {
  const auto flat = [](double, double) { return 0.0; };
  vg::HeightMap ground = make_ground(6.0, flat);
  std::vector<std::uint8_t> patched(ground.heights.size(), 0);
  // A patched (interpolated) region set a meter up must not pull the fit.
  for (int y = 10; y < 25; ++y) {
    for (int x = 10; x < 25; ++x) {
      const auto i = static_cast<std::size_t>(y * ground.width + x);
      ground.heights[i] = 1.0f;
      patched[i] = 1;
    }
  }
  vg::TerrainConfig config;
  config.use_path = false;
  const vg::Terrain t = fit_terrain(ground, patched, {}, config);
  EXPECT_NEAR(terrain_at(t, 2.6, 2.6), 0.0, 0.02);
  EXPECT_EQ(t.ground_cells, ground.heights.size() - 15 * 15);
}

TEST(TerrainTest, UsesTheGivenOrDefaultPhoneHeightWithoutSeenGround) {
  const auto slope = [](double x, double) { return 0.2 * x; };
  const auto walk = make_walk(8.0, 1.4, slope);
  vg::TerrainConfig config;
  config.default_phone_height = 1.4;
  const vg::Terrain t = fit_terrain(vg::HeightMap{}, {}, walk, config);
  EXPECT_FALSE(t.phone_height_solved);
  EXPECT_DOUBLE_EQ(t.phone_height, 1.4);
  EXPECT_NEAR(terrain_at(t, 4.0, 4.0), slope(4.0, 4.0), 0.03);

  config.phone_height = 1.0;
  const vg::Terrain given = fit_terrain(make_ground(8.0, slope), {}, walk, config);
  EXPECT_FALSE(given.phone_height_solved);
  EXPECT_DOUBLE_EQ(given.phone_height, 1.0);
}

TEST(TerrainTest, MapsOnlyWithinReachOfEvidence) {
  // A single straight walk: the terrain reaches out reach meters either side.
  std::vector<Eigen::Vector3d> walk;
  for (double x = 0.0; x <= 6.0; x += 0.1) {
    walk.emplace_back(x, 0.0, 1.5);
  }
  vg::TerrainConfig config;
  config.reach = 1.0;
  const vg::Terrain t = fit_terrain(vg::HeightMap{}, {}, walk, config);
  const vg::HeightMap& m = t.heights;
  ASSERT_GT(m.width, 0);
  const auto cell = [&](double x, double y) {
    return Eigen::Vector2i(static_cast<int>(std::floor((x - m.origin.x()) / m.cell_size)),
                           static_cast<int>(std::floor((y - m.origin.y()) / m.cell_size)));
  };
  const Eigen::Vector2i on = cell(3.0, 0.8);
  EXPECT_TRUE(m.has_data(on.x(), on.y()));
  EXPECT_NEAR(m.at(on.x(), on.y()), 0.0, 1e-3);
  // Past the end of the walk, diagonally: in the grid, beyond reach.
  const Eigen::Vector2i off = cell(6.85, 0.85);
  ASSERT_LT(off.x(), m.width);
  ASSERT_LT(off.y(), m.height);
  EXPECT_FALSE(m.has_data(off.x(), off.y()));
}

TEST(TerrainTest, EmptyWithNoEvidence) {
  const vg::Terrain t = fit_terrain(vg::HeightMap{}, {}, {});
  EXPECT_EQ(t.heights.width, 0);
  EXPECT_TRUE(t.model.nodes.empty());
}

}  // namespace
