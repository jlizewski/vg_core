#include "vg_core/depth_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "test_scene.hpp"
#include "vg_core/tsdf_volume.hpp"

namespace {

using vg::test::look_at;
using vg::test::render_depth;

// A camera 1.5 m up looking at the ground 2 m ahead: near pixels see it at a
// moderate angle, those toward the horizon almost edge-on.
const Eigen::Isometry3d kOblique =
    look_at({0.0, 0.0, 1.5}, {2.0, 0.0, 0.0}, Eigen::Vector3d::UnitZ());

std::size_t count_valid(const vg::DepthFrame& frame) {
  std::size_t n = 0;
  for (const float d : frame.depth) {
    n += d > 0.0f ? 1 : 0;
  }
  return n;
}

TEST(DepthFilter, DropsReadingsOutOfRange) {
  const auto frame = render_depth(vg::test::Scene{}, kOblique);
  vg::DepthFilterConfig config;
  config.max_depth = 3.0;
  const auto filtered = vg::filter_depth(frame, config);
  std::size_t kept = 0;
  for (std::size_t i = 0; i < frame.depth.size(); ++i) {
    if (frame.depth[i] > 0.0f && frame.depth[i] <= 3.0f) {
      EXPECT_EQ(filtered.depth[i], frame.depth[i]);
      ++kept;
    } else {
      EXPECT_EQ(filtered.depth[i], 0.0f);
    }
  }
  EXPECT_GT(kept, 1000u);
}

TEST(DepthFilter, DropsBothSidesOfADepthEdge) {
  vg::DepthFrame frame;
  frame.intrinsics = vg::test::test_intrinsics();
  frame.depth.assign(static_cast<std::size_t>(frame.intrinsics.width * frame.intrinsics.height),
                     2.0f);
  // A flying pixel smeared between a 2 m and a 4 m surface, and the 4 m one.
  frame.depth[frame.index(50, 50)] = 3.0f;
  for (int u = 100; u < frame.intrinsics.width; ++u) {
    for (int v = 0; v < frame.intrinsics.height; ++v) {
      frame.depth[frame.index(u, v)] = 4.0f;
    }
  }
  vg::DepthFilterConfig config;
  config.edge_ratio = 0.05;
  const auto filtered = vg::filter_depth(frame, config);

  EXPECT_EQ(filtered.depth[frame.index(50, 50)], 0.0f);
  EXPECT_EQ(filtered.depth[frame.index(49, 50)], 0.0f);  // Its neighbor is an edge too.
  EXPECT_EQ(filtered.depth[frame.index(99, 10)], 0.0f);
  EXPECT_EQ(filtered.depth[frame.index(100, 10)], 0.0f);
  EXPECT_EQ(filtered.depth[frame.index(20, 20)], 2.0f);
  EXPECT_EQ(filtered.depth[frame.index(120, 10)], 4.0f);
  // Smooth ground within mapping range isn't an edge. (Far toward the
  // horizon, neighboring pixels do differ by more than 5%.)
  const auto ground = render_depth(vg::test::Scene{}, kOblique);
  const auto smooth = vg::filter_depth(ground, config);
  std::size_t near = 0;
  for (std::size_t i = 0; i < ground.depth.size(); ++i) {
    if (ground.depth[i] > 0.0f && ground.depth[i] < 4.0f) {
      EXPECT_EQ(smooth.depth[i], ground.depth[i]) << i;
      ++near;
    }
  }
  EXPECT_GT(near, 1000u);
}

TEST(DepthFilter, DropsSurfacesSeenEdgeOn) {
  const auto frame = render_depth(vg::test::Scene{}, kOblique);
  // The camera is 1.5 m up, so the ground at range r is seen at
  // acos(1.5 / r) from head-on: 75 degrees at about 5.8 m.
  const double cutoff_range = 1.5 / std::cos(75.0 * 3.14159265358979323846 / 180.0);
  vg::DepthFilterConfig config;
  config.max_depth = 100.0;
  config.max_incidence_deg = 75.0;
  const auto filtered = vg::filter_depth(frame, config);
  const auto& k = frame.intrinsics;
  std::size_t checked = 0;
  for (int v = 1; v < k.height - 1; ++v) {
    for (int u = 1; u < k.width - 1; ++u) {
      const float d = frame.depth[frame.index(u, v)];
      if (d <= 0.0f) {
        continue;
      }
      const Eigen::Vector3d ray((u - k.cx) / k.fx * d, (v - k.cy) / k.fy * d, d);
      const double range = ray.norm();
      if (std::abs(range - cutoff_range) < 0.3) {
        continue;  // Too close to call.
      }
      EXPECT_EQ(filtered.depth[frame.index(u, v)] > 0.0f, range < cutoff_range)
          << u << "," << v << " range " << range;
      ++checked;
    }
  }
  EXPECT_GT(checked, 1000u);
}

TEST(DepthFilter, VolumeIgnoresFilteredReadings) {
  // A streak of flying pixels two pixels thick, 0.5 m above the ground: an
  // object's smeared silhouette. Without the edge filter it's mapped.
  auto frame = render_depth(vg::test::Scene{}, look_at({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0},
                                                       Eigen::Vector3d::UnitY()));
  for (int u = 60; u < 100; ++u) {
    frame.depth[frame.index(u, 60)] = 1.0f;
    frame.depth[frame.index(u, 61)] = 1.0f;
  }
  vg::TsdfConfig config;
  vg::TsdfVolume plain(config);
  plain.integrate(frame, look_at({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0}, Eigen::Vector3d::UnitY()));
  config.edge_ratio = 0.05;
  vg::TsdfVolume filtered(config);
  filtered.integrate(frame, look_at({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0}, Eigen::Vector3d::UnitY()));

  const auto above = [](const vg::TsdfVolume& volume) {
    std::size_t n = 0;
    for (const auto& v : volume.occupied_voxels()) {
      n += v.z() * volume.config().voxel_size > 0.3 ? 1 : 0;
    }
    return n;
  };
  EXPECT_GT(above(plain), 0u);
  EXPECT_EQ(above(filtered), 0u);
  EXPECT_GT(filtered.occupied_voxels().size(), 100u);
}

}  // namespace
