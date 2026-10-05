#include "vg_core/depth_spool.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

namespace {

vg::DepthFrame make_frame(vg::Timestamp t, float scale, bool with_confidence) {
  vg::DepthFrame frame;
  frame.timestamp = t;
  frame.intrinsics = {4, 3, 100.0, 101.0, 1.5, 1.0};
  for (int i = 0; i < 12; ++i) {
    frame.depth.push_back(i == 5 ? 0.0f : scale * static_cast<float>(i + 1) + 0.0004f);
    if (with_confidence) {
      frame.confidence.push_back(static_cast<std::uint8_t>(i * 20));
    }
  }
  frame.depth[7] = std::nanf("");
  return frame;
}

Eigen::Isometry3d make_pose(double x) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(Eigen::Vector3d(x, 2.0, 3.0));
  pose.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  return pose;
}

void round_trip(vg::DepthSpool& spool) {
  EXPECT_TRUE(spool.empty());
  EXPECT_FALSE(spool.pop().has_value());

  spool.push("rgb", make_frame(10, 0.5f, false), make_pose(1.0));
  spool.push("left", make_frame(20, 0.25f, true), make_pose(2.0));
  EXPECT_EQ(spool.size(), 2u);

  const auto first = spool.pop();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->camera, "rgb");
  EXPECT_EQ(first->frame.timestamp, 10);
  EXPECT_EQ(first->frame.intrinsics.width, 4);
  EXPECT_DOUBLE_EQ(first->frame.intrinsics.fy, 101.0);
  EXPECT_TRUE(first->frame.confidence.empty());
  EXPECT_TRUE(first->world_from_camera.isApprox(make_pose(1.0)));
  ASSERT_EQ(first->frame.depth.size(), 12u);
  EXPECT_FLOAT_EQ(first->frame.depth[0], 0.5f);  // Rounded to the millimeter.
  EXPECT_FLOAT_EQ(first->frame.depth[5], 0.0f);
  EXPECT_FLOAT_EQ(first->frame.depth[7], 0.0f);  // NaN reads back as no data.

  // Pushing after popping keeps the order.
  spool.push("rgb", make_frame(30, 1.0f, false), make_pose(3.0));
  const auto second = spool.pop();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->camera, "left");
  EXPECT_EQ(second->frame.confidence.size(), 12u);
  EXPECT_EQ(second->frame.confidence[3], 60);
  EXPECT_FLOAT_EQ(second->frame.depth[1], 0.5f);
  const auto third = spool.pop();
  ASSERT_TRUE(third.has_value());
  EXPECT_EQ(third->frame.timestamp, 30);
  EXPECT_TRUE(third->world_from_camera.isApprox(make_pose(3.0)));
  EXPECT_TRUE(spool.empty());
}

TEST(DepthSpool, RoundTripsInMemory) {
  vg::DepthSpool spool;
  round_trip(spool);
}

TEST(DepthSpool, RoundTripsThroughAFileAndDeletesIt) {
  const auto path = (std::filesystem::temp_directory_path() / "vg_depth_spool_test.bin").string();
  {
    vg::DepthSpool spool(path);
    round_trip(spool);
    EXPECT_TRUE(std::filesystem::exists(path));
  }
  EXPECT_FALSE(std::filesystem::exists(path));
}

}  // namespace
