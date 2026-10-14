#include "vg_core/pose_jumps.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

constexpr vg::Timestamp kFrame = 33'333'333;  // 30 Hz
constexpr double kDegree = 3.14159265358979323846 / 180.0;

// A walk along x at 1 m/s, turning slowly, 1.4 m up.
Eigen::Isometry3d true_pose(int i) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(0.2 * kDegree * i, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.translation() = Eigen::Vector3d(i / 30.0, 0.0, 1.4);
  return pose;
}

TEST(PoseJumps, UndoesTrackerSnaps) {
  // The tracker snaps its world 20 cm up and 2 degrees around at frame 50,
  // then 8 cm sideways at frame 120.
  Eigen::Isometry3d snap1 = Eigen::Isometry3d::Identity();
  snap1.linear() = Eigen::AngleAxisd(2.0 * kDegree, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  snap1.translation() = Eigen::Vector3d(0.0, 0.0, 0.2);
  Eigen::Isometry3d snap2 = Eigen::Isometry3d::Identity();
  snap2.translation() = Eigen::Vector3d(0.0, 0.08, 0.0);

  vg::PoseJumpFilter filter;
  for (int i = 0; i < 200; ++i) {
    vg::PoseStamped tracked;
    tracked.timestamp = i * kFrame;
    tracked.world_from_frame = true_pose(i);
    if (i >= 50) tracked.world_from_frame = snap1 * tracked.world_from_frame;
    if (i >= 120) tracked.world_from_frame = snap2 * tracked.world_from_frame;

    const auto corrected = filter.correct(tracked);
    // Within a millimeter: across a snap the filter assumes the last step's
    // motion repeats, which this walk (straight while turning) doesn't quite.
    const Eigen::Isometry3d error = true_pose(i).inverse() * corrected.world_from_frame;
    ASSERT_LT(error.translation().norm(), 1e-3) << "frame " << i;
    ASSERT_LT(std::abs(Eigen::AngleAxisd(error.linear()).angle()), 1e-6) << "frame " << i;
  }
  EXPECT_EQ(filter.jumps(), 2u);
}

TEST(PoseJumps, LeavesSmoothMotionAndGapsAlone) {
  vg::PoseJumpFilter filter;
  for (int i = 0; i < 100; ++i) {
    vg::PoseStamped pose;
    // A 1 s gap at frame 60, across which the phone moved 1 m.
    pose.timestamp = i * kFrame + (i >= 60 ? 1'000'000'000 : 0);
    pose.world_from_frame = true_pose(i + (i >= 60 ? 30 : 0));
    const auto corrected = filter.correct(pose);
    ASSERT_TRUE(corrected.world_from_frame.isApprox(pose.world_from_frame)) << i;
  }
  EXPECT_EQ(filter.jumps(), 0u);
}

}  // namespace
