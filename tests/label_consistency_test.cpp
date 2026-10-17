#include "vg_core/label_consistency.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "test_scene.hpp"

namespace {

using vg::SemanticClass;

const auto kGround = static_cast<std::uint8_t>(SemanticClass::Ground);
const auto kStructure = static_cast<std::uint8_t>(SemanticClass::Structure);

struct View {
  Eigen::Isometry3d pose;
  vg::DepthFrame frame;
};

// The raised bed from a few views, labeled: the box structure, the ground
// ground; with swap, the other way round. (No view looks straight down: look_at
// needs its up direction off the line of sight.)
std::vector<View> views(bool swap) {
  vg::test::Scene scene;
  scene.box = vg::test::Box{{0.2, -0.3, 0.0}, {0.8, 0.3, 0.3}};
  std::vector<View> out;
  const Eigen::Vector3d target(0.5, 0.0, 0.0);
  for (const Eigen::Vector3d& eye :
       {Eigen::Vector3d(0.6, 0.2, 1.5), Eigen::Vector3d(-0.7, 0.5, 1.2),
        Eigen::Vector3d(1.7, -0.5, 1.2)}) {
    View v;
    v.pose = vg::test::look_at(eye, target, Eigen::Vector3d::UnitZ());
    v.frame = vg::test::render_depth(scene, v.pose);
    v.frame.labels = vg::test::render_labels(scene, v.pose, swap ? kStructure : kGround,
                                             swap ? kGround : kStructure);
    out.push_back(std::move(v));
  }
  return out;
}

vg::TsdfVolume fuse(const std::vector<View>& vs) {
  vg::TsdfConfig config;
  config.max_depth = 2.5;
  config.voxel_size = 0.0762;
  config.truncation_distance = 3 * config.voxel_size;
  vg::TsdfVolume volume(config);
  for (const auto& v : vs) {
    volume.integrate(v.frame, v.pose);
  }
  return volume;
}

TEST(LabelConsistency, SteadyLabelsAgreeWithTheMap) {
  const auto labeled = views(false);
  const auto volume = fuse(labeled);
  vg::LabelConsistency consistency(2, 2.5);
  for (const auto& v : labeled) {
    consistency.add(v.frame, v.pose);
  }
  const auto result = consistency.score(volume, 1);
  ASSERT_GT(result.total(), 1000u);
  EXPECT_GT(result.overall(), 0.95);
  EXPECT_GT(result.of_class(SemanticClass::Ground), 0.95);
  EXPECT_GT(result.of_class(SemanticClass::Structure), 0.8);
}

TEST(LabelConsistency, FlippingLabelsDisagree) {
  // The map from steady labels, scored with frames labeled the other way.
  const auto volume = fuse(views(false));
  vg::LabelConsistency consistency(2, 2.5);
  for (const auto& v : views(true)) {
    consistency.add(v.frame, v.pose);
  }
  EXPECT_LT(consistency.score(volume, 1).overall(), 0.1);
}

TEST(LabelConsistency, IgnoresUnlabeledFramesAndDroppedClasses) {
  const auto labeled = views(false);
  const auto volume = fuse(labeled);
  vg::LabelConsistency consistency(2, 2.5, vg::mask_of(SemanticClass::Structure));
  auto unlabeled = labeled[0].frame;
  unlabeled.labels.clear();
  consistency.add(unlabeled, labeled[0].pose);
  EXPECT_EQ(consistency.samples(), 0u);
  consistency.add(labeled[0].frame, labeled[0].pose);
  const auto result = consistency.score(volume, 1);
  EXPECT_EQ(result.readings[static_cast<std::size_t>(SemanticClass::Structure)], 0u);
  EXPECT_GT(result.readings[static_cast<std::size_t>(SemanticClass::Ground)], 0u);
}

}  // namespace
