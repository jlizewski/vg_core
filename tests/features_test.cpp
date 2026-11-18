#include "vg_core/features.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace {

using vg::Feature;
using vg::FeatureKind;
using vg::SemanticClass;
using vg::WalkObject;

constexpr double kPi = 3.14159265358979323846;

WalkObject object(SemanticClass label, Eigen::Vector2d center, double length, double width,
                  double axis_deg, double top, bool grounded = true) {
  WalkObject o;
  o.label = label;
  o.grounded = grounded;
  o.center = center;
  o.length = length;
  o.width = width;
  o.axis = axis_deg * kPi / 180.0;
  o.top_above_ground = top;
  o.footprint_area = length * width;
  return o;
}

TEST(Features, TellsPostsLinesAndBlocksApart) {
  const std::vector<WalkObject> objects = {
      object(SemanticClass::Tree, {1, 1}, 0.4, 0.35, 0, 3.0),          // trunk: post
      object(SemanticClass::Structure, {5, 0}, 6.0, 0.2, 90, 1.5),     // fence: line
      object(SemanticClass::Structure, {3, 3}, 1.2, 0.9, 30, 0.4),     // bed: block
      object(SemanticClass::Plant, {4, 4}, 0.3, 0.3, 0, 0.1),          // too low
      object(SemanticClass::Tree, {6, 6}, 3.0, 3.0, 0, 4.0, false),    // canopy: overhang
      object(SemanticClass::Ground, {7, 7}, 2.0, 2.0, 0, 0.5),         // ground, not joined
      object(SemanticClass::Unknown, {8, 8}, 0.8, 0.2, 0, 0.5),        // small and thin: nothing
  };
  const std::vector<Feature> f = vg::walk_features(objects);
  ASSERT_EQ(f.size(), 3u);
  EXPECT_EQ(f[0].kind, FeatureKind::Post);
  EXPECT_EQ(f[0].label, SemanticClass::Tree);
  EXPECT_DOUBLE_EQ(f[0].height, 3.0);
  EXPECT_EQ(f[1].kind, FeatureKind::Line);
  EXPECT_NEAR(f[1].axis, kPi / 2, 1e-12);
  EXPECT_EQ(f[2].kind, FeatureKind::Block);
}

TEST(Features, MovesIntoTheSiteFrame) {
  Feature f;
  f.kind = FeatureKind::Line;
  f.center = {1.0, 0.0};
  f.axis = 170.0 * kPi / 180.0;
  vg::MapTransform t;
  t.yaw = kPi / 2;
  t.translation = {10.0, 0.0, 0.0};
  const Feature site = vg::to_site(f, t);
  EXPECT_NEAR(site.center.x(), 10.0, 1e-12);
  EXPECT_NEAR(site.center.y(), 1.0, 1e-12);
  EXPECT_NEAR(site.axis, 80.0 * kPi / 180.0, 1e-12);  // 260 degrees, as a line's direction.
}

Feature post(double x, double y) {
  Feature f;
  f.kind = FeatureKind::Post;
  f.center = {x, y};
  f.length = f.width = 0.3;
  f.height = 1.5;
  return f;
}

Feature line(double x, double y, double length, double axis_deg) {
  Feature f;
  f.kind = FeatureKind::Line;
  f.center = {x, y};
  f.length = length;
  f.width = 0.1;
  f.axis = axis_deg * kPi / 180.0;
  f.height = 1.2;
  return f;
}

vg::WalkFeatures walk(const std::string& id, std::vector<Feature> features,
                      bool covers_everything = true) {
  vg::WalkFeatures w;
  w.id = id;
  w.recorded = id;
  w.features = std::move(features);
  w.covers = [covers_everything](const Eigen::Vector2d&) { return covers_everything; };
  return w;
}

TEST(Features, MatchesSightingsAcrossWalks) {
  // Two posts and a fence, seen by three walks a little apart; the fence seen
  // whole once and in part twice.
  const vg::FeatureLayer layer = vg::match_features({
      walk("a", {post(0, 0), post(3, 0), line(0, 5, 6.0, 0)}),
      walk("b", {post(0.1, 0.05), post(3.05, -0.1), line(-1.5, 5.1, 3.0, 2)}),
      walk("c", {post(-0.05, 0.1), line(1.0, 4.95, 4.0, 179)}),
  });
  ASSERT_EQ(layer.elements.size(), 3u);
  const vg::SiteFeature& p1 = layer.elements[0];
  EXPECT_EQ(p1.id, "post-1");
  EXPECT_EQ(p1.seen, 3);
  EXPECT_NEAR(p1.feature.center.x(), 0.05 / 3, 1e-9);
  EXPECT_NEAR(p1.feature.center.y(), 0.05, 1e-9);
  EXPECT_LT(p1.spread, 0.12);
  EXPECT_EQ(p1.first_seen, "a");
  EXPECT_EQ(p1.last_seen, "c");

  const vg::SiteFeature& p2 = layer.elements[1];
  EXPECT_EQ(p2.id, "post-2");
  EXPECT_EQ(p2.seen, 2);
  EXPECT_EQ(p2.missed, 1);  // c covered it but didn't see it.
  EXPECT_FALSE(p2.retired);

  const vg::SiteFeature& fence = layer.elements[2];
  EXPECT_EQ(fence.id, "line-1");
  EXPECT_EQ(fence.seen, 3);  // 179 degrees is 1 degree from 0, as lines go.
  EXPECT_LT(std::min(fence.feature.axis, kPi - fence.feature.axis), 2.0 * kPi / 180.0);
  // Its sightings' centers lie up to 1.5 m apart along it, but its spread is
  // across it.
  EXPECT_LT(fence.spread, 0.15);
  EXPECT_EQ(layer.sightings.size(), 8u);
}

TEST(Features, ASmallBlockCanBeAPostSeenAnotherWay) {
  Feature stump = post(0.1, 0);
  stump.kind = FeatureKind::Block;
  stump.length = 0.7;
  stump.width = 0.5;
  const vg::FeatureLayer layer = vg::match_features({walk("a", {post(0, 0)}), walk("b", {stump})});
  ASSERT_EQ(layer.elements.size(), 1u);
  EXPECT_EQ(layer.elements[0].seen, 2);
}

TEST(Features, RetiresAnElementMissedByWalksThatCoveredIt) {
  const vg::FeatureLayer layer = vg::match_features({
      walk("a", {post(0, 0)}),
      walk("b", {}),
      walk("c", {}, false),  // Didn't cover it: not a miss.
      walk("d", {}),
      walk("e", {post(0, 0)}),  // A new post where the retired one was.
  });
  ASSERT_EQ(layer.elements.size(), 2u);
  EXPECT_TRUE(layer.elements[0].retired);
  EXPECT_EQ(layer.elements[0].missed, 2);
  EXPECT_EQ(layer.elements[1].id, "post-2");
  EXPECT_EQ(layer.elements[1].first_seen, "e");
}

TEST(Features, KeepsDifferentThingsApart) {
  // A post and a bed at the same spot, posts 0.8 m apart, crossing lines.
  Feature block = post(0, 0);
  block.kind = FeatureKind::Block;
  block.length = 1.5;
  block.width = 1.2;
  const vg::FeatureLayer layer = vg::match_features({
      walk("a", {post(0, 0), post(2, 0), line(0, 5, 4.0, 0)}),
      walk("b", {block, post(2.8, 0), line(0, 5, 4.0, 90)}),
  });
  EXPECT_EQ(layer.elements.size(), 6u);
}

}  // namespace
