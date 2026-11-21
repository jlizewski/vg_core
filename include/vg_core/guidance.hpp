#pragma once

// Guidance for ground fill (docs/design/guided-mapping.md): the site's
// unknown holes (GroundState::Unknown, inside its boundary) as targets, each
// with spots to look at it from, and whether the ground is done.
//
//   - A target is a connected region of unknown cells; they're ranked
//     largest first.
//   - A spot is a measured ground cell (somewhere to stand) between min_view
//     and max_view from the hole, chosen to see as much of it as it can, in a
//     straight line not crossing a covered cell (a wall, a fence, a bed).
//     Spots are added one at a time, each seeing the most not yet seen, until
//     coverage_goal of the hole is seen or max_spots are chosen. Each says
//     which way to face.
//   - The targets are visited nearest first, from a start (the site's origin:
//     where its first walk began).
//   - Done when no target is left of at least min_area.
//
// On disk, in <site>/layers/guidance/ (Site::make_ground_layer()):
//
//   targets.csv    one line per spot: target (rank), area, center, the spot,
//                  which way to face, how far, what share of the hole it sees,
//                  and the visiting order
//   guidance.json  whether the ground is done, and counts

#include <Eigen/Core>
#include <vector>

#include "vg_core/ground_layer.hpp"

namespace vg {

namespace guidance_file {
inline constexpr const char* kDir = "layers/guidance";
inline constexpr const char* kTargets = "targets.csv";
inline constexpr const char* kSummary = "guidance.json";
}  // namespace guidance_file

struct GuidanceConfig {
  // How far from a hole a spot may be (m): phone depth reaches 3-5 m, best
  // nearer; too near, the hole doesn't fit in view.
  double min_view = 1.0;
  double max_view = 3.0;
  // Spots are added until this share of the hole is seen, up to max_spots.
  double coverage_goal = 0.9;
  int max_spots = 3;
  // Holes smaller than this (m^2) don't count against being done.
  double min_area = 0.0;
};

struct LookSpot {
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  // Site frame, m.
  double heading = 0.0;   // Which way to face, radians from x (toward what it sees).
  double distance = 0.0;  // To the nearest cell of the hole, m.
  double sees = 0.0;      // Share of the hole's cells it sees.
};

struct GuidanceTarget {
  int rank = 0;  // 1 = largest.
  std::size_t cells = 0;
  double area = 0.0;  // m^2.
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  std::vector<LookSpot> spots;
  double seen = 0.0;  // Share of its cells seen from its spots together.
  int order = 0;      // When to visit it (1 = first).
};

struct Guidance {
  std::vector<GuidanceTarget> targets;  // By rank.
  bool done = true;
};

// The guidance for a ground layer, visiting from start (site frame).
Guidance plan_guidance(const GroundLayer& ground, const Eigen::Vector2d& start = {0.0, 0.0},
                       const GuidanceConfig& config = {});

// A cell of the ground layer's grid.
struct HoleCell {
  int x = 0;
  int y = 0;
};

// The holes plan_guidance targets: 4-connected regions of Unknown cells,
// largest first (hole k is GuidanceTarget rank k + 1).
std::vector<std::vector<HoleCell>> find_holes(const GroundLayer& ground);

}  // namespace vg
