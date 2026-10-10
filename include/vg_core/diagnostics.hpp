#pragma once

// Diagnostics for map quality: what a recording's poses and depth look like,
// and where a built map has ghost surfaces. They point at the cause of a rough
// map: tracking jumps (pose error doubles surfaces), noisy far or edge depth
// (floating blobs), or both.

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <vector>

#include "vg_core/height_map.hpp"
#include "vg_core/sensor_data.hpp"

namespace vg {

// Whether pixel (u, v) of a depth image sits on a depth discontinuity: a valid
// 8-neighbor differs from it by more than ratio times its depth. Depth at
// such pixels is often smeared between the two surfaces ("flying pixels").
bool is_depth_edge(const DepthFrame& frame, int u, int v, double ratio);

struct PoseJumpConfig {
  // Motion between consecutive poses faster than a hand-held phone moves.
  double max_speed = 2.0;          // m/s
  double max_angular_speed = 3.0;  // rad/s
  // Jumps smaller than these are ignored, however short the interval.
  double min_translation = 0.03;  // m
  double min_rotation = 0.05;     // rad
  // Consecutive poses further apart than this are a gap, not a jump.
  Timestamp max_gap = 500'000'000;
};

struct PoseJump {
  Timestamp timestamp = 0;  // Of the pose after the jump.
  double translation = 0.0;  // m
  double rotation = 0.0;     // rad
  double dt = 0.0;           // s
};

struct PoseTrackStats {
  std::size_t poses = 0;
  double duration = 0.0;     // s
  double path_length = 0.0;  // m, excluding jumps
  std::size_t gaps = 0;
  double longest_gap = 0.0;  // s
  // Largest first.
  std::vector<PoseJump> jumps;
};

// Finds jumps and gaps in a pose track, given in time order.
PoseTrackStats analyze_pose_track(const std::vector<PoseStamped>& poses,
                                  const PoseJumpConfig& config = {});

// Running statistics over depth frames.
class DepthStats {
 public:
  // Range bands, in meters: [0, 1), [1, 2), ... and the last open ended.
  static constexpr int kBands = 7;
  // Confidence bands: [0, 64), [64, 128), [128, 192), [192, 255].
  static constexpr int kConfidenceBands = 4;

  explicit DepthStats(double edge_ratio = 0.05) : edge_ratio_(edge_ratio) {}

  void add(const DepthFrame& frame);

  std::size_t frames() const { return frames_; }
  std::size_t frames_with_confidence() const { return frames_with_confidence_; }
  std::size_t pixels() const { return pixels_; }
  // Pixels with a depth reading.
  std::size_t valid() const { return valid_; }
  const std::array<std::size_t, kBands>& valid_by_range() const { return valid_by_range_; }
  const std::array<std::size_t, kBands>& edges_by_range() const { return edges_by_range_; }
  const std::array<std::size_t, kConfidenceBands>& by_confidence() const { return by_confidence_; }
  double edge_ratio() const { return edge_ratio_; }
  // Size of the last frame.
  int width() const { return width_; }
  int height() const { return height_; }

 private:
  double edge_ratio_;
  std::size_t frames_ = 0;
  std::size_t frames_with_confidence_ = 0;
  std::size_t pixels_ = 0;
  std::size_t valid_ = 0;
  std::array<std::size_t, kBands> valid_by_range_{};
  std::array<std::size_t, kBands> edges_by_range_{};
  std::array<std::size_t, kConfidenceBands> by_confidence_{};
  int width_ = 0;
  int height_ = 0;
};

// Surfaces stacked over the same ground: in each voxel column, the gap
// between the lowest run of occupied voxels and the next one above it. A
// small gap over a wide area (a second ground a few cm to tens of cm up) is
// a ghost, usually from pose error; large gaps are things like canopies.
struct SurfaceLayers {
  // Per cell, the smallest such gap among its columns, in meters; NaN where
  // every column has a single surface.
  HeightMap gap;
  std::size_t columns = 0;
  std::size_t layered_columns = 0;
  // Layered columns by gap: [0, 0.15), [0.15, 0.3), [0.3, 0.6), [0.6, 1.2),
  // 1.2 m and up.
  std::array<std::size_t, 5> by_gap{};
};

// voxels as from TsdfVolume::occupied_voxels(), of edge voxel_size. Runs
// separated by fewer than min_gap_voxels empty voxels count as one.
SurfaceLayers find_surface_layers(const std::vector<Eigen::Vector3i>& voxels, double voxel_size,
                                  double cell_size, int min_gap_voxels = 2);

}  // namespace vg
