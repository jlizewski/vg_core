#include "vg_core/diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vg {
namespace {

bool valid_depth(float d) { return std::isfinite(d) && d > 0.0f; }

}  // namespace

bool is_depth_edge(const DepthFrame& frame, int u, int v, double ratio) {
  const CameraIntrinsics& k = frame.intrinsics;
  const float d = frame.depth[frame.index(u, v)];
  const double limit = ratio * d;
  for (int dv = -1; dv <= 1; ++dv) {
    for (int du = -1; du <= 1; ++du) {
      const int nu = u + du;
      const int nv = v + dv;
      if ((du == 0 && dv == 0) || nu < 0 || nv < 0 || nu >= k.width || nv >= k.height) {
        continue;
      }
      const float n = frame.depth[frame.index(nu, nv)];
      if (valid_depth(n) && std::abs(n - d) > limit) {
        return true;
      }
    }
  }
  return false;
}

PoseTrackStats analyze_pose_track(const std::vector<PoseStamped>& poses,
                                  const PoseJumpConfig& config) {
  PoseTrackStats stats;
  stats.poses = poses.size();
  if (poses.size() < 2) {
    return stats;
  }
  stats.duration = static_cast<double>(poses.back().timestamp - poses.front().timestamp) * 1e-9;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const double dt = static_cast<double>(poses[i].timestamp - poses[i - 1].timestamp) * 1e-9;
    const Eigen::Isometry3d delta =
        poses[i - 1].world_from_frame.inverse() * poses[i].world_from_frame;
    const double translation = delta.translation().norm();
    const double rotation = std::abs(Eigen::AngleAxisd(delta.linear()).angle());
    if (dt * 1e9 > static_cast<double>(config.max_gap)) {
      ++stats.gaps;
      stats.longest_gap = std::max(stats.longest_gap, dt);
      continue;
    }
    const bool jumped =
        (translation > config.min_translation && translation > config.max_speed * dt) ||
        (rotation > config.min_rotation && rotation > config.max_angular_speed * dt);
    if (jumped) {
      stats.jumps.push_back({poses[i].timestamp, translation, rotation, dt});
    } else {
      stats.path_length += translation;
    }
  }
  std::sort(stats.jumps.begin(), stats.jumps.end(), [](const PoseJump& a, const PoseJump& b) {
    return a.translation + a.rotation > b.translation + b.rotation;
  });
  return stats;
}

void DepthStats::add(const DepthFrame& frame) {
  const CameraIntrinsics& k = frame.intrinsics;
  ++frames_;
  width_ = k.width;
  height_ = k.height;
  const bool has_confidence = !frame.confidence.empty();
  frames_with_confidence_ += has_confidence ? 1 : 0;
  pixels_ += frame.depth.size();
  for (int v = 0; v < k.height; ++v) {
    for (int u = 0; u < k.width; ++u) {
      const std::size_t i = frame.index(u, v);
      const float d = frame.depth[i];
      if (!valid_depth(d)) {
        continue;
      }
      ++valid_;
      const auto band = static_cast<std::size_t>(std::min(static_cast<int>(d), kBands - 1));
      ++valid_by_range_[band];
      if (is_depth_edge(frame, u, v, edge_ratio_)) {
        ++edges_by_range_[band];
      }
      if (has_confidence) {
        ++by_confidence_[static_cast<std::size_t>(frame.confidence[i] / 64)];
      }
    }
  }
}

SurfaceLayers find_surface_layers(const std::vector<Eigen::Vector3i>& voxels, double voxel_size,
                                  double cell_size, int min_gap_voxels) {
  SurfaceLayers result;
  result.gap.cell_size = cell_size;
  if (voxels.empty()) {
    return result;
  }
  std::vector<Eigen::Vector3i> sorted = voxels;
  std::sort(sorted.begin(), sorted.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
    if (a.y() != b.y()) return a.y() < b.y();
    if (a.x() != b.x()) return a.x() < b.x();
    return a.z() < b.z();
  });

  // Cells line up with the ground and height maps' global grid.
  const auto cell_of = [&](const Eigen::Vector3i& v) -> Eigen::Vector2i {
    return {static_cast<int>(std::floor((v.x() + 0.5) * voxel_size / cell_size)),
            static_cast<int>(std::floor((v.y() + 0.5) * voxel_size / cell_size))};
  };
  Eigen::Vector2i min = cell_of(sorted.front());
  Eigen::Vector2i max = min;
  for (const auto& v : sorted) {
    min = min.cwiseMin(cell_of(v));
    max = max.cwiseMax(cell_of(v));
  }
  HeightMap& map = result.gap;
  map.origin = min.cast<double>() * cell_size;
  map.width = max.x() - min.x() + 1;
  map.height = max.y() - min.y() + 1;
  map.heights.assign(static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height),
                     std::numeric_limits<float>::quiet_NaN());

  constexpr std::array<double, 4> kBandEdges = {0.15, 0.3, 0.6, 1.2};
  for (std::size_t begin = 0; begin < sorted.size();) {
    std::size_t end = begin + 1;
    while (end < sorted.size() && sorted[end].x() == sorted[begin].x() &&
           sorted[end].y() == sorted[begin].y()) {
      ++end;
    }
    ++result.columns;
    // The first gap of at least min_gap_voxels empty voxels above the lowest run.
    for (std::size_t i = begin + 1; i < end; ++i) {
      const int empty = sorted[i].z() - sorted[i - 1].z() - 1;
      if (empty < min_gap_voxels) {
        continue;
      }
      const double gap = empty * voxel_size;
      ++result.layered_columns;
      const auto band = static_cast<std::size_t>(
          std::upper_bound(kBandEdges.begin(), kBandEdges.end(), gap) - kBandEdges.begin());
      ++result.by_gap[band];
      const Eigen::Vector2i c = cell_of(sorted[begin]) - min;
      float& g = map.heights[static_cast<std::size_t>(c.y()) * static_cast<std::size_t>(map.width) +
                             static_cast<std::size_t>(c.x())];
      if (std::isnan(g) || gap < g) {
        g = static_cast<float>(gap);
      }
      break;
    }
    begin = end;
  }
  return result;
}

}  // namespace vg
