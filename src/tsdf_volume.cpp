#include "vg_core/tsdf_volume.hpp"

#include <algorithm>
#include <cmath>

namespace vg {
namespace {

constexpr int kBlockSize = TsdfVolume::kBlockSize;

int floor_div(int a, int b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }

std::size_t linear_index(const Eigen::Vector3i& local) {
  return static_cast<std::size_t>((local.z() * kBlockSize + local.y()) * kBlockSize + local.x());
}

Eigen::Vector3i block_of(const Eigen::Vector3i& voxel) {
  return {floor_div(voxel.x(), kBlockSize), floor_div(voxel.y(), kBlockSize),
          floor_div(voxel.z(), kBlockSize)};
}

bool valid_depth(float d, const TsdfConfig& config) {
  return std::isfinite(d) && d >= config.min_depth && d <= config.max_depth;
}

bool confident(const DepthFrame& frame, std::size_t i, const TsdfConfig& config) {
  return frame.confidence.empty() || frame.confidence[i] >= config.min_confidence;
}

}  // namespace

std::size_t TsdfVolume::IndexHash::operator()(const Eigen::Vector3i& i) const {
  // Teschner et al. spatial hash.
  const auto x = static_cast<std::size_t>(static_cast<std::uint32_t>(i.x()));
  const auto y = static_cast<std::size_t>(static_cast<std::uint32_t>(i.y()));
  const auto z = static_cast<std::size_t>(static_cast<std::uint32_t>(i.z()));
  return (x * 73856093u) ^ (y * 19349669u) ^ (z * 83492791u);
}

TsdfVolume::TsdfVolume(const TsdfConfig& config) : config_(config) {}

Eigen::Vector3i TsdfVolume::voxel_index(const Eigen::Vector3d& point) const {
  return (point / config_.voxel_size).array().floor().cast<int>();
}

Eigen::Vector3d TsdfVolume::voxel_center(const Eigen::Vector3i& voxel) const {
  return (voxel.cast<double>().array() + 0.5) * config_.voxel_size;
}

const TsdfVolume::Voxel* TsdfVolume::find_voxel(const Eigen::Vector3i& voxel) const {
  const Eigen::Vector3i block = block_of(voxel);
  const auto it = blocks_.find(block);
  if (it == blocks_.end()) {
    return nullptr;
  }
  return &it->second[linear_index(voxel - block * kBlockSize)];
}

void TsdfVolume::integrate(const DepthFrame& frame, const Eigen::Isometry3d& world_from_camera) {
  const CameraIntrinsics& k = frame.intrinsics;
  const double trunc = config_.truncation_distance;
  const double block_edge = config_.voxel_size * kBlockSize;

  // Allocate every block within the truncation band around each depth reading.
  std::unordered_set<Eigen::Vector3i, IndexHash> touched;
  for (int v = 0; v < k.height; ++v) {
    for (int u = 0; u < k.width; ++u) {
      const std::size_t i = frame.index(u, v);
      const float d = frame.depth[i];
      if (!valid_depth(d, config_) || !confident(frame, i, config_)) {
        continue;
      }
      const Eigen::Vector3d ray((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
      for (double z = d - trunc; z <= d + trunc; z += config_.voxel_size) {
        const Eigen::Vector3d p = world_from_camera * (ray * z);
        touched.insert((p / block_edge).array().floor().cast<int>());
      }
    }
  }

  const Eigen::Isometry3d camera_from_world = world_from_camera.inverse();
  for (const Eigen::Vector3i& index : touched) {
    integrate_block(index, blocks_[index], frame, camera_from_world);
    changed_.insert(index);
  }
}

void TsdfVolume::integrate_block(const Eigen::Vector3i& block_index, Block& block,
                                 const DepthFrame& frame,
                                 const Eigen::Isometry3d& camera_from_world) {
  const CameraIntrinsics& k = frame.intrinsics;
  const double trunc = config_.truncation_distance;
  const Eigen::Vector3i origin = block_index * kBlockSize;

  for (int z = 0; z < kBlockSize; ++z) {
    for (int y = 0; y < kBlockSize; ++y) {
      for (int x = 0; x < kBlockSize; ++x) {
        const Eigen::Vector3i local(x, y, z);
        const Eigen::Vector3d p = camera_from_world * voxel_center(origin + local);
        if (p.z() <= 0.0) {
          continue;
        }
        const long u = std::lround(k.fx * p.x() / p.z() + k.cx);
        const long v = std::lround(k.fy * p.y() / p.z() + k.cy);
        if (u < 0 || v < 0 || u >= k.width || v >= k.height) {
          continue;
        }
        const std::size_t i = frame.index(static_cast<int>(u), static_cast<int>(v));
        const float d = frame.depth[i];
        if (!valid_depth(d, config_) || !confident(frame, i, config_)) {
          continue;
        }
        // Projective distance along the optical axis.
        const double sdf = d - p.z();
        if (sdf < -trunc) {
          continue;  // Occluded: too far behind the observed surface.
        }
        const float tsdf = static_cast<float>(std::min(1.0, sdf / trunc));

        Voxel& voxel = block[linear_index(local)];
        const float w = voxel.weight;
        voxel.tsdf = (voxel.tsdf * w + tsdf) / (w + 1.0f);
        voxel.weight = std::min(w + 1.0f, config_.max_weight);
      }
    }
  }
}

std::optional<double> TsdfVolume::distance_at(const Eigen::Vector3d& point) const {
  const Voxel* voxel = find_voxel(voxel_index(point));
  if (voxel == nullptr || voxel->weight <= 0.0f) {
    return std::nullopt;
  }
  return static_cast<double>(voxel->tsdf) * config_.truncation_distance;
}

std::vector<Eigen::Vector3d> TsdfVolume::extract_surface_points() const {
  std::vector<Eigen::Vector3d> points;
  for (const auto& [block_index, block] : blocks_) {
    append_surface_points(block_index, block, points);
  }
  return points;
}

std::vector<Eigen::Vector3d> TsdfVolume::extract_surface_points(
    const Eigen::Vector3i& block_index) const {
  std::vector<Eigen::Vector3d> points;
  if (const auto it = blocks_.find(block_index); it != blocks_.end()) {
    append_surface_points(block_index, it->second, points);
  }
  return points;
}

std::vector<Eigen::Vector3i> TsdfVolume::take_changed_blocks() {
  std::unordered_set<Eigen::Vector3i, IndexHash> result = changed_;
  for (const Eigen::Vector3i& index : changed_) {
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Vector3i neighbor = index - Eigen::Vector3i::Unit(axis);
      if (blocks_.count(neighbor) != 0) {
        result.insert(neighbor);
      }
    }
  }
  changed_.clear();
  return {result.begin(), result.end()};
}

void TsdfVolume::append_surface_points(const Eigen::Vector3i& block_index, const Block& block,
                                       std::vector<Eigen::Vector3d>& points) const {
  const Eigen::Vector3i origin = block_index * kBlockSize;
  for (int z = 0; z < kBlockSize; ++z) {
    for (int y = 0; y < kBlockSize; ++y) {
      for (int x = 0; x < kBlockSize; ++x) {
        const Eigen::Vector3i local(x, y, z);
        const Voxel& a = block[linear_index(local)];
        // Saturated values sit at the edge of the truncation band, where a
        // sign flip is an occlusion boundary rather than a surface.
        if (a.weight <= 0.0f || std::abs(a.tsdf) >= 1.0f) {
          continue;
        }
        for (int axis = 0; axis < 3; ++axis) {
          const Eigen::Vector3i neighbor = origin + local + Eigen::Vector3i::Unit(axis);
          const Voxel* b = find_voxel(neighbor);
          if (b == nullptr || b->weight <= 0.0f || std::abs(b->tsdf) >= 1.0f ||
              (a.tsdf >= 0.0f) == (b->tsdf >= 0.0f)) {
            continue;
          }
          const double t = static_cast<double>(a.tsdf / (a.tsdf - b->tsdf));
          points.push_back(voxel_center(origin + local) +
                           t * config_.voxel_size * Eigen::Vector3d::Unit(axis));
        }
      }
    }
  }
}

}  // namespace vg
