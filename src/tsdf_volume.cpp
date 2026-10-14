#include "vg_core/tsdf_volume.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <thread>
#include <utility>

#include "vg_core/depth_filter.hpp"

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

void TsdfVolume::integrate(const DepthFrame& raw, const Eigen::Isometry3d& world_from_camera) {
  // Readings the filters reject are removed up front, so they neither place
  // blocks nor update the voxels that project onto them.
  std::optional<DepthFrame> filtered;
  if (config_.edge_ratio > 0.0 || config_.max_incidence_deg < 90.0) {
    DepthFilterConfig filter;
    filter.min_depth = config_.min_depth;
    filter.max_depth = config_.max_depth;
    filter.edge_ratio = config_.edge_ratio;
    filter.max_incidence_deg = config_.max_incidence_deg;
    filtered = filter_depth(raw, filter);
  }
  if (config_.drop_labels != 0 && raw.labels.size() == raw.depth.size()) {
    if (!filtered) {
      filtered = raw;
    }
    for (std::size_t i = 0; i < filtered->depth.size(); ++i) {
      if (filtered->labels[i] < 32 && (config_.drop_labels >> filtered->labels[i]) & 1u) {
        filtered->depth[i] = 0.0f;
      }
    }
  }
  const DepthFrame& frame = filtered ? *filtered : raw;
  const bool labeled = config_.label_votes && frame.labels.size() == frame.depth.size();
  const CameraIntrinsics& k = frame.intrinsics;
  const double trunc = config_.truncation_distance;
  const double block_edge = config_.voxel_size * kBlockSize;
  const int stride = std::max(1, config_.pixel_stride);

  // Allocate every block within the truncation band around each depth reading.
  std::unordered_set<Eigen::Vector3i, IndexHash> touched;
  for (int v = 0; v < k.height; v += stride) {
    for (int u = 0; u < k.width; u += stride) {
      const std::size_t i = frame.index(u, v);
      const float d = frame.depth[i];
      if (!valid_depth(d, config_) || !confident(frame, i, config_)) {
        continue;
      }
      const Eigen::Vector3d ray((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
      Eigen::Vector3i previous(std::numeric_limits<int>::min(), 0, 0);
      for (double z = d - trunc; z <= d + trunc; z += config_.voxel_size) {
        const Eigen::Vector3d p = world_from_camera * (ray * z);
        const Eigen::Vector3i index = (p / block_edge).array().floor().cast<int>();
        if (index != previous) {  // Consecutive samples are mostly in one block.
          touched.insert(index);
          previous = index;
        }
      }
    }
  }

  // Carving: existing blocks between the camera and the truncation band of
  // each sampled reading. Fusing them marks the voxels the ray sees through
  // as free space (the projective update gives them +1).
  if (config_.carve) {
    const int carve_stride = std::max(1, config_.carve_stride);
    for (int v = carve_stride / 2; v < k.height; v += carve_stride) {
      for (int u = carve_stride / 2; u < k.width; u += carve_stride) {
        const std::size_t i = frame.index(u, v);
        const float d = frame.depth[i];
        if (!valid_depth(d, config_) || !confident(frame, i, config_)) {
          continue;
        }
        const Eigen::Vector3d ray((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
        // Half a block per step along the ray, in units of depth.
        const double step = 0.5 * block_edge / ray.norm();
        Eigen::Vector3i previous(std::numeric_limits<int>::min(), 0, 0);
        for (double z = config_.min_depth; z < d - trunc; z += step) {
          const Eigen::Vector3d p = world_from_camera * (ray * z);
          const Eigen::Vector3i index = (p / block_edge).array().floor().cast<int>();
          if (index != previous && blocks_.count(index) != 0) {
            touched.insert(index);
          }
          previous = index;
        }
      }
    }
  }

  // Blocks are fused independently, so they can be split across threads.
  // Allocating first keeps the hash map untouched while workers run.
  struct Work {
    Eigen::Vector3i index;
    Block* block;
    LabelBlock* labels;
  };
  std::vector<Work> work;
  work.reserve(touched.size());
  for (const Eigen::Vector3i& index : touched) {
    work.push_back({index, &blocks_[index], labeled ? &labels_[index] : nullptr});
    changed_.insert(index);
  }
  // Per-pixel weights, when they aren't all 1.
  std::vector<float> weights;
  if (config_.quality_weights) {
    weights.assign(frame.depth.size(), 0.0f);
    const double reference = config_.weight_reference_depth;
    for (int v = 0; v < k.height; ++v) {
      for (int u = 0; u < k.width; ++u) {
        const std::size_t i = frame.index(u, v);
        const float d = frame.depth[i];
        if (!valid_depth(d, config_)) {
          continue;
        }
        double w = (reference / d) * (reference / d);
        if (!frame.confidence.empty()) {
          w *= frame.confidence[i] / 255.0;
        }
        const double angle = incidence_deg(frame, u, v);
        if (!std::isnan(angle)) {
          w *= std::cos(angle * 3.14159265358979323846 / 180.0);
        }
        weights[i] = static_cast<float>(w);
      }
    }
  }

  const Eigen::Isometry3d camera_from_world = world_from_camera.inverse();
  std::atomic<std::size_t> next{0};
  const auto fuse = [&]() {
    for (std::size_t w = next++; w < work.size(); w = next++) {
      integrate_block(work[w].index, *work[w].block, work[w].labels, frame, weights,
                      camera_from_world);
    }
  };
  const unsigned hardware = std::max(1u, std::thread::hardware_concurrency());
  unsigned threads = config_.threads > 0 ? static_cast<unsigned>(config_.threads) : hardware;
  // Starting threads costs about as much as fusing a few dozen blocks.
  threads = std::min<unsigned>(threads, static_cast<unsigned>(work.size() / 32 + 1));
  std::vector<std::thread> workers;
  for (unsigned t = 1; t < threads; ++t) {
    workers.emplace_back(fuse);
  }
  fuse();
  for (auto& worker : workers) {
    worker.join();
  }
}

void TsdfVolume::integrate_block(const Eigen::Vector3i& block_index, Block& block,
                                 LabelBlock* labels, const DepthFrame& frame,
                                 const std::vector<float>& weights,
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

        const float sample = weights.empty() ? 1.0f : weights[i];
        if (sample <= 0.0f) {
          continue;
        }
        Voxel& voxel = block[linear_index(local)];
        const float w = voxel.weight;
        voxel.tsdf = (voxel.tsdf * w + tsdf * sample) / (w + sample);
        voxel.weight = std::min(w + sample, config_.max_weight);

        // A vote for what the voxel is, from readings of a surface near it
        // (not from space seen through in front of one).
        const std::uint8_t label = labels != nullptr ? frame.labels[i] : 0;
        if (label != 0 && label < kSemanticClasses && sdf <= trunc) {
          LabelVotes& votes = (*labels)[linear_index(local)];
          if (votes[label] == 255) {
            for (auto& count : votes) {
              count = static_cast<std::uint8_t>(count / 2);
            }
          }
          ++votes[label];
        }
      }
    }
  }
}

std::vector<Eigen::Vector3i> TsdfVolume::block_indices() const {
  std::vector<Eigen::Vector3i> indices;
  indices.reserve(blocks_.size());
  for (const auto& entry : blocks_) {
    indices.push_back(entry.first);
  }
  std::sort(indices.begin(), indices.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
    if (a.z() != b.z()) return a.z() < b.z();
    if (a.y() != b.y()) return a.y() < b.y();
    return a.x() < b.x();
  });
  return indices;
}

const TsdfVolume::Block* TsdfVolume::find_block(const Eigen::Vector3i& index) const {
  const auto it = blocks_.find(index);
  return it == blocks_.end() ? nullptr : &it->second;
}

TsdfVolume::Block& TsdfVolume::insert_block(const Eigen::Vector3i& index) {
  changed_.insert(index);
  return blocks_[index];
}

const TsdfVolume::LabelBlock* TsdfVolume::find_labels(const Eigen::Vector3i& block_index) const {
  const auto it = labels_.find(block_index);
  return it == labels_.end() ? nullptr : &it->second;
}

TsdfVolume::LabelBlock& TsdfVolume::insert_labels(const Eigen::Vector3i& block_index) {
  return labels_[block_index];
}

std::vector<Eigen::Vector3i> TsdfVolume::label_block_indices() const {
  std::vector<Eigen::Vector3i> indices;
  indices.reserve(labels_.size());
  for (const auto& entry : labels_) {
    indices.push_back(entry.first);
  }
  std::sort(indices.begin(), indices.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
    if (a.z() != b.z()) return a.z() < b.z();
    if (a.y() != b.y()) return a.y() < b.y();
    return a.x() < b.x();
  });
  return indices;
}

TsdfVolume::LabelVotes TsdfVolume::votes_at(const Eigen::Vector3i& voxel) const {
  const Eigen::Vector3i block = block_of(voxel);
  const LabelBlock* labels = find_labels(block);
  return labels == nullptr ? LabelVotes{} : (*labels)[linear_index(voxel - block * kBlockSize)];
}

std::optional<SemanticClass> TsdfVolume::label_of(const Eigen::Vector3i& voxel,
                                                  int min_votes) const {
  const LabelVotes votes = votes_at(voxel);
  int total = 0;
  std::size_t best = 0;
  for (std::size_t c = 1; c < votes.size(); ++c) {  // Class 0 is unknown.
    total += votes[c];
    if (votes[c] > votes[best]) {
      best = c;
    }
  }
  if (votes[best] == 0 || total < std::max(1, min_votes)) {
    return std::nullopt;
  }
  return static_cast<SemanticClass>(best);
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
      for (const int side : {-1, 1}) {
        const Eigen::Vector3i neighbor = index + side * Eigen::Vector3i::Unit(axis);
        if (blocks_.count(neighbor) != 0) {
          result.insert(neighbor);
        }
      }
    }
  }
  changed_.clear();
  return {result.begin(), result.end()};
}

std::vector<Eigen::Vector3i> TsdfVolume::occupied_voxels() const {
  std::vector<Eigen::Vector3i> voxels;
  for (const auto& [block_index, block] : blocks_) {
    append_occupied_voxels(block_index, block, voxels);
  }
  return voxels;
}

std::vector<Eigen::Vector3i> TsdfVolume::occupied_voxels(const Eigen::Vector3i& block_index) const {
  std::vector<Eigen::Vector3i> voxels;
  if (const auto it = blocks_.find(block_index); it != blocks_.end()) {
    append_occupied_voxels(block_index, it->second, voxels);
  }
  return voxels;
}

void TsdfVolume::append_occupied_voxels(const Eigen::Vector3i& block_index, const Block& block,
                                        std::vector<Eigen::Vector3i>& voxels) const {
  const Eigen::Vector3i origin = block_index * kBlockSize;
  for (int z = 0; z < kBlockSize; ++z) {
    for (int y = 0; y < kBlockSize; ++y) {
      for (int x = 0; x < kBlockSize; ++x) {
        const Eigen::Vector3i local(x, y, z);
        const Voxel& v = block[linear_index(local)];
        // Behind a surface, but not saturated: a saturated value next to free
        // space is an occlusion edge, not a surface.
        if (v.weight <= 0.0f || v.weight < config_.min_weight || v.tsdf >= 0.0f ||
            v.tsdf <= -1.0f) {
          continue;
        }
        bool in_front_seen = false;
        for (int axis = 0; axis < 3 && !in_front_seen; ++axis) {
          for (const int side : {-1, 1}) {
            const Voxel* n = find_voxel(origin + local + side * Eigen::Vector3i::Unit(axis));
            if (n != nullptr && n->weight > 0.0f && n->tsdf >= 0.0f) {
              in_front_seen = true;
              break;
            }
          }
        }
        if (in_front_seen) {
          voxels.push_back(origin + local);
        }
      }
    }
  }
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
        if (a.weight <= 0.0f || a.weight < config_.min_weight || std::abs(a.tsdf) >= 1.0f) {
          continue;
        }
        for (int axis = 0; axis < 3; ++axis) {
          const Eigen::Vector3i neighbor = origin + local + Eigen::Vector3i::Unit(axis);
          const Voxel* b = find_voxel(neighbor);
          if (b == nullptr || b->weight <= 0.0f || b->weight < config_.min_weight ||
              std::abs(b->tsdf) >= 1.0f ||
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
