#include "vg_core/live_height_map.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>

namespace vg {
namespace {

// Indices i with [i, i + 1) * step overlapping [lo, hi).
std::pair<int, int> overlapping(double lo, double hi, double step) {
  return {static_cast<int>(std::floor(lo / step)), static_cast<int>(std::ceil(hi / step)) - 1};
}

}  // namespace

std::size_t LiveHeightMap::Hash::operator()(const Eigen::Vector2i& i) const {
  const auto x = static_cast<std::size_t>(static_cast<std::uint32_t>(i.x()));
  const auto y = static_cast<std::size_t>(static_cast<std::uint32_t>(i.y()));
  return (x * 73856093u) ^ (y * 19349669u);
}

LiveHeightMap::LiveHeightMap(double cell_size) : cell_size_(cell_size) {}

Eigen::Vector2i LiveHeightMap::cell_of(const Eigen::Vector3d& point) const {
  return (point.head<2>() / cell_size_).array().floor().cast<int>();
}

std::vector<HeightCell> LiveHeightMap::update(const TsdfVolume& volume,
                                              const std::vector<Eigen::Vector3i>& changed_blocks) {
  block_edge_ = volume.block_edge();
  voxel_size_ = volume.config().voxel_size;

  // Surface points can sit up to one voxel past their block's far edge
  // (interpolated toward the +x/+y neighbor), so footprints are padded by that.
  std::unordered_set<Eigen::Vector2i, Hash> dirty_cells;
  for (const Eigen::Vector3i& block : changed_blocks) {
    const Eigen::Vector2i column_index = block.head<2>();
    Column& column = columns_[column_index];
    auto points = volume.extract_surface_points(block);
    if (points.empty()) {
      column.erase(block.z());
    } else {
      column[block.z()] = std::move(points);
    }
    if (column.empty()) {
      columns_.erase(column_index);
    }

    const auto [x0, x1] = overlapping(block.x() * block_edge_,
                                      (block.x() + 1) * block_edge_ + voxel_size_, cell_size_);
    const auto [y0, y1] = overlapping(block.y() * block_edge_,
                                      (block.y() + 1) * block_edge_ + voxel_size_, cell_size_);
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        dirty_cells.insert({x, y});
      }
    }
  }

  // Columns whose points can land in a dirty cell.
  std::unordered_set<Eigen::Vector2i, Hash> source_columns;
  for (const Eigen::Vector2i& cell : dirty_cells) {
    const auto [bx0, bx1] =
        overlapping(cell.x() * cell_size_ - voxel_size_, (cell.x() + 1) * cell_size_, block_edge_);
    const auto [by0, by1] =
        overlapping(cell.y() * cell_size_ - voxel_size_, (cell.y() + 1) * cell_size_, block_edge_);
    for (int by = by0; by <= by1; ++by) {
      for (int bx = bx0; bx <= bx1; ++bx) {
        if (columns_.count({bx, by}) != 0) {
          source_columns.insert({bx, by});
        }
      }
    }
  }

  std::unordered_map<Eigen::Vector2i, float, Hash> heights;
  for (const Eigen::Vector2i& column_index : source_columns) {
    for (const auto& [bz, points] : columns_.at(column_index)) {
      for (const Eigen::Vector3d& p : points) {
        const Eigen::Vector2i cell = cell_of(p);
        if (dirty_cells.count(cell) == 0) {
          continue;
        }
        const auto z = static_cast<float>(p.z());
        auto [it, inserted] = heights.emplace(cell, z);
        if (!inserted && z > it->second) {
          it->second = z;
        }
      }
    }
  }

  std::vector<HeightCell> changed;
  for (const Eigen::Vector2i& cell : dirty_cells) {
    const auto now = heights.find(cell);
    const auto before = cells_.find(cell);
    if (now == heights.end()) {
      if (before != cells_.end()) {
        cells_.erase(before);
        changed.push_back({cell.x(), cell.y(), std::numeric_limits<float>::quiet_NaN()});
      }
    } else if (before == cells_.end() || before->second != now->second) {
      cells_[cell] = now->second;
      changed.push_back({cell.x(), cell.y(), now->second});
    }
  }
  return changed;
}

HeightMap LiveHeightMap::snapshot() const {
  HeightMap map;
  map.cell_size = cell_size_;
  if (cells_.empty()) {
    return map;
  }
  Eigen::Vector2i min = cells_.begin()->first;
  Eigen::Vector2i max = min;
  for (const auto& [cell, height] : cells_) {
    min = min.cwiseMin(cell);
    max = max.cwiseMax(cell);
  }
  map.origin = min.cast<double>() * cell_size_;
  map.width = max.x() - min.x() + 1;
  map.height = max.y() - min.y() + 1;
  map.heights.assign(static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height),
                     std::numeric_limits<float>::quiet_NaN());
  for (const auto& [cell, height] : cells_) {
    const Eigen::Vector2i local = cell - min;
    map.heights[static_cast<std::size_t>(local.y()) * static_cast<std::size_t>(map.width) +
                static_cast<std::size_t>(local.x())] = height;
  }
  return map;
}

}  // namespace vg
