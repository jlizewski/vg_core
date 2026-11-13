#include "vg_core/sky.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include <unordered_set>

#include "vg_core/semantics.hpp"

namespace vg {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr char kMagic[8] = {'V', 'G', 'S', 'K', 'Y', '1', 0, 0};
// Cell indices are packed 21 bits each, offset to be unsigned.
constexpr std::int64_t kOffset = 1 << 20;
constexpr std::int64_t kMask = (1 << 21) - 1;

std::int64_t key(const Eigen::Vector3i& c) {
  return (((c.x() + kOffset) & kMask) << 42) | (((c.y() + kOffset) & kMask) << 21) |
         ((c.z() + kOffset) & kMask);
}

Eigen::Vector3i unkey(std::int64_t k) {
  return {static_cast<int>(((k >> 42) & kMask) - kOffset),
          static_cast<int>(((k >> 21) & kMask) - kOffset), static_cast<int>((k & kMask) - kOffset)};
}

template <typename T>
void put(std::ostream& out, const T& value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
T get(std::istream& in) {
  T value{};
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
  if (!in) {
    throw std::runtime_error("sky: truncated file");
  }
  return value;
}

}  // namespace

SkyEvidence::SkyEvidence(double cell_size) : cell_size_(cell_size) {
  if (!(cell_size > 0.0)) {
    throw std::invalid_argument("sky: cell size must be positive");
  }
}

void SkyEvidence::add(const DepthFrame& frame, const Eigen::Isometry3d& world_from_camera,
                      const SkyConfig& config) {
  const CameraIntrinsics& k = frame.intrinsics;
  const auto pixels = static_cast<std::size_t>(k.width) * static_cast<std::size_t>(k.height);
  if (frame.labels.size() != pixels || frame.depth.size() != pixels || k.fx <= 0.0 ||
      k.fy <= 0.0) {
    return;
  }
  ++frames_;
  const Eigen::Matrix3d r = world_from_camera.linear();
  const Eigen::Vector3d origin = world_from_camera.translation();
  const double min_up = std::sin(config.min_elevation_deg * kPi / 180.0);
  const double step = cell_size_ / 2.0;
  const auto sky = static_cast<std::uint8_t>(SemanticClass::Sky);
  const auto unknown = static_cast<std::uint8_t>(SemanticClass::Unknown);
  // Each cell counts once per frame, however many of its rays pass it.
  std::unordered_set<std::int64_t> clear;
  std::unordered_set<std::int64_t> blocked;
  const auto march = [&](const Eigen::Vector3d& direction, double from,
                         std::unordered_set<std::int64_t>& cells) {
    for (double s = from + step / 2.0; s < config.range; s += step) {
      const Eigen::Vector3d p = origin + direction * s;
      cells.insert(key((p / cell_size_).array().floor().cast<int>()));
    }
  };
  const int stride = std::max(1, config.pixel_stride);
  for (int v = 0; v < k.height; v += stride) {
    for (int u = 0; u < k.width; u += stride) {
      const std::size_t i = static_cast<std::size_t>(v) * static_cast<std::size_t>(k.width) +
                            static_cast<std::size_t>(u);
      const std::uint8_t label = frame.labels[i];
      if (label == unknown) {
        continue;
      }
      const Eigen::Vector3d ray((u + 0.5 - k.cx) / k.fx, (v + 0.5 - k.cy) / k.fy, 1.0);
      const double length = ray.norm();
      const Eigen::Vector3d direction = r * (ray / length);
      if (label == sky) {
        march(direction, 0.0, clear);
        continue;
      }
      // Not sky: blocked, by something within depth's reach (mapped by the
      // depth) or beyond it (what this is for).
      const float z = frame.depth[i];
      if (std::isfinite(z) && z > 0.0f && z * length <= config.blocked_beyond) {
        continue;
      }
      if (direction.z() >= min_up) {
        march(direction, config.blocked_beyond, blocked);
      }
    }
  }
  for (const auto c : clear) {
    ++cells_[c].clear;
  }
  for (const auto c : blocked) {
    ++cells_[c].blocked;
  }
}

void SkyEvidence::add(const SkyEvidence& other, const MapTransform& t) {
  frames_ += other.frames_;
  for (const auto& [k, counts] : other.cells_) {
    const Eigen::Vector3d center = (unkey(k).cast<double>().array() + 0.5) * other.cell_size_;
    const Eigen::Vector3i cell = (t.apply(center) / cell_size_).array().floor().cast<int>();
    Counts& c = cells_[key(cell)];
    c.clear += counts.clear;
    c.blocked += counts.blocked;
  }
}

std::vector<std::pair<Eigen::Vector3i, SkyEvidence::Counts>> SkyEvidence::cells() const {
  std::vector<std::pair<Eigen::Vector3i, Counts>> out;
  out.reserve(cells_.size());
  for (const auto& [k, counts] : cells_) {
    out.emplace_back(unkey(k), counts);
  }
  return out;
}

void SkyEvidence::set(const Eigen::Vector3i& cell, const Counts& counts) {
  cells_[key(cell)] = counts;
}

void save_sky(const SkyEvidence& sky, const std::filesystem::path& path) {
  std::ofstream out(path, std::ios::binary);
  out.write(kMagic, sizeof(kMagic));
  put(out, sky.cell_size());
  put(out, static_cast<std::uint64_t>(sky.frames()));
  const auto cells = sky.cells();
  put(out, static_cast<std::uint64_t>(cells.size()));
  for (const auto& [c, counts] : cells) {
    put(out, static_cast<std::int32_t>(c.x()));
    put(out, static_cast<std::int32_t>(c.y()));
    put(out, static_cast<std::int32_t>(c.z()));
    put(out, counts.clear);
    put(out, counts.blocked);
  }
  if (!out) {
    throw std::runtime_error("sky: cannot write " + path.string());
  }
}

SkyEvidence load_sky(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("sky: cannot open " + path.string());
  }
  char magic[sizeof(kMagic)] = {};
  in.read(magic, sizeof(magic));
  if (!in || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    throw std::runtime_error("sky: " + path.string() + " is not a sky evidence file");
  }
  SkyEvidence sky(get<double>(in));
  const auto frames = get<std::uint64_t>(in);
  const auto n = get<std::uint64_t>(in);
  for (std::uint64_t i = 0; i < n; ++i) {
    Eigen::Vector3i c;
    c.x() = get<std::int32_t>(in);
    c.y() = get<std::int32_t>(in);
    c.z() = get<std::int32_t>(in);
    SkyEvidence::Counts counts;
    counts.clear = get<std::uint32_t>(in);
    counts.blocked = get<std::uint32_t>(in);
    sky.set(c, counts);
  }
  sky.set_frames(static_cast<std::size_t>(frames));
  return sky;
}

Canopy make_canopy(const SkyEvidence& sky, const HeightMap& terrain, const CanopyConfig& config) {
  Canopy canopy;
  const double cell = sky.cell_size();
  canopy.cell_size = cell;
  // Per column: lowest and highest occupied height over the terrain.
  std::map<std::pair<int, int>, std::pair<double, double>> columns;
  for (const auto& [c, counts] : sky.cells()) {
    const bool occupied = static_cast<int>(counts.blocked) >= config.min_blocked &&
                          static_cast<int>(counts.clear) <= config.max_clear;
    if (occupied || static_cast<int>(counts.clear) > config.max_clear) {
      ++canopy.observed;
    }
    if (!occupied) {
      continue;
    }
    const Eigen::Vector3d center = (c.cast<double>().array() + 0.5) * cell;
    const Eigen::Vector2d f = (center.head<2>() - terrain.origin) / terrain.cell_size;
    const int tx = static_cast<int>(std::floor(f.x()));
    const int ty = static_cast<int>(std::floor(f.y()));
    if (tx < 0 || ty < 0 || tx >= terrain.width || ty >= terrain.height ||
        !terrain.has_data(tx, ty)) {
      continue;  // No ground known under it to measure from.
    }
    const double height = center.z() - terrain.at(tx, ty);
    if (height < config.min_height) {
      continue;
    }
    canopy.cells.push_back(c);
    auto [it, added] = columns.try_emplace({c.x(), c.y()}, height, height);
    if (!added) {
      it->second.first = std::min(it->second.first, height);
      it->second.second = std::max(it->second.second, height);
    }
  }
  canopy.bottom.cell_size = canopy.top.cell_size = cell;
  if (!columns.empty()) {
    Eigen::Vector2i lo(columns.begin()->first.first, columns.begin()->first.second);
    Eigen::Vector2i hi = lo;
    for (const auto& [xy, heights] : columns) {
      lo = lo.cwiseMin(Eigen::Vector2i(xy.first, xy.second));
      hi = hi.cwiseMax(Eigen::Vector2i(xy.first, xy.second));
    }
    for (HeightMap* m : {&canopy.bottom, &canopy.top}) {
      m->origin = lo.cast<double>() * cell;
      m->width = hi.x() - lo.x() + 1;
      m->height = hi.y() - lo.y() + 1;
      m->heights.assign(static_cast<std::size_t>(m->width) * static_cast<std::size_t>(m->height),
                        std::numeric_limits<float>::quiet_NaN());
    }
    for (const auto& [xy, heights] : columns) {
      const auto i = static_cast<std::size_t>(xy.second - lo.y()) *
                         static_cast<std::size_t>(canopy.bottom.width) +
                     static_cast<std::size_t>(xy.first - lo.x());
      canopy.bottom.heights[i] = static_cast<float>(heights.first);
      canopy.top.heights[i] = static_cast<float>(heights.second);
    }
  }
  std::sort(canopy.cells.begin(), canopy.cells.end(),
            [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
              return std::make_tuple(a.z(), a.y(), a.x()) < std::make_tuple(b.z(), b.y(), b.x());
            });
  return canopy;
}

}  // namespace vg
