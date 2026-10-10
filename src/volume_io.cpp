#include "vg_core/volume_io.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace vg {
namespace {

constexpr std::array<char, 8> kMagic = {'V', 'G', 'M', 'A', 'P', '\0', '\0', '\0'};
constexpr std::uint32_t kVersion = 1;

// Values are copied as raw bytes: little-endian on all supported hosts.
template <typename T>
void put(std::ostream& out, T value) {
  char bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));
  out.write(bytes, sizeof(T));
}

template <typename T>
T get(std::istream& in) {
  char bytes[sizeof(T)];
  if (!in.read(bytes, sizeof(T))) {
    throw std::runtime_error("map file ends early");
  }
  T value;
  std::memcpy(&value, bytes, sizeof(T));
  return value;
}

void put_string(std::ostream& out, const std::string& s) {
  put(out, static_cast<std::uint32_t>(s.size()));
  out.write(s.data(), static_cast<std::streamsize>(s.size()));
}

std::string get_string(std::istream& in) {
  const auto size = get<std::uint32_t>(in);
  if (size > (1u << 20)) {
    throw std::runtime_error("map file has an implausible string length");
  }
  std::string s(size, '\0');
  if (!in.read(s.data(), static_cast<std::streamsize>(size))) {
    throw std::runtime_error("map file ends early");
  }
  return s;
}

}  // namespace

void write_map(const TsdfVolume& volume, const MapMetadata& metadata, std::ostream& out) {
  out.write(kMagic.data(), kMagic.size());
  put(out, kVersion);

  const TsdfConfig& config = volume.config();
  put(out, config.voxel_size);
  put(out, config.truncation_distance);
  put(out, config.min_depth);
  put(out, config.max_depth);
  put(out, config.min_confidence);
  put(out, config.max_weight);

  put_string(out, metadata.session);
  put(out, metadata.cell_size);
  put(out, static_cast<std::uint8_t>(metadata.geo ? 1 : 0));
  if (metadata.geo) {
    put(out, metadata.geo->latitude);
    put(out, metadata.geo->longitude);
    put(out, metadata.geo->altitude);
    put(out, metadata.geo->heading);
    put(out, metadata.geo->heading_sigma);
  }
  put(out, static_cast<std::uint64_t>(metadata.frames_integrated));

  const auto indices = volume.block_indices();
  put(out, static_cast<std::uint64_t>(indices.size()));
  for (const Eigen::Vector3i& index : indices) {
    put(out, static_cast<std::int32_t>(index.x()));
    put(out, static_cast<std::int32_t>(index.y()));
    put(out, static_cast<std::int32_t>(index.z()));
    for (const TsdfVolume::Voxel& voxel : *volume.find_block(index)) {
      put(out, voxel.tsdf);
      put(out, voxel.weight);
    }
  }
}

SavedMap read_map(std::istream& in) {
  std::array<char, kMagic.size()> magic{};
  if (!in.read(magic.data(), magic.size()) || magic != kMagic) {
    throw std::runtime_error("not a vg map file");
  }
  const auto version = get<std::uint32_t>(in);
  if (version != kVersion) {
    throw std::runtime_error("unsupported map file version " + std::to_string(version));
  }

  TsdfConfig config;
  config.voxel_size = get<double>(in);
  config.truncation_distance = get<double>(in);
  config.min_depth = get<double>(in);
  config.max_depth = get<double>(in);
  config.min_confidence = get<std::uint8_t>(in);
  config.max_weight = get<float>(in);
  if (!(config.voxel_size > 0.0)) {
    throw std::runtime_error("map file has an invalid voxel size");
  }

  MapMetadata metadata;
  metadata.session = get_string(in);
  metadata.cell_size = get<double>(in);
  if (get<std::uint8_t>(in) != 0) {
    GeoReference geo;
    geo.latitude = get<double>(in);
    geo.longitude = get<double>(in);
    geo.altitude = get<double>(in);
    geo.heading = get<double>(in);
    geo.heading_sigma = get<double>(in);
    metadata.geo = geo;
  }
  metadata.frames_integrated = static_cast<std::size_t>(get<std::uint64_t>(in));

  SavedMap map{TsdfVolume(config), std::move(metadata)};
  const auto blocks = get<std::uint64_t>(in);
  for (std::uint64_t b = 0; b < blocks; ++b) {
    const int x = get<std::int32_t>(in);
    const int y = get<std::int32_t>(in);
    const int z = get<std::int32_t>(in);
    TsdfVolume::Block& block = map.volume.insert_block({x, y, z});
    for (TsdfVolume::Voxel& voxel : block) {
      voxel.tsdf = get<float>(in);
      voxel.weight = get<float>(in);
    }
  }
  // Nothing has changed since the map was saved.
  map.volume.take_changed_blocks();
  return map;
}

void save_map(const TsdfVolume& volume, const MapMetadata& metadata,
              const std::filesystem::path& path) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    throw std::runtime_error("cannot write " + path.string());
  }
  write_map(volume, metadata, out);
  if (!out) {
    throw std::runtime_error("failed writing " + path.string());
  }
}

SavedMap load_map(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("cannot open " + path.string());
  }
  try {
    return read_map(in);
  } catch (const std::runtime_error& e) {
    throw std::runtime_error(path.string() + ": " + e.what());
  }
}

}  // namespace vg
