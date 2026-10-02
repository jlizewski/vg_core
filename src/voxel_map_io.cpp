#include "vg_core/voxel_map_io.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <unordered_set>

namespace vg {
namespace {

struct Hash {
  std::size_t operator()(const Eigen::Vector3i& i) const {
    const auto x = static_cast<std::size_t>(static_cast<std::uint32_t>(i.x()));
    const auto y = static_cast<std::size_t>(static_cast<std::uint32_t>(i.y()));
    const auto z = static_cast<std::size_t>(static_cast<std::uint32_t>(i.z()));
    return (x * 73856093u) ^ (y * 19349669u) ^ (z * 83492791u);
  }
};

// The four corners of each cube face, as offsets from the voxel's min corner,
// counter-clockwise when seen from outside. Indexed by face: -x, +x, -y, +y, -z, +z.
constexpr std::array<std::array<std::array<int, 3>, 4>, 6> kFaces = {{
    {{{0, 0, 0}, {0, 0, 1}, {0, 1, 1}, {0, 1, 0}}},
    {{{1, 0, 0}, {1, 1, 0}, {1, 1, 1}, {1, 0, 1}}},
    {{{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}}},
    {{{0, 1, 0}, {0, 1, 1}, {1, 1, 1}, {1, 1, 0}}},
    {{{0, 0, 0}, {0, 1, 0}, {1, 1, 0}, {1, 0, 0}}},
    {{{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}}},
}};

Eigen::Vector3i face_normal(int face) {
  Eigen::Vector3i n = Eigen::Vector3i::Zero();
  n[face / 2] = face % 2 == 0 ? -1 : 1;
  return n;
}

template <typename T>
void put(std::ostream& out, T value) {
  char bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));  // Little-endian on all supported hosts.
  out.write(bytes, sizeof(T));
}

std::uint8_t lerp(int a, int b, double t) {
  return static_cast<std::uint8_t>(a + (b - a) * t + 0.5);
}

}  // namespace

void write_voxels_ply(const std::vector<Eigen::Vector3i>& voxels, double voxel_size,
                      std::ostream& out) {
  const std::unordered_set<Eigen::Vector3i, Hash> occupied(voxels.begin(), voxels.end());

  std::size_t faces = 0;
  int min_z = 0;
  int max_z = 0;
  if (!occupied.empty()) {
    min_z = max_z = occupied.begin()->z();
  }
  for (const Eigen::Vector3i& v : occupied) {
    min_z = std::min(min_z, v.z());
    max_z = std::max(max_z, v.z());
    for (int face = 0; face < 6; ++face) {
      if (occupied.count(v + face_normal(face)) == 0) {
        ++faces;
      }
    }
  }

  out << "ply\n"
      << "format binary_little_endian 1.0\n"
      << "comment vg_core voxel map, voxel size " << voxel_size << " m\n"
      << "element vertex " << faces * 4 << "\n"
      << "property float x\nproperty float y\nproperty float z\n"
      << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
      << "element face " << faces << "\n"
      << "property list uchar int vertex_indices\n"
      << "end_header\n";

  const double z_range = std::max(1, max_z - min_z);
  for (const Eigen::Vector3i& v : occupied) {
    const double t = (v.z() - min_z) / z_range;
    const std::uint8_t rgb[3] = {lerp(121, 76, t), lerp(85, 160, t), lerp(58, 60, t)};
    for (int face = 0; face < 6; ++face) {
      if (occupied.count(v + face_normal(face)) != 0) {
        continue;
      }
      for (const auto& corner : kFaces[static_cast<std::size_t>(face)]) {
        for (int axis = 0; axis < 3; ++axis) {
          put(out,
              static_cast<float>((v[axis] + corner[static_cast<std::size_t>(axis)]) * voxel_size));
        }
        out.write(reinterpret_cast<const char*>(rgb), 3);
      }
    }
  }

  std::int32_t vertex = 0;
  for (std::size_t f = 0; f < faces; ++f) {
    put<std::uint8_t>(out, 4);
    for (int i = 0; i < 4; ++i) {
      put(out, vertex++);
    }
  }
}

void save_voxels_ply(const std::vector<Eigen::Vector3i>& voxels, double voxel_size,
                     const std::filesystem::path& path) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    throw std::runtime_error("voxel map: cannot open " + path.string() + " for writing");
  }
  write_voxels_ply(voxels, voxel_size, out);
  out.flush();
  if (!out) {
    throw std::runtime_error("voxel map: failed writing " + path.string());
  }
}

}  // namespace vg
