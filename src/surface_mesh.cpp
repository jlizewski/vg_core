#include "vg_core/surface_mesh.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace vg {

Mesh mesh_from_height_map(const HeightMap& heights, const std::function<Rgba(int x, int y)>& color,
                          const SurfaceMeshConfig& config) {
  Mesh mesh;
  const int w = heights.width;
  const int h = heights.height;
  std::vector<std::int64_t> vertex(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), -1);
  const auto cell = [w](int x, int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
  };
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (!heights.has_data(x, y)) {
        continue;
      }
      vertex[cell(x, y)] = static_cast<std::int64_t>(mesh.positions.size());
      mesh.positions.emplace_back(
          static_cast<float>(heights.origin.x() + (x + 0.5) * heights.cell_size),
          static_cast<float>(heights.origin.y() + (y + 0.5) * heights.cell_size), heights.at(x, y));
      mesh.colors.push_back(color(x, y));
    }
  }

  const auto add = [&](std::int64_t a, std::int64_t b, std::int64_t c) {
    const float za = mesh.positions[static_cast<std::size_t>(a)].z();
    const float zb = mesh.positions[static_cast<std::size_t>(b)].z();
    const float zc = mesh.positions[static_cast<std::size_t>(c)].z();
    if (std::max({za, zb, zc}) - std::min({za, zb, zc}) > config.max_step) {
      return;
    }
    mesh.indices.insert(mesh.indices.end(),
                        {static_cast<std::uint32_t>(a), static_cast<std::uint32_t>(b),
                         static_cast<std::uint32_t>(c)});
  };
  // Each 2x2 block: a (x, y), b (x + 1, y), c (x, y + 1), d (x + 1, y + 1).
  // Triangles wind counter-clockwise seen from above (+z).
  for (int y = 0; y + 1 < h; ++y) {
    for (int x = 0; x + 1 < w; ++x) {
      const std::int64_t a = vertex[cell(x, y)];
      const std::int64_t b = vertex[cell(x + 1, y)];
      const std::int64_t c = vertex[cell(x, y + 1)];
      const std::int64_t d = vertex[cell(x + 1, y + 1)];
      const int present = (a >= 0) + (b >= 0) + (c >= 0) + (d >= 0);
      if (present == 4) {
        const auto z = [&](std::int64_t i) {
          return mesh.positions[static_cast<std::size_t>(i)].z();
        };
        if (std::abs(z(a) - z(d)) <= std::abs(z(b) - z(c))) {
          add(a, b, d);
          add(a, d, c);
        } else {
          add(a, b, c);
          add(b, d, c);
        }
      } else if (present == 3) {
        if (a < 0) {
          add(b, d, c);
        } else if (b < 0) {
          add(a, d, c);
        } else if (c < 0) {
          add(a, b, d);
        } else {
          add(a, b, c);
        }
      }
    }
  }

  // Vertex normals: the area-weighted normals of the triangles around each.
  mesh.normals.assign(mesh.positions.size(), Eigen::Vector3f::Zero());
  for (std::size_t t = 0; t < mesh.indices.size(); t += 3) {
    const Eigen::Vector3f& p0 = mesh.positions[mesh.indices[t]];
    const Eigen::Vector3f& p1 = mesh.positions[mesh.indices[t + 1]];
    const Eigen::Vector3f& p2 = mesh.positions[mesh.indices[t + 2]];
    const Eigen::Vector3f n = (p1 - p0).cross(p2 - p0);
    for (std::size_t k = 0; k < 3; ++k) {
      mesh.normals[mesh.indices[t + k]] += n;
    }
  }
  for (Eigen::Vector3f& n : mesh.normals) {
    const float length = n.norm();
    n = length > 0.0f ? Eigen::Vector3f(n / length) : Eigen::Vector3f::UnitZ();
  }
  return mesh;
}

namespace {

// glTF constants.
constexpr int kFloat = 5126;
constexpr int kUnsignedByte = 5121;
constexpr int kUnsignedInt = 5125;
constexpr int kArrayBuffer = 34962;
constexpr int kElementArrayBuffer = 34963;

// The map's z-up frame to glTF's y-up one.
Eigen::Vector3f to_gltf(const Eigen::Vector3f& p) { return {p.x(), p.z(), -p.y()}; }

void append(std::string& bin, const void* data, std::size_t size) {
  bin.append(static_cast<const char*>(data), size);
}

void put_u32(std::string& out, std::uint32_t v) {
  char bytes[4];
  std::memcpy(bytes, &v, 4);  // Little-endian hosts only, as elsewhere.
  out.append(bytes, 4);
}

}  // namespace

std::string encode_glb(const std::vector<MeshPart>& parts) {
  std::string bin;
  std::ostringstream views;
  std::ostringstream accessors;
  std::ostringstream meshes;
  std::ostringstream materials;
  std::ostringstream nodes;
  for (std::ostringstream* s : {&views, &accessors, &meshes, &materials, &nodes}) {
    s->imbue(std::locale::classic());
    s->precision(9);
  }
  int view_count = 0;
  int accessor_count = 0;
  int mesh_count = 0;
  const auto separator = [](int count) { return count > 0 ? "," : ""; };
  const auto add_view = [&](const std::string& data, int target) {
    views << separator(view_count) << "{\"buffer\":0,\"byteOffset\":" << bin.size()
          << ",\"byteLength\":" << data.size() << ",\"target\":" << target << '}';
    bin += data;  // Every element is a multiple of 4 bytes: stays aligned.
    return view_count++;
  };

  for (const MeshPart& part : parts) {
    const Mesh& m = part.mesh;
    if (m.empty()) {
      continue;
    }
    std::string positions;
    std::string normals;
    std::string colors;
    std::string indices;
    Eigen::Vector3f lo = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    Eigen::Vector3f hi = -lo;
    for (std::size_t i = 0; i < m.positions.size(); ++i) {
      const Eigen::Vector3f p = to_gltf(m.positions[i]);
      const Eigen::Vector3f n = to_gltf(m.normals[i]);
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
      append(positions, p.data(), 3 * sizeof(float));
      append(normals, n.data(), 3 * sizeof(float));
      append(colors, m.colors[i].data(), 4);
    }
    append(indices, m.indices.data(), m.indices.size() * sizeof(std::uint32_t));
    const std::size_t count = m.positions.size();

    const int position_view = add_view(positions, kArrayBuffer);
    const int normal_view = add_view(normals, kArrayBuffer);
    const int color_view = add_view(colors, kArrayBuffer);
    const int index_view = add_view(indices, kElementArrayBuffer);
    const int first = accessor_count;
    accessors << separator(accessor_count) << "{\"bufferView\":" << position_view
              << ",\"componentType\":" << kFloat << ",\"count\":" << count
              << ",\"type\":\"VEC3\",\"min\":[" << lo.x() << ',' << lo.y() << ',' << lo.z()
              << "],\"max\":[" << hi.x() << ',' << hi.y() << ',' << hi.z() << "]}"
              << ",{\"bufferView\":" << normal_view << ",\"componentType\":" << kFloat
              << ",\"count\":" << count << ",\"type\":\"VEC3\"}"
              << ",{\"bufferView\":" << color_view << ",\"componentType\":" << kUnsignedByte
              << ",\"normalized\":true,\"count\":" << count << ",\"type\":\"VEC4\"}"
              << ",{\"bufferView\":" << index_view << ",\"componentType\":" << kUnsignedInt
              << ",\"count\":" << m.indices.size() << ",\"type\":\"SCALAR\"}";
    accessor_count += 4;

    const bool see_through = part.opacity < 1.0f;
    materials << separator(mesh_count) << "{\"name\":\"" << part.name
              << "\",\"pbrMetallicRoughness\":{\"baseColorFactor\":[1,1,1," << part.opacity
              << "],\"metallicFactor\":0,\"roughnessFactor\":1},\"alphaMode\":\""
              << (see_through ? "BLEND" : "OPAQUE")
              << "\",\"doubleSided\":" << (part.double_sided ? "true" : "false") << '}';
    meshes << separator(mesh_count) << "{\"name\":\"" << part.name
           << "\",\"primitives\":[{\"attributes\":{\"POSITION\":" << first
           << ",\"NORMAL\":" << first + 1 << ",\"COLOR_0\":" << first + 2
           << "},\"indices\":" << first + 3 << ",\"material\":" << mesh_count << ",\"mode\":4}]}";
    nodes << separator(mesh_count) << "{\"name\":\"" << part.name << "\",\"mesh\":" << mesh_count
          << '}';
    ++mesh_count;
  }

  std::ostringstream json;
  json.imbue(std::locale::classic());
  json << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"vg_core\"},\"scene\":0,"
       << "\"scenes\":[{\"nodes\":[";
  for (int i = 0; i < mesh_count; ++i) {
    json << separator(i) << i;
  }
  json << "]}],\"nodes\":[" << nodes.str() << "]";
  if (mesh_count > 0) {
    json << ",\"meshes\":[" << meshes.str() << "],\"materials\":[" << materials.str()
         << "],\"accessors\":[" << accessors.str() << "],\"bufferViews\":[" << views.str()
         << "],\"buffers\":[{\"byteLength\":" << bin.size() << "}]";
  }
  json << '}';

  std::string json_chunk = json.str();
  json_chunk.append((4 - json_chunk.size() % 4) % 4, ' ');
  bin.append((4 - bin.size() % 4) % 4, '\0');
  const bool has_bin = !bin.empty();
  const std::size_t total = 12 + 8 + json_chunk.size() + (has_bin ? 8 + bin.size() : 0);
  std::string out;
  out.reserve(total);
  put_u32(out, 0x46546C67);  // "glTF"
  put_u32(out, 2);
  put_u32(out, static_cast<std::uint32_t>(total));
  put_u32(out, static_cast<std::uint32_t>(json_chunk.size()));
  put_u32(out, 0x4E4F534A);  // "JSON"
  out += json_chunk;
  if (has_bin) {
    put_u32(out, static_cast<std::uint32_t>(bin.size()));
    put_u32(out, 0x004E4942);  // "BIN\0"
    out += bin;
  }
  return out;
}

void save_glb(const std::vector<MeshPart>& parts, const std::filesystem::path& path) {
  const std::string data = encode_glb(parts);
  std::ofstream file(path, std::ios::binary);
  file.write(data.data(), static_cast<std::streamsize>(data.size()));
  if (!file) {
    throw std::runtime_error("cannot write " + path.string());
  }
}

}  // namespace vg
