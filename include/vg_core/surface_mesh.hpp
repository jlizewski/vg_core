#pragma once

// Triangle meshes of 2.5D maps, for viewing a map in 3D: a height map becomes
// a surface with a vertex at each cell's center, and meshes are saved as
// glTF binary (.glb), which opens in Blender, browsers (three.js), Windows' 3D
// Viewer and phone 3D views alike.

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "vg_core/height_map.hpp"

namespace vg {

using Rgba = std::array<std::uint8_t, 4>;

// An indexed triangle mesh in a z-up map frame (meters), counter-clockwise
// seen from above.
struct Mesh {
  std::vector<Eigen::Vector3f> positions;
  std::vector<Eigen::Vector3f> normals;  // Unit length, one per position.
  std::vector<Rgba> colors;              // One per position.
  std::vector<std::uint32_t> indices;    // Three per triangle.

  std::size_t triangles() const { return indices.size() / 3; }
  bool empty() const { return indices.empty(); }
};

struct SurfaceMeshConfig {
  // Neighboring cells further apart in height than this (m) aren't joined,
  // leaving a gap instead of a wall; infinity joins every pair, so raised
  // beds and walls get steep sides.
  float max_step = std::numeric_limits<float>::infinity();
};

// The surface of a height map: a vertex at the center of each cell with data,
// at its height, and triangles across each 2x2 block of cells (two where all
// four have data, split along the flatter diagonal; one where three do).
// Cells with no data leave holes. color gives each cell's vertex color.
Mesh mesh_from_height_map(const HeightMap& heights, const std::function<Rgba(int x, int y)>& color,
                          const SurfaceMeshConfig& config = {});

// One mesh of a glTF scene, with its material.
struct MeshPart {
  std::string name;
  Mesh mesh;
  // Multiplies the vertex colors; alpha below 1 draws it see-through.
  float opacity = 1.0f;
  // Drawn from both sides (e.g. a canopy seen from above and below).
  bool double_sided = false;
};

// The parts as a glTF 2.0 binary (.glb) file's contents. glTF is y-up: the
// map's x stays x, its z (up) becomes y and its y becomes -z. Empty parts are
// left out.
std::string encode_glb(const std::vector<MeshPart>& parts);

// Writes encode_glb(parts) to path. Throws std::runtime_error if it can't.
void save_glb(const std::vector<MeshPart>& parts, const std::filesystem::path& path);

}  // namespace vg
