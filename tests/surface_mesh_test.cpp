#include "vg_core/surface_mesh.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <string>

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

vg::HeightMap grid(int width, int height, std::vector<float> heights) {
  vg::HeightMap map;
  map.origin = {10.0, -2.0};
  map.cell_size = 0.5;
  map.width = width;
  map.height = height;
  map.heights = std::move(heights);
  return map;
}

vg::Rgba gray(int, int) { return {100, 100, 100, 255}; }

std::uint32_t u32(const std::string& data, std::size_t at) {
  std::uint32_t v = 0;
  std::memcpy(&v, data.data() + at, 4);
  return v;
}

TEST(SurfaceMesh, JoinsEveryBlockOfFourCells) {
  const vg::Mesh mesh = vg::mesh_from_height_map(grid(3, 3, std::vector<float>(9, 0.2f)), gray);
  ASSERT_EQ(mesh.positions.size(), 9u);
  EXPECT_EQ(mesh.triangles(), 8u);
  // Vertices at cell centers, at their heights.
  EXPECT_FLOAT_EQ(mesh.positions[0].x(), 10.25f);
  EXPECT_FLOAT_EQ(mesh.positions[0].y(), -1.75f);
  EXPECT_FLOAT_EQ(mesh.positions[0].z(), 0.2f);
  // Counter-clockwise seen from above, and normals pointing up.
  for (std::size_t t = 0; t < mesh.indices.size(); t += 3) {
    const Eigen::Vector3f a = mesh.positions[mesh.indices[t]];
    const Eigen::Vector3f b = mesh.positions[mesh.indices[t + 1]];
    const Eigen::Vector3f c = mesh.positions[mesh.indices[t + 2]];
    EXPECT_GT((b - a).cross(c - a).z(), 0.0f);
  }
  for (const Eigen::Vector3f& n : mesh.normals) {
    EXPECT_NEAR(n.z(), 1.0f, 1e-6f);
  }
}

TEST(SurfaceMesh, LeavesHolesWhereThereIsNoData) {
  // The middle cell is missing: each of the four blocks keeps one triangle.
  std::vector<float> heights(9, 0.0f);
  heights[4] = kNaN;
  const vg::Mesh mesh = vg::mesh_from_height_map(grid(3, 3, heights), gray);
  EXPECT_EQ(mesh.positions.size(), 8u);
  EXPECT_EQ(mesh.triangles(), 4u);
}

TEST(SurfaceMesh, TiltsNormalsOnASlope) {
  // Rising along x: normals lean back toward -x.
  const vg::Mesh mesh = vg::mesh_from_height_map(grid(2, 2, {0.0f, 0.5f, 0.0f, 0.5f}), gray);
  ASSERT_EQ(mesh.triangles(), 2u);
  for (const Eigen::Vector3f& n : mesh.normals) {
    EXPECT_LT(n.x(), -0.5f);
    EXPECT_GT(n.z(), 0.5f);
  }
}

TEST(SurfaceMesh, MaxStepLeavesAGapAtTallSteps) {
  vg::SurfaceMeshConfig config;
  config.max_step = 0.3f;
  const vg::Mesh mesh =
      vg::mesh_from_height_map(grid(2, 2, {0.0f, 1.0f, 0.0f, 1.0f}), gray, config);
  EXPECT_TRUE(mesh.empty());
}

TEST(SurfaceMesh, ColorsEachCell) {
  const vg::Mesh mesh = vg::mesh_from_height_map(grid(2, 1, {0.0f, 0.0f}), [](int x, int) {
    return vg::Rgba{static_cast<std::uint8_t>(x), 0, 0, 255};
  });
  ASSERT_EQ(mesh.colors.size(), 2u);
  EXPECT_EQ(mesh.colors[0][0], 0);
  EXPECT_EQ(mesh.colors[1][0], 1);
}

TEST(Glb, WritesAValidYUpFile) {
  vg::MeshPart surface;
  surface.name = "surface";
  surface.mesh = vg::mesh_from_height_map(grid(3, 3, std::vector<float>(9, 0.75f)), gray);
  vg::MeshPart canopy;  // Empty: left out.
  canopy.name = "canopy";
  canopy.opacity = 0.5f;
  const std::string glb = vg::encode_glb({surface, canopy});

  ASSERT_GE(glb.size(), 20u);
  EXPECT_EQ(u32(glb, 0), 0x46546C67u);  // "glTF"
  EXPECT_EQ(u32(glb, 4), 2u);
  EXPECT_EQ(u32(glb, 8), glb.size());
  const std::uint32_t json_length = u32(glb, 12);
  EXPECT_EQ(json_length % 4, 0u);
  EXPECT_EQ(u32(glb, 16), 0x4E4F534Au);  // "JSON"
  const std::string json = glb.substr(20, json_length);
  EXPECT_NE(json.find("\"POSITION\":0"), std::string::npos);
  EXPECT_NE(json.find("\"count\":9"), std::string::npos);
  EXPECT_NE(json.find("\"count\":24"), std::string::npos);  // 8 triangles' indices.
  EXPECT_EQ(json.find("canopy"), std::string::npos);
  // Up is glTF's y: every vertex is at y 0.75.
  EXPECT_NE(json.find("\"min\":[10.25,0.75,"), std::string::npos) << json;

  const std::size_t bin_header = 20 + json_length;
  ASSERT_LE(bin_header + 8, glb.size());
  EXPECT_EQ(u32(glb, bin_header + 4), 0x004E4942u);  // "BIN\0"
  const std::uint32_t bin_length = u32(glb, bin_header);
  EXPECT_EQ(bin_header + 8 + bin_length, glb.size());
  // Positions, normals (12 bytes a vertex each), colors (4) and indices (4 each).
  EXPECT_EQ(bin_length, 9u * 12 + 9u * 12 + 9u * 4 + 24u * 4);
}

TEST(Glb, EmptySceneIsStillValid) {
  const std::string glb = vg::encode_glb({});
  EXPECT_EQ(u32(glb, 0), 0x46546C67u);
  EXPECT_EQ(u32(glb, 8), glb.size());
}

}  // namespace
