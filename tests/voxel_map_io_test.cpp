#include "vg_core/voxel_map_io.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <sstream>
#include <string>

namespace {

struct Ply {
  std::string header;
  std::string body;
};

Ply split(const std::string& data) {
  const std::string marker = "end_header\n";
  const auto end = data.find(marker);
  EXPECT_NE(end, std::string::npos);
  return {data.substr(0, end + marker.size()), data.substr(end + marker.size())};
}

std::string write(const std::vector<Eigen::Vector3i>& voxels, double voxel_size = 0.1) {
  std::ostringstream out;
  vg::write_voxels_ply(voxels, voxel_size, out);
  return out.str();
}

TEST(VoxelMapIo, SingleVoxelIsACube) {
  const Ply ply = split(write({{0, 0, 0}}));
  EXPECT_NE(ply.header.find("format binary_little_endian 1.0\n"), std::string::npos);
  EXPECT_NE(ply.header.find("element vertex 24\n"), std::string::npos);
  EXPECT_NE(ply.header.find("element face 6\n"), std::string::npos);
  // 24 vertices of 3 floats + 3 colors, then 6 quads of a count + 4 indices.
  EXPECT_EQ(ply.body.size(), 24u * 15u + 6u * 17u);
}

TEST(VoxelMapIo, SharedFacesAreDropped) {
  // Two neighbors share one face each: 12 - 2 = 10 visible faces.
  const Ply ply = split(write({{0, 0, 0}, {1, 0, 0}}));
  EXPECT_NE(ply.header.find("element face 10\n"), std::string::npos);
  EXPECT_EQ(ply.body.size(), 40u * 15u + 10u * 17u);
}

TEST(VoxelMapIo, VerticesAreInMeters) {
  const Ply ply = split(write({{2, -1, 3}}, 0.5));
  // Every vertex lies on the cube [1, 1.5] x [-0.5, 0] x [1.5, 2].
  for (std::size_t i = 0; i < 24; ++i) {
    float xyz[3];
    std::memcpy(xyz, ply.body.data() + i * 15, sizeof(xyz));
    EXPECT_TRUE(xyz[0] == 1.0f || xyz[0] == 1.5f) << xyz[0];
    EXPECT_TRUE(xyz[1] == -0.5f || xyz[1] == 0.0f) << xyz[1];
    EXPECT_TRUE(xyz[2] == 1.5f || xyz[2] == 2.0f) << xyz[2];
  }
}

TEST(VoxelMapIo, EmptyMapIsValid) {
  const Ply ply = split(write({}));
  EXPECT_NE(ply.header.find("element vertex 0\n"), std::string::npos);
  EXPECT_TRUE(ply.body.empty());
}

TEST(VoxelMapIo, SavesFile) {
  const auto path = std::filesystem::temp_directory_path() / "vg_core_voxel_map_test.ply";
  vg::save_voxels_ply({{0, 0, 0}}, 0.1, path);
  EXPECT_EQ(std::filesystem::file_size(path), write({{0, 0, 0}}).size());
  std::filesystem::remove(path);
}

}  // namespace
