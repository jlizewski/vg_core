#include "vg_core/voxel_cleanup.hpp"

#include <gtest/gtest.h>

namespace {

TEST(VoxelCleanup, RemovesSmallIsolatedGroups) {
  std::vector<Eigen::Vector3i> voxels;
  // A 10 x 10 ground patch with a 3-voxel post touching it diagonally.
  for (int y = 0; y < 10; ++y) {
    for (int x = 0; x < 10; ++x) {
      voxels.emplace_back(x, y, 0);
    }
  }
  for (int z = 1; z <= 3; ++z) {
    voxels.emplace_back(10, 10, z);
  }
  // A 4-voxel speck floating 2 voxels up, and a lone voxel.
  voxels.emplace_back(3, 3, 3);
  voxels.emplace_back(4, 3, 3);
  voxels.emplace_back(3, 4, 3);
  voxels.emplace_back(4, 4, 4);
  voxels.emplace_back(8, 1, 5);

  const auto result = vg::remove_small_components(voxels, 5);
  EXPECT_EQ(result.removed_components, 2u);
  EXPECT_EQ(result.removed_voxels, 5u);
  ASSERT_EQ(result.voxels.size(), 103u);
  // Input order is kept.
  EXPECT_EQ(result.voxels.front(), Eigen::Vector3i(0, 0, 0));
  EXPECT_EQ(result.voxels.back(), Eigen::Vector3i(10, 10, 3));

  EXPECT_EQ(vg::remove_small_components(voxels, 1).voxels.size(), voxels.size());
  EXPECT_EQ(vg::remove_small_components(voxels, 200).voxels.size(), 0u);
}

}  // namespace
