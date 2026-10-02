#pragma once

// Exporting the 3D voxel map (TsdfVolume::occupied_voxels) as a PLY mesh of
// cubes, which opens in MeshLab, Blender, CloudCompare and most 3D tools.

#include <Eigen/Core>
#include <filesystem>
#include <ostream>
#include <vector>

namespace vg {

// Writes voxels as a binary PLY mesh of cubes in world coordinates (meters).
// Faces shared by two voxels are left out, so the file holds only the visible
// shell. Vertices are colored by height, from soil brown (lowest) to leaf
// green (highest).
void write_voxels_ply(const std::vector<Eigen::Vector3i>& voxels, double voxel_size,
                      std::ostream& out);

// File wrapper. Throws std::runtime_error if the file can't be written.
void save_voxels_ply(const std::vector<Eigen::Vector3i>& voxels, double voxel_size,
                     const std::filesystem::path& path);

}  // namespace vg
