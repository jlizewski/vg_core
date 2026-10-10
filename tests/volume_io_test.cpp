#include "vg_core/volume_io.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>

#include "test_scene.hpp"

namespace {

vg::TsdfVolume fused_ground() {
  vg::TsdfConfig config;
  config.voxel_size = 0.05;
  config.truncation_distance = 0.15;
  config.max_weight = 32.0f;
  vg::TsdfVolume volume(config);
  const auto pose =
      vg::test::look_at({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0}, Eigen::Vector3d::UnitY());
  volume.integrate(vg::test::render_depth(vg::test::Scene{}, pose), pose);
  return volume;
}

TEST(VolumeIo, RoundTripsVolumeAndMetadata) {
  const auto volume = fused_ground();
  vg::MapMetadata metadata;
  metadata.session = "garden.mcap";
  metadata.cell_size = 0.1524;
  metadata.frames_integrated = 42;
  vg::GeoReference geo;
  geo.latitude = 42.36;
  geo.longitude = -71.06;
  geo.altitude = 10.0;
  geo.heading = 0.3;
  geo.heading_sigma = 0.05;
  metadata.geo = geo;

  std::stringstream stream;
  vg::write_map(volume, metadata, stream);
  const auto saved = vg::read_map(stream);

  EXPECT_EQ(saved.metadata.session, "garden.mcap");
  EXPECT_EQ(saved.metadata.cell_size, 0.1524);
  EXPECT_EQ(saved.metadata.frames_integrated, 42u);
  ASSERT_TRUE(saved.metadata.geo);
  EXPECT_EQ(saved.metadata.geo->latitude, geo.latitude);
  EXPECT_EQ(saved.metadata.geo->longitude, geo.longitude);
  EXPECT_EQ(saved.metadata.geo->heading, geo.heading);
  EXPECT_EQ(saved.metadata.geo->heading_sigma, geo.heading_sigma);

  const auto& config = saved.volume.config();
  EXPECT_EQ(config.voxel_size, 0.05);
  EXPECT_EQ(config.truncation_distance, 0.15);
  EXPECT_EQ(config.max_weight, 32.0f);
  const auto indices = volume.block_indices();
  ASSERT_GT(indices.size(), 0u);
  ASSERT_EQ(saved.volume.block_indices(), indices);
  for (const auto& index : indices) {
    const auto& a = *volume.find_block(index);
    const auto& b = *saved.volume.find_block(index);
    for (std::size_t i = 0; i < a.size(); ++i) {
      ASSERT_EQ(a[i].tsdf, b[i].tsdf);
      ASSERT_EQ(a[i].weight, b[i].weight);
    }
  }
  EXPECT_EQ(saved.volume.occupied_voxels().size(), volume.occupied_voxels().size());

  // Saving again gives the same bytes.
  std::stringstream again;
  vg::write_map(saved.volume, saved.metadata, again);
  EXPECT_EQ(again.str(), stream.str());
}

TEST(VolumeIo, KeepsMissingGeoReference) {
  std::stringstream stream;
  vg::write_map(vg::TsdfVolume(), vg::MapMetadata{}, stream);
  const auto saved = vg::read_map(stream);
  EXPECT_FALSE(saved.metadata.geo);
  EXPECT_EQ(saved.volume.num_blocks(), 0u);
}

TEST(VolumeIo, RejectsOtherFiles) {
  std::stringstream text("ncols 3\nnrows 2\n");
  EXPECT_THROW(vg::read_map(text), std::runtime_error);

  std::stringstream stream;
  vg::write_map(fused_ground(), vg::MapMetadata{}, stream);
  std::stringstream truncated(stream.str().substr(0, stream.str().size() / 2));
  EXPECT_THROW(vg::read_map(truncated), std::runtime_error);
}

}  // namespace
