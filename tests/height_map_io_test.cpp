#include "vg_core/height_map_io.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {

vg::HeightMap sample_map() {
  vg::HeightMap map;
  map.origin = {-1.25, 0.5};
  map.cell_size = 0.05;
  map.width = 3;
  map.height = 2;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  // Row 0 (y = 0.5) then row 1 (y = 0.55).
  map.heights = {0.0f, 0.123456f, nan, 0.3f, -0.02f, 1.5f};
  return map;
}

void expect_same(const vg::HeightMap& a, const vg::HeightMap& b) {
  EXPECT_DOUBLE_EQ(a.origin.x(), b.origin.x());
  EXPECT_DOUBLE_EQ(a.origin.y(), b.origin.y());
  EXPECT_DOUBLE_EQ(a.cell_size, b.cell_size);
  ASSERT_EQ(a.width, b.width);
  ASSERT_EQ(a.height, b.height);
  for (int y = 0; y < a.height; ++y) {
    for (int x = 0; x < a.width; ++x) {
      ASSERT_EQ(a.has_data(x, y), b.has_data(x, y)) << x << "," << y;
      if (a.has_data(x, y)) {
        EXPECT_FLOAT_EQ(a.at(x, y), b.at(x, y)) << x << "," << y;
      }
    }
  }
}

TEST(HeightMapIo, WritesTopRowFirst) {
  std::ostringstream out;
  vg::write_height_map_ascii(sample_map(), out);
  EXPECT_EQ(out.str(),
            "ncols 3\n"
            "nrows 2\n"
            "xllcorner -1.25\n"
            "yllcorner 0.5\n"
            "cellsize 0.05\n"
            "NODATA_value -9999\n"
            "0.3 -0.02 1.5\n"
            "0 0.123456 -9999\n");
}

TEST(HeightMapIo, RoundTripsThroughStream) {
  std::stringstream buffer;
  vg::write_height_map_ascii(sample_map(), buffer);
  expect_same(sample_map(), vg::read_height_map_ascii(buffer));
}

TEST(HeightMapIo, RoundTripsThroughFile) {
  const auto path = std::filesystem::temp_directory_path() / "vg_core_height_map_test.asc";
  vg::save_height_map(sample_map(), path);
  const auto loaded = vg::load_height_map(path);
  std::filesystem::remove(path);
  expect_same(sample_map(), loaded);
}

TEST(HeightMapIo, ReadsCenterOriginAndCustomNoData) {
  std::istringstream in(
      "NCOLS 2\nNROWS 1\nXLLCENTER 0.5\nYLLCENTER 1.5\nCELLSIZE 1\nNODATA_VALUE -1\n7 -1\n");
  const auto map = vg::read_height_map_ascii(in);
  EXPECT_DOUBLE_EQ(map.origin.x(), 0.0);
  EXPECT_DOUBLE_EQ(map.origin.y(), 1.0);
  EXPECT_FLOAT_EQ(map.at(0, 0), 7.0f);
  EXPECT_FALSE(map.has_data(1, 0));
}

TEST(HeightMapIo, RejectsTruncatedGrid) {
  std::istringstream in("ncols 2\nnrows 2\nxllcorner 0\nyllcorner 0\ncellsize 1\n1 2 3\n");
  EXPECT_THROW(vg::read_height_map_ascii(in), std::runtime_error);
}

TEST(HeightMapIo, ThrowsOnMissingFile) {
  EXPECT_THROW(vg::load_height_map("does/not/exist.asc"), std::runtime_error);
}

}  // namespace
