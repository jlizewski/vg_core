#include "vg_core/onnx_backend.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>
#include <vector>

namespace {

const std::filesystem::path kData = VG_CORE_TEST_DATA_DIR;

// An 8 x 8 camera image, its left half bright.
std::vector<std::uint8_t> half_bright() {
  std::vector<std::uint8_t> rgb(8 * 8 * 3, 0);
  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 4; ++x) {
      for (int c = 0; c < 3; ++c) {
        rgb[static_cast<std::size_t>((y * 8 + x) * 3 + c)] = 255;
      }
    }
  }
  return rgb;
}

TEST(OnnxBackend, ReadsTheModelsTensors) {
  const auto nhwc = vg::make_onnx_backend(kData / "tiny_seg_nhwc.onnx");
  EXPECT_EQ(nhwc->input().width, 8);
  EXPECT_EQ(nhwc->input().channels, 3);
  EXPECT_FALSE(nhwc->input().channels_first);
  EXPECT_TRUE(nhwc->float_input());
  EXPECT_EQ(nhwc->output().width, 4);
  EXPECT_EQ(nhwc->output().channels, 151);
  EXPECT_FALSE(nhwc->output().channels_first);

  const auto nchw = vg::make_onnx_backend(kData / "tiny_seg_nchw.onnx");
  EXPECT_EQ(nchw->output().channels, 151);
  EXPECT_EQ(nchw->output().height, 4);
  EXPECT_TRUE(nchw->output().channels_first);
}

TEST(OnnxBackend, SegmentsThroughTheSegmenter) {
  const auto rgb = half_bright();
  for (const char* file : {"tiny_seg_nhwc.onnx", "tiny_seg_nchw.onnx"}) {
    SCOPED_TRACE(file);
    vg::Segmenter segmenter(vg::make_onnx_backend(kData / file), vg::deeplabv3_mnv2_ade20k());
    for (const int rotation : {0, 90}) {
      SCOPED_TRACE(rotation);
      const auto result = segmenter.segment({rgb.data(), 8, 8, vg::ImageView::Format::Rgb8},
                                            rotation, 4, 4);
      for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
          const auto expected = x < 2 ? vg::SemanticClass::Ground : vg::SemanticClass::Structure;
          EXPECT_EQ(static_cast<vg::SemanticClass>(result.labels[static_cast<std::size_t>(
                        y * 4 + x)]),
                    expected)
              << x << "," << y;
        }
      }
    }
  }
}

TEST(OnnxBackend, MissingModelThrows) {
  EXPECT_THROW(vg::make_onnx_backend(kData / "no_such_model.onnx"), std::runtime_error);
}

}  // namespace
