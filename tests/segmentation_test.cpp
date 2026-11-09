#include "vg_core/segmentation.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <vector>

namespace {

using vg::SemanticClass;

constexpr int kGrass = 10;  // ADE20K label: ground.
constexpr int kWall = 1;    // ADE20K label: structure.

// A stand-in model: labels each output cell grass if the input pixels under it
// are bright, wall if dark. It depends only on brightness, so it labels a
// scene the same whichever way up it's shown.
class BrightnessModel : public vg::InferenceBackend {
 public:
  BrightnessModel(bool channels_first, int outputs, bool float_in)
      : channels_first_(channels_first), outputs_(outputs), float_in_(float_in) {}

  Tensor input() const override { return {32, 32, 3, channels_first_}; }
  Tensor output() const override { return {8, 8, outputs_, channels_first_}; }
  bool float_input() const override { return float_in_; }
  std::string name() const override { return "test"; }

  void run(const void* input, float* scores) override {
    const int in_size = 32;
    const int cells = 64;
    const int offset = 151 - outputs_;  // Label id of output 0.
    for (int cy = 0; cy < 8; ++cy) {
      for (int cx = 0; cx < 8; ++cx) {
        // Mean red value over the cell's 4x4 input pixels.
        double sum = 0.0;
        for (int y = cy * 4; y < cy * 4 + 4; ++y) {
          for (int x = cx * 4; x < cx * 4 + 4; ++x) {
            const int i = y * in_size + x;
            const int o = channels_first_ ? i : 3 * i;
            sum += float_in_ ? static_cast<const float*>(input)[o]
                             : (static_cast<const std::uint8_t*>(input)[o] - 127.5) / 127.5;
          }
        }
        const int label = sum > 0.0 ? kGrass : kWall;
        const int cell = cy * 8 + cx;
        for (int c = 0; c < outputs_; ++c) {
          const float score = c + offset == label ? 1.0f : 0.0f;
          scores[channels_first_ ? c * cells + cell : cell * outputs_ + c] = score;
        }
      }
    }
  }

 private:
  bool channels_first_;
  int outputs_;
  bool float_in_;
};

vg::Segmenter make_segmenter(bool channels_first = false, int outputs = 151,
                             bool float_in = true) {
  return vg::Segmenter(std::make_unique<BrightnessModel>(channels_first, outputs, float_in),
                       vg::deeplabv3_mnv2_ade20k());
}

// A 64 x 32 camera image: the left quarter bright, the rest dark.
std::vector<std::uint8_t> test_rgb() {
  std::vector<std::uint8_t> rgb(64 * 32 * 3, 0);
  for (int y = 0; y < 32; ++y) {
    for (int x = 0; x < 16; ++x) {
      for (int c = 0; c < 3; ++c) {
        rgb[(y * 64 + x) * 3 + c] = 255;
      }
    }
  }
  return rgb;
}

// The same image as NV21 (gray: neutral chroma).
std::vector<std::uint8_t> test_nv21() {
  std::vector<std::uint8_t> nv21(64 * 32 + 64 * 16, 128);
  for (int y = 0; y < 32; ++y) {
    for (int x = 0; x < 64; ++x) {
      nv21[y * 64 + x] = x < 16 ? 255 : 0;
    }
  }
  return nv21;
}

// Ground in the depth image's left quarter, structure elsewhere: what the
// camera image shows, in the camera's orientation.
void expect_left_quarter_ground(const vg::Segmenter::Result& r, int width, int height) {
  ASSERT_EQ(r.labels.size(), static_cast<std::size_t>(width * height));
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto expected = x < width / 4 ? SemanticClass::Ground : SemanticClass::Structure;
      ASSERT_EQ(static_cast<SemanticClass>(r.labels[static_cast<std::size_t>(y * width + x)]),
                expected)
          << "pixel " << x << "," << y;
    }
  }
  EXPECT_EQ(r.counts[static_cast<std::size_t>(SemanticClass::Ground)],
            static_cast<std::size_t>(width / 4 * height));
}

TEST(Segmentation, LabelsLandOnTheirCameraPixelsWhateverTheRotation) {
  const auto rgb = test_rgb();
  const vg::ImageView image{rgb.data(), 64, 32, vg::ImageView::Format::Rgb8};
  auto segmenter = make_segmenter();
  for (const int rotation : {0, 90, 180, 270}) {
    SCOPED_TRACE(rotation);
    expect_left_quarter_ground(segmenter.segment(image, rotation, 16, 8), 16, 8);
  }
}

TEST(Segmentation, ReadsNv21LikeRgb) {
  const auto nv21 = test_nv21();
  const vg::ImageView image{nv21.data(), 64, 32, vg::ImageView::Format::Nv21};
  auto segmenter = make_segmenter();
  for (const int rotation : {0, 90}) {
    SCOPED_TRACE(rotation);
    expect_left_quarter_ground(segmenter.segment(image, rotation, 16, 8), 16, 8);
  }
}

TEST(Segmentation, HandlesLayoutsInputTypesAndClassOffsets) {
  const auto rgb = test_rgb();
  const vg::ImageView image{rgb.data(), 64, 32, vg::ImageView::Format::Rgb8};
  // NCHW tensors, uint8 input, and 150 outputs (no "other"), as SegFormer has.
  auto nchw = make_segmenter(true, 151, true);
  expect_left_quarter_ground(nchw.segment(image, 90, 16, 8), 16, 8);
  auto bytes = make_segmenter(false, 151, false);
  expect_left_quarter_ground(bytes.segment(image, 270, 16, 8), 16, 8);
  auto no_other = make_segmenter(false, 150, true);
  expect_left_quarter_ground(no_other.segment(image, 0, 16, 8), 16, 8);
}

TEST(Segmentation, DepthFinerThanTheModelOutput) {
  // 128 x 64 depth from an 8 x 8 output: each depth pixel falls in one cell.
  const auto rgb = test_rgb();
  auto segmenter = make_segmenter();
  expect_left_quarter_ground(
      segmenter.segment({rgb.data(), 64, 32, vg::ImageView::Format::Rgb8}, 90, 128, 64), 128, 64);
}

TEST(Segmentation, RejectsBadInput) {
  auto segmenter = make_segmenter();
  const auto rgb = test_rgb();
  EXPECT_THROW(segmenter.segment({rgb.data(), 64, 32, vg::ImageView::Format::Rgb8}, 45, 16, 8),
               std::invalid_argument);
  EXPECT_THROW(segmenter.segment({nullptr, 64, 32, vg::ImageView::Format::Rgb8}, 0, 16, 8),
               std::invalid_argument);
  // More outputs than the model has labels.
  EXPECT_THROW(make_segmenter(false, 200, true), std::invalid_argument);
}

TEST(Segmentation, Ade20kTableMatchesTheApp) {
  const auto& t = vg::ade20k_classes();
  EXPECT_EQ(t[0], SemanticClass::Unknown);  // other
  EXPECT_EQ(t[10], SemanticClass::Ground);  // grass
  EXPECT_EQ(t[18], SemanticClass::Plant);   // plant
  EXPECT_EQ(t[5], SemanticClass::Tree);     // tree
  EXPECT_EQ(t[33], SemanticClass::Structure);  // fence
  EXPECT_EQ(t[3], SemanticClass::Sky);
  EXPECT_EQ(t[13], SemanticClass::Dynamic);  // person
  EXPECT_EQ(t[16], SemanticClass::Unknown);  // table: furniture isn't a class
  std::size_t mapped = 0;
  for (const auto c : t) {
    mapped += c != SemanticClass::Unknown ? 1 : 0;
  }
  EXPECT_EQ(mapped, 45u);  // As vg_android's Ade20k.kt.
}

}  // namespace
