#include "vg_core/segmentation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vg {
namespace {

// Normalized coordinates: (s, t) across and down the camera image; (a, b)
// across and down the upright image, which is the camera image turned
// clockwise by the rotation.
float camera_s(int rotation, float a, float b) {
  switch (rotation) {
    case 90: return b;
    case 180: return 1.0f - a;
    case 270: return 1.0f - b;
    default: return a;
  }
}

float camera_t(int rotation, float a, float b) {
  switch (rotation) {
    case 90: return 1.0f - a;
    case 180: return 1.0f - b;
    case 270: return a;
    default: return b;
  }
}

float upright_a(int rotation, float s, float t) {
  switch (rotation) {
    case 90: return 1.0f - t;
    case 180: return 1.0f - s;
    case 270: return t;
    default: return s;
  }
}

float upright_b(int rotation, float s, float t) {
  switch (rotation) {
    case 90: return s;
    case 180: return 1.0f - t;
    case 270: return 1.0f - s;
    default: return t;
  }
}

double ms_since(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
      .count();
}

// RGB (0-255) of camera pixel (x, y).
std::array<float, 3> rgb_at(const ImageView& image, int x, int y) {
  if (image.format == ImageView::Format::Rgb8) {
    const auto stride =
        static_cast<std::size_t>(image.row_stride > 0 ? image.row_stride : image.width * 3);
    const std::uint8_t* p =
        image.data + static_cast<std::size_t>(y) * stride + static_cast<std::size_t>(x) * 3;
    return {static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])};
  }
  // NV21: BT.601 full range, as Android cameras produce.
  const auto stride =
      static_cast<std::size_t>(image.row_stride > 0 ? image.row_stride : image.width);
  const std::uint8_t* chroma_plane = image.data + stride * static_cast<std::size_t>(image.height);
  const auto luma =
      static_cast<float>(image.data[static_cast<std::size_t>(y) * stride +
                                    static_cast<std::size_t>(x)]);
  const std::uint8_t* vu = chroma_plane + static_cast<std::size_t>(y / 2) * stride +
                           static_cast<std::size_t>(x / 2) * 2;
  const float v = static_cast<float>(vu[0]) - 128.0f;
  const float u = static_cast<float>(vu[1]) - 128.0f;
  return {std::clamp(luma + 1.402f * v, 0.0f, 255.0f),
          std::clamp(luma - 0.344136f * u - 0.714136f * v, 0.0f, 255.0f),
          std::clamp(luma + 1.772f * u, 0.0f, 255.0f)};
}

constexpr SemanticClass G = SemanticClass::Ground;
constexpr SemanticClass P = SemanticClass::Plant;
constexpr SemanticClass T = SemanticClass::Tree;
constexpr SemanticClass S = SemanticClass::Structure;
constexpr SemanticClass K = SemanticClass::Sky;
constexpr SemanticClass D = SemanticClass::Dynamic;

}  // namespace

const std::array<SemanticClass, 151>& ade20k_classes() {
  static const std::array<SemanticClass, 151> table = [] {
    std::array<SemanticClass, 151> t{};  // All Unknown.
    const std::pair<int, SemanticClass> mapping[] = {
        // Ground.
        {4, G},   // floor
        {7, G},   // road
        {10, G},  // grass
        {12, G},  // sidewalk
        {14, G},  // earth
        {17, G},  // mountain
        {30, G},  // field
        {47, G},  // sand
        {53, G},  // path
        {55, G},  // runway
        {69, G},  // hill
        {92, G},  // dirt track
        {95, G},  // land
        // Plants.
        {18, P},  // plant
        {67, P},  // flower
        // Trees.
        {5, T},   // tree
        {73, T},  // palm
        // Structures.
        {1, S},    // wall
        {2, S},    // building
        {6, S},    // ceiling, e.g. of a porch
        {9, S},    // windowpane
        {15, S},   // door
        {26, S},   // house
        {33, S},   // fence
        {39, S},   // railing
        {43, S},   // column
        {49, S},   // skyscraper
        {52, S},   // grandstand
        {54, S},   // stairs
        {59, S},   // screen door
        {60, S},   // stairway
        {62, S},   // bridge
        {80, S},   // hovel
        {85, S},   // tower
        {87, S},   // awning
        {88, S},   // streetlight
        {94, S},   // pole
        {96, S},   // bannister
        {107, S},  // canopy
        {115, S},  // tent
        {122, S},  // step
        {141, S},  // pier
        // Sky.
        {3, K},
        // Dynamic.
        {13, D},   // person
        {127, D},  // animal
    };
    for (const auto& [label, c] : mapping) {
      t[static_cast<std::size_t>(label)] = c;
    }
    return t;
  }();
  return table;
}

SegmentationModelInfo deeplabv3_mnv2_ade20k() {
  SegmentationModelInfo m;
  m.name = "deeplabv3_mnv2_ade20k";
  m.file = "deeplabv3_mnv2_ade20k.tflite";
  m.mean = {127.5f, 127.5f, 127.5f};
  m.std = {127.5f, 127.5f, 127.5f};
  m.classes.assign(ade20k_classes().begin(), ade20k_classes().end());
  return m;
}

SegmentationModelInfo segformer_b0_ade20k() {
  SegmentationModelInfo m;
  m.name = "segformer_b0_ade20k";
  m.file = "segformer_b0_ade20k.tflite";
  m.mean = {123.675f, 116.28f, 103.53f};
  m.std = {58.395f, 57.12f, 57.375f};
  m.classes.assign(ade20k_classes().begin(), ade20k_classes().end());
  return m;
}

Segmenter::Segmenter(std::unique_ptr<InferenceBackend> backend, SegmentationModelInfo model)
    : backend_(std::move(backend)), model_(std::move(model)) {
  if (!backend_) {
    throw std::invalid_argument("segmenter: no inference backend");
  }
  in_ = backend_->input();
  out_ = backend_->output();
  if (in_.channels != 3 || in_.width <= 0 || in_.height <= 0) {
    throw std::invalid_argument("segmenter: model input is not one RGB image");
  }
  if (out_.width <= 0 || out_.height <= 0 || out_.channels <= 0 ||
      static_cast<std::size_t>(out_.channels) > model_.classes.size()) {
    throw std::invalid_argument("segmenter: model output has " + std::to_string(out_.channels) +
                                " classes, more than " + model_.name + "'s " +
                                std::to_string(model_.classes.size()) + " labels");
  }
  class_offset_ = model_.classes.size() - static_cast<std::size_t>(out_.channels);
  const auto pixels = static_cast<std::size_t>(in_.width) * static_cast<std::size_t>(in_.height);
  if (backend_->float_input()) {
    input_floats_.resize(pixels * 3);
  } else {
    input_bytes_.resize(pixels * 3);
  }
  const auto cells = static_cast<std::size_t>(out_.width) * static_cast<std::size_t>(out_.height);
  scores_.resize(cells * static_cast<std::size_t>(out_.channels));
  cell_classes_.resize(cells);
}

Segmenter::Result Segmenter::segment(const ImageView& image, int rotation_cw, int depth_width,
                                     int depth_height) {
  if (image.data == nullptr || image.width <= 0 || image.height <= 0) {
    throw std::invalid_argument("segmenter: empty image");
  }
  if (rotation_cw != 0 && rotation_cw != 90 && rotation_cw != 180 && rotation_cw != 270) {
    throw std::invalid_argument("segmenter: rotation must be 0, 90, 180 or 270");
  }
  Result result;
  const auto start = std::chrono::steady_clock::now();
  fill_input(image, rotation_cw);
  result.preprocess_ms = ms_since(start);

  const auto inference_start = std::chrono::steady_clock::now();
  backend_->run(backend_->float_input() ? static_cast<const void*>(input_floats_.data())
                                        : static_cast<const void*>(input_bytes_.data()),
                scores_.data());
  result.inference_ms = ms_since(inference_start);

  const auto post_start = std::chrono::steady_clock::now();
  classify_cells();
  result.labels = shrink_to_depth(rotation_cw, depth_width, depth_height, result.counts);
  result.postprocess_ms = ms_since(post_start);
  return result;
}

void Segmenter::fill_input(const ImageView& image, int rotation) {
  const auto pixels = static_cast<std::size_t>(in_.width) * static_cast<std::size_t>(in_.height);
  const bool as_float = backend_->float_input();
  for (int my = 0; my < in_.height; ++my) {
    const float b = (static_cast<float>(my) + 0.5f) / static_cast<float>(in_.height);
    for (int mx = 0; mx < in_.width; ++mx) {
      const float a = (static_cast<float>(mx) + 0.5f) / static_cast<float>(in_.width);
      const int x = std::clamp(
          static_cast<int>(camera_s(rotation, a, b) * static_cast<float>(image.width)), 0,
          image.width - 1);
      const int y = std::clamp(
          static_cast<int>(camera_t(rotation, a, b) * static_cast<float>(image.height)), 0,
          image.height - 1);
      const std::array<float, 3> rgb = rgb_at(image, x, y);
      const auto i = static_cast<std::size_t>(my) * static_cast<std::size_t>(in_.width) +
                     static_cast<std::size_t>(mx);
      for (std::size_t c = 0; c < 3; ++c) {
        const std::size_t o = in_.channels_first ? c * pixels + i : 3 * i + c;
        if (as_float) {
          input_floats_[o] = (rgb[c] - model_.mean[c]) / model_.std[c];
        } else {
          input_bytes_[o] = static_cast<std::uint8_t>(std::lround(rgb[c]));
        }
      }
    }
  }
}

void Segmenter::classify_cells() {
  const auto cells = static_cast<std::size_t>(out_.width) * static_cast<std::size_t>(out_.height);
  const auto classes = static_cast<std::size_t>(out_.channels);
  for (std::size_t cell = 0; cell < cells; ++cell) {
    std::size_t best = 0;
    float best_score = -std::numeric_limits<float>::infinity();
    for (std::size_t c = 0; c < classes; ++c) {
      const float score = out_.channels_first ? scores_[c * cells + cell]
                                              : scores_[cell * classes + c];
      if (score > best_score) {
        best_score = score;
        best = c;
      }
    }
    cell_classes_[cell] = static_cast<std::uint8_t>(model_.classes[best + class_offset_]);
  }
}

std::vector<std::uint8_t> Segmenter::shrink_to_depth(
    int rotation, int width, int height, std::array<std::size_t, kSemanticClasses>& counts) {
  std::vector<std::uint8_t> labels(static_cast<std::size_t>(width) *
                                   static_cast<std::size_t>(height));
  std::array<std::size_t, kSemanticClasses> votes{};
  const auto out_w = static_cast<float>(out_.width);
  const auto out_h = static_cast<float>(out_.height);
  for (int dy = 0; dy < height; ++dy) {
    const float t0 = static_cast<float>(dy) / static_cast<float>(height);
    const float t1 = static_cast<float>(dy + 1) / static_cast<float>(height);
    for (int dx = 0; dx < width; ++dx) {
      const float s0 = static_cast<float>(dx) / static_cast<float>(width);
      const float s1 = static_cast<float>(dx + 1) / static_cast<float>(width);
      // The pixel's footprint in upright coordinates; rotating keeps it a rectangle.
      const float a0 = std::min(upright_a(rotation, s0, t0), upright_a(rotation, s1, t1));
      const float a1 = std::max(upright_a(rotation, s0, t0), upright_a(rotation, s1, t1));
      const float b0 = std::min(upright_b(rotation, s0, t0), upright_b(rotation, s1, t1));
      const float b1 = std::max(upright_b(rotation, s0, t0), upright_b(rotation, s1, t1));
      const int x0 = std::clamp(static_cast<int>(std::floor(a0 * out_w)), 0, out_.width - 1);
      const int x1 = std::clamp(static_cast<int>(std::ceil(a1 * out_w)), x0 + 1, out_.width);
      const int y0 = std::clamp(static_cast<int>(std::floor(b0 * out_h)), 0, out_.height - 1);
      const int y1 = std::clamp(static_cast<int>(std::ceil(b1 * out_h)), y0 + 1, out_.height);
      std::uint8_t label = 0;
      if (x1 - x0 == 1 && y1 - y0 == 1) {
        label = cell_classes_[static_cast<std::size_t>(y0) * static_cast<std::size_t>(out_.width) +
                              static_cast<std::size_t>(x0)];
      } else {
        votes.fill(0);
        for (int y = y0; y < y1; ++y) {
          for (int x = x0; x < x1; ++x) {
            ++votes[cell_classes_[static_cast<std::size_t>(y) *
                                      static_cast<std::size_t>(out_.width) +
                                  static_cast<std::size_t>(x)]];
          }
        }
        std::size_t best = 0;
        for (std::size_t c = 1; c < votes.size(); ++c) {
          if (votes[c] > votes[best]) {
            best = c;
          }
        }
        label = static_cast<std::uint8_t>(best);
      }
      labels[static_cast<std::size_t>(dy) * static_cast<std::size_t>(width) +
             static_cast<std::size_t>(dx)] = label;
      ++counts[label];
    }
  }
  return labels;
}

}  // namespace vg
