#pragma once

// Semantic segmentation of camera images into vg's classes (semantics.hpp),
// for labeling depth: the same steps on every platform (Android, iOS, a ROS2
// node, the PC), so only running the model itself differs.
//
// A Segmenter takes a camera image and the size of the depth image taken with
// it, and returns a class per depth pixel:
//   1. The image is turned upright (models are trained on upright photos;
//      phone cameras deliver the sensor's orientation), stretched to the
//      model's input size and normalized (SegmentationModelInfo).
//   2. An InferenceBackend runs the model: LiteRT, ONNX Runtime, ... Each is
//      its own small library, so vg_core needs none of them.
//   3. Each output cell takes its best-scoring class, mapped to a vg class
//      (e.g. ADE20K's 150 through ade20k_classes()).
//   4. The cells are shrunk to the depth image, in the camera's orientation:
//      each depth pixel takes the class most of the cells it covers have. The
//      depth image covers the same field of view as the camera image.
//
// See the ship-time TODO in semantics.hpp: the models here were chosen to get
// started.

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "vg_core/semantics.hpp"

namespace vg {

// A camera image in memory.
struct ImageView {
  enum class Format {
    Nv21,  // Android camera YUV: full-res Y plane, then interleaved V/U at half res.
    Rgb8,  // 3 bytes per pixel, e.g. a decoded JPEG.
  };
  const std::uint8_t* data = nullptr;
  int width = 0;
  int height = 0;
  Format format = Format::Rgb8;
  // Bytes from one row to the next (Rgb8; the Y plane for Nv21); 0 = packed.
  int row_stride = 0;
};

// What a model needs beyond its file: how to normalize its input, and how its
// outputs map to vg classes.
struct SegmentationModelInfo {
  std::string name;  // e.g. "deeplabv3_mnv2_ade20k"
  std::string file;  // e.g. "deeplabv3_mnv2_ade20k.tflite"
  // Each RGB value v (0-255) is fed as (v - mean) / std.
  std::array<float, 3> mean{0.0f, 0.0f, 0.0f};
  std::array<float, 3> std{1.0f, 1.0f, 1.0f};
  // vg class of each of the model's label ids. A model with fewer outputs
  // than labels skips the first ones: with 150 outputs and 151 labels (as
  // ADE20K's "other" plus 150 classes), output 0 is label 1.
  std::vector<SemanticClass> classes;
};

// vg class of each ADE20K label: 0 "other", then 1 (wall) to 150 (flag), as
// numbered in ADE20K's objectInfo150. Indoor things, furniture, vehicles and
// water are Unknown.
const std::array<SemanticClass, 151>& ade20k_classes();

// TensorFlow's DeepLab model zoo deeplabv3_mnv2_ade20k (513 x 513 in, values
// scaled to -1..1, 151 outputs).
SegmentationModelInfo deeplabv3_mnv2_ade20k();
// NVIDIA's segformer-b0-finetuned-ade-512-512 (512 x 512 in, ImageNet
// normalization, 150 outputs). Research / non-commercial license.
SegmentationModelInfo segformer_b0_ade20k();

// Runs a model: the one part that differs per platform and runtime.
class InferenceBackend {
 public:
  struct Tensor {
    int height = 0;
    int width = 0;
    int channels = 0;  // 3 (RGB) for the input; classes for the output.
    bool channels_first = false;  // NCHW rather than NHWC.
  };

  virtual ~InferenceBackend() = default;

  virtual Tensor input() const = 0;
  virtual Tensor output() const = 0;
  // Whether the input is float32 (normalized); otherwise uint8 (0-255).
  virtual bool float_input() const = 0;
  // Runs the model on input (input() floats or bytes, in its layout) and
  // writes output() scores, in its layout.
  virtual void run(const void* input, float* scores) = 0;
  // Where it runs, e.g. "LiteRT GPU" or "ONNX Runtime CPU".
  virtual std::string name() const = 0;
};

class Segmenter {
 public:
  // Throws std::invalid_argument if the backend's tensors don't fit the model
  // (not one RGB image in, or a different number of classes out).
  Segmenter(std::unique_ptr<InferenceBackend> backend, SegmentationModelInfo model);

  struct Result {
    // One SemanticClass value per depth pixel, row-major, in the camera's
    // orientation (as the depth image is).
    std::vector<std::uint8_t> labels;
    std::array<std::size_t, kSemanticClasses> counts{};  // Depth pixels per class.
    double preprocess_ms = 0.0;
    double inference_ms = 0.0;
    double postprocess_ms = 0.0;
  };

  // Labels a depth_width x depth_height depth image from the camera image
  // taken with it. rotation_cw (0, 90, 180 or 270) is how far to turn the
  // camera image clockwise to make it upright.
  Result segment(const ImageView& image, int rotation_cw, int depth_width, int depth_height);

  const InferenceBackend& backend() const { return *backend_; }
  const SegmentationModelInfo& model() const { return model_; }

 private:
  void fill_input(const ImageView& image, int rotation_cw);
  void classify_cells();
  std::vector<std::uint8_t> shrink_to_depth(int rotation_cw, int width, int height,
                                            std::array<std::size_t, kSemanticClasses>& counts);

  std::unique_ptr<InferenceBackend> backend_;
  SegmentationModelInfo model_;
  InferenceBackend::Tensor in_;
  InferenceBackend::Tensor out_;
  std::size_t class_offset_ = 0;  // Label id of output 0.
  std::vector<float> input_floats_;
  std::vector<std::uint8_t> input_bytes_;
  std::vector<float> scores_;
  std::vector<std::uint8_t> cell_classes_;  // vg class per output cell, upright.
};

}  // namespace vg
