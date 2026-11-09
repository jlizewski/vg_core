#include "vg_core/onnx_backend.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace vg {
namespace {

std::string shape_text(const std::vector<std::int64_t>& shape) {
  std::string s = "[";
  for (std::size_t i = 0; i < shape.size(); ++i) {
    s += (i > 0 ? ", " : "") + std::to_string(shape[i]);
  }
  return s + "]";
}

// Height, width, channels and layout of a 4D image-like tensor, given which
// dimensions could be the channels (3 for an RGB input).
InferenceBackend::Tensor image_tensor(const std::vector<std::int64_t>& shape, bool rgb) {
  InferenceBackend::Tensor t;
  if (shape.size() != 4 || shape[0] != 1) {
    throw std::runtime_error("onnx model: tensor " + shape_text(shape) + " is not one image");
  }
  if (rgb) {
    t.channels_first = shape[1] == 3 && shape[3] != 3;
  } else {
    // Scores: channels first if the last two dimensions are the (square or
    // not) image and the second isn't one of them.
    t.channels_first = shape[2] == shape[3] && shape[1] != shape[2];
  }
  t.channels = static_cast<int>(t.channels_first ? shape[1] : shape[3]);
  t.height = static_cast<int>(t.channels_first ? shape[2] : shape[1]);
  t.width = static_cast<int>(t.channels_first ? shape[3] : shape[2]);
  return t;
}

class OnnxBackend : public InferenceBackend {
 public:
  OnnxBackend(const std::filesystem::path& model, int threads)
      : env_(ORT_LOGGING_LEVEL_WARNING, "vg_core"), session_(nullptr) {
    Ort::SessionOptions options;
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    if (threads > 0) {
      options.SetIntraOpNumThreads(threads);
    }
    try {
      session_ = Ort::Session(env_, model.c_str(), options);
    } catch (const Ort::Exception& e) {
      throw std::runtime_error("onnx model " + model.string() + ": " + e.what());
    }
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() < 1) {
      throw std::runtime_error("onnx model " + model.string() + ": needs one input");
    }
    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_.GetInputNameAllocated(0, allocator).get();
    output_name_ = session_.GetOutputNameAllocated(0, allocator).get();

    const auto input_info = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
    input_shape_ = input_info.GetShape();
    if (std::any_of(input_shape_.begin(), input_shape_.end(),
                    [](std::int64_t d) { return d <= 0; })) {
      throw std::runtime_error("onnx model " + model.string() + ": input " +
                               shape_text(input_shape_) +
                               " has no fixed size; export it with one");
    }
    const auto type = input_info.GetElementType();
    if (type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT && type != ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
      throw std::runtime_error("onnx model " + model.string() + ": input is not float32 or uint8");
    }
    float_input_ = type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
    in_ = image_tensor(input_shape_, true);
    if (in_.channels != 3) {
      throw std::runtime_error("onnx model " + model.string() + ": input " +
                               shape_text(input_shape_) + " is not RGB");
    }

    // The output's size may only be known by running it: run once on zeros.
    std::vector<std::uint8_t> zeros(input_bytes());
    auto outputs = run_model(zeros.data());
    const auto output_info = outputs[0].GetTensorTypeAndShapeInfo();
    if (output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
      throw std::runtime_error("onnx model " + model.string() + ": output is not float scores");
    }
    output_shape_ = output_info.GetShape();
    out_ = image_tensor(output_shape_, false);
  }

  Tensor input() const override { return in_; }
  Tensor output() const override { return out_; }
  bool float_input() const override { return float_input_; }
  std::string name() const override { return "ONNX Runtime CPU"; }

  void run(const void* input, float* scores) override {
    auto outputs = run_model(input);
    const auto count = outputs[0].GetTensorTypeAndShapeInfo().GetElementCount();
    std::memcpy(scores, outputs[0].GetTensorData<float>(), count * sizeof(float));
  }

 private:
  std::size_t input_bytes() const {
    std::size_t n = 1;
    for (const auto d : input_shape_) {
      n *= static_cast<std::size_t>(d);
    }
    return n * (float_input_ ? sizeof(float) : 1);
  }

  std::vector<Ort::Value> run_model(const void* input) {
    const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    // ONNX Runtime only reads the input.
    void* data = const_cast<void*>(input);
    Ort::Value tensor =
        float_input_
            ? Ort::Value::CreateTensor<float>(memory, static_cast<float*>(data),
                                              input_bytes() / sizeof(float), input_shape_.data(),
                                              input_shape_.size())
            : Ort::Value::CreateTensor<std::uint8_t>(memory, static_cast<std::uint8_t*>(data),
                                                     input_bytes(), input_shape_.data(),
                                                     input_shape_.size());
    const char* inputs[] = {input_name_.c_str()};
    const char* outputs[] = {output_name_.c_str()};
    return session_.Run(Ort::RunOptions{nullptr}, inputs, &tensor, 1, outputs, 1);
  }

  Ort::Env env_;
  Ort::Session session_;
  std::string input_name_;
  std::string output_name_;
  std::vector<std::int64_t> input_shape_;
  std::vector<std::int64_t> output_shape_;
  bool float_input_ = true;
  Tensor in_;
  Tensor out_;
};

}  // namespace

std::unique_ptr<InferenceBackend> make_onnx_backend(const std::filesystem::path& model,
                                                    int threads) {
  if (!std::filesystem::is_regular_file(model)) {
    throw std::runtime_error("onnx model " + model.string() + " not found");
  }
  return std::make_unique<OnnxBackend>(model, threads);
}

}  // namespace vg
