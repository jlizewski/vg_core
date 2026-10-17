#pragma once

// An InferenceBackend running ONNX models with ONNX Runtime on the CPU: the
// desktop's segmentation runtime (vg segment). Built as vg_core_onnx, apart
// from vg_core, so nothing else needs ONNX Runtime.
//
// The model needs one image input with a fixed size, NHWC or NCHW, float32 or
// uint8, and one output of per-class scores (vg models export makes them).

#include <filesystem>
#include <memory>

#include "vg_core/segmentation.hpp"

namespace vg {

// Loads the model. threads: CPU threads for inference, 0 for ONNX Runtime's
// default. Throws std::runtime_error if it can't be loaded or isn't one image
// in, scores out.
std::unique_ptr<InferenceBackend> make_onnx_backend(const std::filesystem::path& model,
                                                    int threads = 0);

}  // namespace vg
