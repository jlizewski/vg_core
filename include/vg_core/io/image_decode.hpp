#pragma once

// Decoding recorded camera images (CameraImage: JPEG or PNG) to RGB, e.g. to
// segment them on the desktop (vg segment).

#include <cstdint>
#include <optional>
#include <vector>

#include "vg_core/io/session.hpp"
#include "vg_core/segmentation.hpp"

namespace vg::io {

struct RgbImage {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> pixels;  // Row-major, 3 bytes (R, G, B) per pixel.

  ImageView view() const { return {pixels.data(), width, height, ImageView::Format::Rgb8}; }
};

// The image's pixels, or nullopt if it can't be decoded.
std::optional<RgbImage> decode_image(const CameraImage& image);

}  // namespace vg::io
