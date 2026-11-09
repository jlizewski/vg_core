#include "vg_core/io/image_decode.hpp"

#include <cstring>
#include <limits>

// stb_image, compiled here only, for the formats recordings use.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#if defined(_MSC_VER)
#pragma warning(push, 0)
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif
#include <stb_image.h>
#if defined(_MSC_VER)
#pragma warning(pop)
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

namespace vg::io {

std::optional<RgbImage> decode_image(const CameraImage& image) {
  if (image.data.empty() ||
      image.data.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }
  int width = 0;
  int height = 0;
  int channels = 0;
  stbi_uc* pixels = stbi_load_from_memory(image.data.data(), static_cast<int>(image.data.size()),
                                          &width, &height, &channels, 3);
  if (pixels == nullptr) {
    return std::nullopt;
  }
  RgbImage rgb;
  rgb.width = width;
  rgb.height = height;
  rgb.pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3);
  std::memcpy(rgb.pixels.data(), pixels, rgb.pixels.size());
  stbi_image_free(pixels);
  return rgb;
}

}  // namespace vg::io
