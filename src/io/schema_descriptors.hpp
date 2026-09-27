#pragma once

#include <string_view>

namespace vg::io::detail {

// Serialized FileDescriptorSet for a protobuf message type such as
// "foxglove.RawImage", or empty if the type is unknown.
std::string_view schema_descriptor(std::string_view name);

}  // namespace vg::io::detail
