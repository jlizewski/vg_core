#pragma once

#include <string_view>

namespace vg {

struct Version {
  int major;
  int minor;
  int patch;
};

// Version of the vg_core library this binary was built against.
Version version() noexcept;

// Version as "major.minor.patch".
std::string_view version_string() noexcept;

}  // namespace vg
