#include "vg_core/version.hpp"

#include "vg_core/version_config.hpp"

namespace vg {

Version version() noexcept {
  return {VG_CORE_VERSION_MAJOR, VG_CORE_VERSION_MINOR, VG_CORE_VERSION_PATCH};
}

std::string_view version_string() noexcept { return VG_CORE_VERSION_STRING; }

}  // namespace vg
