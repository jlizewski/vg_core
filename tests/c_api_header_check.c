/* Compiles vg_core_c.h as C, not C++, so the header stays valid C for C
 * callers and for FFI generators (Dart's ffigen, Swift) that parse it as C.
 * c_api_test.cpp calls this. */
#include "vg_core_c/vg_core_c.h"

int vg_c_header_check(void);

int vg_c_header_check(void) {
  vg_pose pose = {{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}};
  vg_depth_frame frame = {0};
  vg_map_builder_config config;
  if (vg_map_builder_config_defaults(&config) != VG_OK) {
    return 0;
  }
  (void)frame;
  return vg_api_version() == VG_C_API_VERSION && pose.orientation[3] > 0.5 &&
         config.voxel_size > 0.0;
}
