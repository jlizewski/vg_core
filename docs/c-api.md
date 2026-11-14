# C API

`vg_core_c` is vg_core's C interface, for apps written in other languages:
vg_flutter's Dart UI (through `dart:ffi`), its Kotlin and Swift capture
plugins, the desktop dev interface, and Python (through `ctypes`). It's one
header, [include/vg_core_c/vg_core_c.h](../include/vg_core_c/vg_core_c.h),
over the C++ library. Decision: [D11](decisions.md#d11-a-c-api-for-the-app-bindings).

## What it covers

| Area | Functions | Used by |
| --- | --- | --- |
| Library | `vg_api_version`, `vg_version_string`, `vg_last_error` | Everyone |
| Recording | `vg_recorder_*` | Capture plugins: write a recording in the [capture format](capture-format.md) |
| Live map | `vg_map_builder_*` | The phone's live preview while walking |
| 2D maps | `vg_grid_*` | Showing any `.asc` map: ground, sun hours, ... |
| Packages | `vg_package_*` | The 3D voxel view, where a walk is on the Earth |
| Sites | `vg_site_*` | Listing a site's walks and how they lined up |
| Pipeline jobs | `vg_replay`, `vg_map_stages`, `vg_site_add`, `vg_site_make_maps` | Building maps after a walk; re-tuning them |

The jobs are the same code the command-line apps run: `vg replay`, `vg map`
and `vg site` call the functions in
[config/jobs.hpp](../include/vg_core/config/jobs.hpp), and so does the C API.
A job run from an app gives the same result as the same job on the command
line.

Not covered yet, and added as the apps need them: segmentation (it needs a
callback-based `InferenceBackend`), the garden model, and cancelling or
reporting the progress of a job beyond its log.

## Rules

These hold for every function. The header repeats them at the top.

**Errors.** Every function that can fail returns a `vg_status`: `VG_OK`
(0), or one of the following. `vg_last_error()` then gives the message, on
the same thread, until the next call.

| Status | Meaning |
| --- | --- |
| `VG_ERROR_INVALID_ARGUMENT` | A NULL pointer, bad size or bad value from the caller |
| `VG_ERROR_CONFIG` | A setting that doesn't exist or doesn't parse |
| `VG_ERROR_FAILED` | The work failed: a file couldn't be read or written, a recording was bad |
| `VG_ERROR_INTERNAL` | A bug, or out of memory |

No C++ exception ever crosses the boundary; each function catches
everything and turns it into a status.

**Handles.** Objects are opaque pointers: `*_open` or `*_new` makes one,
`*_free` releases it (and accepts NULL). A handle may be used from any
thread, but by one thread at a time. The exception is `vg_recorder`, which
locks around each write, because capture plugins write from several threads
(camera, IMU, location).

**Memory.** What you pass in is copied, so you keep ownership. Arrays a
function returns belong to the handle and stay valid until the next call on
that handle that returns the same kind of array, or until the handle is
freed. Copy what you keep.

**Types.** Strings and paths are UTF-8. Units are SI. Times are int64
nanoseconds on the recording's monotonic clock. Booleans are `int32_t`.
Poses are a unit quaternion `(x, y, z, w)` plus a position, and take points
from the child frame to the parent frame (`world_from_body`). Frames are as
in the [capture format](capture-format.md#conventions).

**Versions.** `VG_C_API_VERSION` in the header must equal
`vg_api_version()` from the library. Check it at start-up. The number goes up
whenever a declaration changes in a way that breaks existing callers.

## Examples

### Recording a walk (capture plugin)

```c
vg_session_info info = {0};
info.producer = "vg_flutter 0.1.0";
info.platform = "arcore";
info.start_time_ns = now_ns;
info.utc_offset_ns = utc_ns - now_ns;

vg_recorder* rec = NULL;
if (vg_recorder_open(path, &info, &rec) != VG_OK) {
  log_error(vg_last_error());
  return;
}
vg_recorder_write_calibration(rec, t, "rgb", &intrinsics);

// Each frame, from whichever thread has it:
vg_recorder_write_pose(rec, t, &world_from_body);
vg_depth_frame depth = {0};
depth.timestamp_ns = t;
depth.intrinsics = depth_intrinsics;
depth.depth_mm = arcore_depth;        // or depth_m for float meters
depth.confidence = arcore_confidence; // optional
vg_recorder_write_depth(rec, "rgb", &depth);

vg_recorder_close(rec);  // writes the index; check its status
vg_recorder_free(rec);
```

### Live preview while walking

```c
vg_map_builder_config config;
vg_map_builder_config_defaults(&config);  // vg replay's defaults
config.keyframe_translation = 0.15;       // fuse keyframes live, defer the rest
config.keyframe_rotation = 0.17;
vg_map_builder* map = NULL;
vg_map_builder_new(&config, &map);

// Per frame:
vg_map_builder_add_pose(map, t, &world_from_body);
vg_map_builder_add_depth(map, "rgb", &depth);

// A few times a second, on the mapping thread:
vg_map_update update;
vg_map_builder_update(map, &update);
for (size_t i = 0; i < update.cell_count; ++i) {
  redraw_cell(update.cells[i].x, update.cells[i].y, update.cells[i].height);  // NaN: cleared
}

// After the walk:
vg_map_builder_flush(map);
size_t left = 1;
while (left > 0) vg_map_builder_integrate_deferred(map, 10, &left);
```

### Building and showing maps after a walk

```c
const char* settings[] = {"--set", "sun.year=2027"};
int32_t complete = 0;
if (vg_replay(recording, package_dir, settings, 2, on_log_line, ctx, &complete) != VG_OK) {
  show_error(vg_last_error());
}
// complete == 0: a stage was skipped (the log says why, e.g. no GPS for the sun)

vg_grid* sun = NULL;
vg_grid_load("garden/sun_hours.asc", &sun);
vg_grid_info info;
vg_grid_get_info(sun, &info);
const float* hours = vg_grid_values(sun);  // row y = 0 is the bottom (south when x is east)
float lo, hi;
vg_grid_range(sun, &lo, &hi, NULL);        // for the color scale
vg_grid_free(sun);
```

Jobs take settings exactly as the command line does (`--config FILE`,
`--set PATH=VALUE`, or a short flag), so anything in
[config/mapping.yaml](../config/mapping.yaml) can be changed from an app.

## Binding it

### Dart (vg_flutter, the desktop dev interface)

Generate bindings with [`ffigen`](https://pub.dev/packages/ffigen) from the
header, and load the library per platform:

| Platform | Library | Notes |
| --- | --- | --- |
| Windows | `vg_core_c.dll` | Next to the app's executable |
| Linux | `libvg_core_c.so` | |
| macOS | `libvg_core_c.dylib` | |
| Android | `libvg_core_c.so` | Built by the app's Gradle CMake step, per ABI |
| iOS | Static (`-DVG_CORE_C_SHARED=OFF`), in an xcframework | `DynamicLibrary.process()` |

Jobs and file loading block, so call them on a background isolate.
Callbacks (`vg_log_fn`) run on the calling thread; from Dart, use
`NativeCallable.listener` for callbacks that may come from other threads.

### Swift and Kotlin (capture plugins)

Swift imports the header directly through a module map. Kotlin calls it
through a thin JNI layer, or the plugin's C++ calls the header directly.

### Python

`ctypes.CDLL("vg_core_c")` works for scripts and tests, but VG Home will use
pybind11 over the C++ API instead (see [architecture](architecture.md#bindings-planned)).

## Building it

`vg_core_c` is built by default (`VG_CORE_BUILD_C`), as a shared library
(`VG_CORE_C_SHARED`). Turning on the shared library builds everything it links
as position-independent code. It needs `vg_core_io` and `vg_core_config`.

```sh
cmake --preset debug
cmake --build --preset debug --target vg_core_c
```

## Where the code is

| File | Holds |
| --- | --- |
| [include/vg_core_c/vg_core_c.h](../include/vg_core_c/vg_core_c.h) | The whole API, with comments |
| [src/c_api/common.hpp](../src/c_api/common.hpp) | Exception-to-status `guard`, struct conversions, the log adapter |
| [src/c_api/library.cpp](../src/c_api/library.cpp) | Version, errors, conversions |
| [src/c_api/recorder.cpp](../src/c_api/recorder.cpp) | `vg_recorder_*` over `io::SessionWriter` |
| [src/c_api/map_builder.cpp](../src/c_api/map_builder.cpp) | `vg_map_builder_*` over `MapBuilder` and `PoseJumpFilter` |
| [src/c_api/maps.cpp](../src/c_api/maps.cpp) | Grids, packages, sites |
| [src/c_api/pipeline.cpp](../src/c_api/pipeline.cpp) | The jobs, over [config/jobs.hpp](../include/vg_core/config/jobs.hpp) |
| [tests/c_api_test.cpp](../tests/c_api_test.cpp) | Every area end to end on a synthetic yard: record, live map, replay, stages, site |
| [tests/c_api_header_check.c](../tests/c_api_header_check.c) | Compiles the header as C |

Each function follows the same shape: `guard([&] { require(...); convert;
call C++; write outputs; })`. To add one, declare it in the header with
`VG_API`, implement it in that pattern in the matching source file, add a
test, and add it to the table above.
