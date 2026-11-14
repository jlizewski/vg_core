# Code tour

Every module in vg_core, what it does, and where to start reading. Each
header in [include/vg_core/](../include/vg_core/) opens with a comment
explaining its module; the matching source is in [src/](../src/) and its test
in [tests/](../tests/) (`<name>_test.cpp`). The tests are often the quickest
way to see a module used.

## Suggested reading order

1. [sensor_data.hpp](../include/vg_core/sensor_data.hpp): the input types.
2. [map_builder.hpp](../include/vg_core/map_builder.hpp): how the input
   becomes a map, live.
3. [tsdf_volume.hpp](../include/vg_core/tsdf_volume.hpp): the 3D map itself.
4. [map_package.hpp](../include/vg_core/map_package.hpp): the files a walk
   produces, and the stages that make them.
5. [ground_segmentation.hpp](../include/vg_core/ground_segmentation.hpp),
   [terrain.hpp](../include/vg_core/terrain.hpp),
   [sun_map.hpp](../include/vg_core/sun_map.hpp): the main derived maps.
6. [site.hpp](../include/vg_core/site.hpp) and
   [registration.hpp](../include/vg_core/registration.hpp): many walks of one
   yard.

## Modules by area

### Input

| Header | What it does |
| --- | --- |
| [sensor_data.hpp](../include/vg_core/sensor_data.hpp) | Plain structs for sensor input: `DepthFrame`, `PoseStamped`, `ImuSample`, `GpsFix`, `CameraIntrinsics`. No file format attached. |
| [depth_filter.hpp](../include/vg_core/depth_filter.hpp) | Drops unreliable depth: out of range, on depth edges ("flying pixels"), surfaces seen edge-on. |
| [pose_jumps.hpp](../include/vg_core/pose_jumps.hpp) | `PoseJumpFilter` undoes the tracker's sudden drift corrections so a walk stays in one frame. |

### Building the 3D map

See [pipeline/building.md](pipeline/building.md).

| Header | What it does |
| --- | --- |
| [map_builder.hpp](../include/vg_core/map_builder.hpp) | `MapBuilder`: the live entry point. Pairs depth with poses, picks keyframes, defers the rest, reports what changed for a live view. |
| [tsdf_volume.hpp](../include/vg_core/tsdf_volume.hpp) | `TsdfVolume`: sparse voxel-hashed TSDF, fusion, occupied voxels, label votes. |
| [depth_spool.hpp](../include/vg_core/depth_spool.hpp) | `DepthSpool`: a FIFO of deferred depth frames, in memory or on disk. |
| [live_height_map.hpp](../include/vg_core/live_height_map.hpp) | `LiveHeightMap`: a 2.5D height map kept up to date block by block for a live view. |
| [height_map.hpp](../include/vg_core/height_map.hpp) | `HeightMap`: the 2D grid type every derived map uses. |
| [voxel_cleanup.hpp](../include/vg_core/voxel_cleanup.hpp) | Removes specks (small disconnected voxel groups). |

### Derived maps

See [pipeline/derived-maps.md](pipeline/derived-maps.md).

| Header | What it does |
| --- | --- |
| [map_package.hpp](../include/vg_core/map_package.hpp) | `MapPackage`: one walk's directory and its stages (`make_surface`, `make_ground`, `make_terrain`, `make_sky`, `make_sun`, ...). The manifest records what each stage ran with. |
| [ground_segmentation.hpp](../include/vg_core/ground_segmentation.hpp) | `segment_ground`: splits voxels into ground and segments, and patches the ground map. |
| [terrain.hpp](../include/vg_core/terrain.hpp) | `fit_terrain`: a smooth ground contour from seen ground and the walked path (weighted least squares). |
| [diagnostics.hpp](../include/vg_core/diagnostics.hpp) | Pose-track and depth statistics, and the ghost-surface ("layers") map. |

### Sun and sky

See [pipeline/sun-and-sky.md](pipeline/sun-and-sky.md).

| Header | What it does |
| --- | --- |
| [geo_reference.hpp](../include/vg_core/geo_reference.hpp) | Places a walk's world frame on the Earth from GPS and poses. |
| [solar.hpp](../include/vg_core/solar.hpp) | Sun position (NOAA algorithm) and clear-sky direct irradiance. |
| [sun_map.hpp](../include/vg_core/sun_map.hpp) | Sun hours and solar energy per ground cell, by ray casting through the voxels. |
| [sky.hpp](../include/vg_core/sky.hpp) | Sky evidence from labeled frames, and the canopy beyond depth's reach. |

### Semantic labels

See [pipeline/semantics.md](pipeline/semantics.md).

| Header | What it does |
| --- | --- |
| [semantics.hpp](../include/vg_core/semantics.hpp) | The seven classes and class masks. |
| [segmentation.hpp](../include/vg_core/segmentation.hpp) | `Segmenter`: image → class per depth pixel, the same on every platform. `InferenceBackend` is the one platform-specific piece. |
| [onnx_backend.hpp](../include/vg_core/onnx_backend.hpp) | ONNX Runtime backend (separate `vg_core_onnx` target), for labeling on a PC. |
| [label_consistency.hpp](../include/vg_core/label_consistency.hpp) | Scores how steady a model's labels are against the map's votes, with no ground truth. |

### Sites

See [pipeline/sites.md](pipeline/sites.md).

| Header | What it does |
| --- | --- |
| [registration.hpp](../include/vg_core/registration.hpp) | `register_map`: lines one map up with another (FFT coarse search, then ICP). `MapTransform`. |
| [site.hpp](../include/vg_core/site.hpp) | `Site`: the visits, adding one, and making the current maps and changes. |

### Files

See [formats.md](formats.md).

| Header | What it does |
| --- | --- |
| [volume_io.hpp](../include/vg_core/volume_io.hpp) | Saving and loading the 3D map (`.vgm`) with its metadata. |
| [height_map_io.hpp](../include/vg_core/height_map_io.hpp) | ESRI ASCII grids (`.asc`). |
| [voxel_map_io.hpp](../include/vg_core/voxel_map_io.hpp) | PLY export of voxels. |
| [version.hpp](../include/vg_core/version.hpp) | Library version. |

### Recording and replay (`vg_core_io`)

In [include/vg_core/io/](../include/vg_core/io/). See [capture-format.md](capture-format.md).

| Header | What it does |
| --- | --- |
| `io/session.hpp` | Topic names and message types of a capture session. |
| `io/session_writer.hpp`, `io/session_reader.hpp` | Write and read recordings. |
| `io/session_player.hpp` | `play()`: replay a recording at recorded speed, scaled, or as fast as possible. |
| `io/map_build.hpp` | `build_map_package`: replay a recording into a new map package (the build stage). |
| `io/mcap_repair.hpp` | Recover a recording the app never closed. |
| `io/image_decode.hpp` | Decode JPEG/PNG camera images. |

### Settings (`vg_core_config`)

In [include/vg_core/config/](../include/vg_core/config/).

| Header | What it does |
| --- | --- |
| `config/mapping_config.hpp` | `MappingConfig`: every pipeline setting in one struct; the YAML sections. |
| `config/params.hpp` | Binding settings to YAML paths and command-line flags. |
| `config/jobs.hpp` | The pipeline jobs as functions: `replay`, `rerun_stages`, `add_to_site`, `make_site_maps`. The command-line apps and the C API both call these. |

[config/mapping.yaml](../config/mapping.yaml) lists every setting with its
default and an explanation. It's generated (`vg config defaults`); regenerate
it after adding a setting.

### C API (`vg_core_c`)

[include/vg_core_c/vg_core_c.h](../include/vg_core_c/vg_core_c.h): the one
header apps in other languages bind to, implemented in
[src/c_api/](../src/c_api/). See [c-api.md](c-api.md).

## Command-line apps

In [apps/](../apps/); run as `vg <name>` through the Python CLI.

| App | What it does |
| --- | --- |
| `vg replay` | Recording → full map package. |
| `vg map` | Re-run stages of a package from its saved 3D map. |
| `vg site` | Add walks to a site, make its current maps, list visits. |
| `vg segment` | Relabel a recording's depth with a model on the PC. |
| `vg mcap stats` | What a recording holds, and how good its poses and depth look. |
| `vg mcap repair` | Index an unclosed recording. |
| `vg config` | Show or check settings. |

## Python tools

In [tools/vg_tools/](../tools/vg_tools/): the `vg` CLI ([cli.py](../tools/vg_tools/cli.py)),
heatmap rendering, label comparison, model export, path statistics, and the
browser dashboard for replays, sites and maps. See the README's
[Development tools](../README.md#development-tools-python--310).
