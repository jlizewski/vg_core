# vg_core

Core Victory Garden software and tools.

vg_core is the platform-agnostic C++ core of Victory Garden. It builds a map of a
garden from sensor input (depth, IMU, GPS) supplied by a host app such as
[vg_android](https://github.com/jlizewski/vg_android), and will later track
entities in that map (growth, sunshine, temperature, wind) and compute a sun map.
Host apps only provide an interface; all the logic lives here so it can be reused
on any platform.

## Layout

| Path | Contents |
| --- | --- |
| `include/vg_core/` | Public C++ headers (namespace `vg`) |
| `src/` | Library implementation |
| `tests/` | C++ unit tests (GoogleTest) |
| `cmake/` | CMake helper modules |
| `src/io/`, `include/vg_core/io/` | `vg_core_io`: capture session recording and playback (MCAP) |
| `apps/` | Command-line apps (`vg_replay`) |
| `tools/` | Python development tools (`vg` CLI) |
| `docs/` | Design docs, including the [capture format](docs/capture-format.md) |
| `schemas/` | Protobuf schemas for vg-specific capture messages |

## Building (C++17, CMake ≥ 3.20)

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Eigen 3.4 and GoogleTest are used from the system if found, otherwise fetched at
configure time. `vg_core_io` also fetches zstd and the MCAP C++ library; turn it
off with `-DVG_CORE_BUILD_IO=OFF` to build only the mapping library.

## Live mapping

`vg::MapBuilder` builds the map as data arrives, on device or from a replay:

```cpp
vg::MapBuilder builder;
builder.set_camera_extrinsics("rgb", body_from_camera);  // once
builder.add_pose(pose);                                  // every tracker pose
builder.add_depth("rgb", depth_frame);                   // every depth frame
// A few times a second, e.g. on the mapping thread before handing off to the UI:
const vg::MapBuilder::Update update = builder.update();
for (const vg::HeightCell& cell : update.height_cells) {
  redraw(cell.x, cell.y, cell.height);  // NaN height: cell cleared
}
for (const Eigen::Vector3i& block : update.blocks) {
  redraw_voxels(block, builder.volume().occupied_voxels(block));  // 3D view
}
```

`update()` only re-reads the parts of the map that changed since the last
call. The height map is a 2.5D view; the 3D view is the occupied voxels, the
solid voxels at the observed surface, as integer indices (multiply by the
voxel size for meters). `builder.height_map()` and
`builder.volume().occupied_voxels()` give the whole map, e.g. to export.

### Keyframes and finishing later

Fusing every depth frame live is more than a phone needs to show what has been
covered. With keyframing on, only frames taken after the camera has moved or
turned far enough are fused live; the rest are deferred, with the pose they
were paired with, to a spool file. Once recording stops, fuse them in batches
(on a background thread, say) so the finished map holds every frame:

```cpp
vg::MapBuilderConfig config;
config.keyframe_translation = 0.15;           // meters
config.keyframe_rotation = 10.0 * kDegree;    // radians
config.deferred_path = cache_dir + "/map.spool";  // or empty: keep in memory
vg::MapBuilder builder(config);
// ... live: add_pose / add_depth / update as above. While the mapping thread
// has a backlog, builder.set_live(false) defers every frame instead.
builder.flush();
while (builder.integrate_deferred(10) > 0) {
  builder.update();  // report progress
}
```

## Separating the ground

`vg::segment_ground()` splits the 3D voxel map into the ground and everything
on or over it, so the ground can be mapped on its own and the rest classified
later:

```cpp
const vg::GroundSegmentation split = vg::segment_ground(builder.volume());
split.ground.heights;   // 2.5D ground height map, holes under objects patched
split.ground.filled;    // which cells were patched rather than seen
for (const vg::MapSegment& segment : split.segments) {  // largest first
  segment.voxels;               // a connected object, structure or overhang
  segment.grounded;             // stands on the ground (false: overhang)
  segment.top_above_ground;     // height in meters, e.g. of a plant
}
```

The ground is the largest stretch of the map's lowest surfaces that slopes no
more steeply than `GroundConfig::max_slope`; raised bed tops, decks and
canopies are kept out because they drop off steeply to the ground around
them. Holes in the ground map where objects stood, or small unseen holes
enclosed by ground, are filled by smooth interpolation from the ground
around them. It reprocesses the whole map, so run it occasionally (e.g. when
a scan ends) rather than on every `update()`.

## Replaying a recording

`vg_replay` rebuilds a map from a capture session
([format](docs/capture-format.md)) and writes its height map, and optionally the 3D voxel map as a PLY mesh of
cubes (opens in MeshLab, Blender or CloudCompare):

```sh
build/debug/vg_replay session.mcap garden.asc            # as fast as possible
build/debug/vg_replay session.mcap garden.asc --rate 1   # at recorded speed, printing progress
build/debug/vg_replay session.mcap garden.asc --voxel 0.01 --trunc 0.04 --cell 0.02
build/debug/vg_replay session.mcap garden.asc --voxels garden.ply  # also write 3D voxels
build/debug/vg_replay session.mcap garden.asc --ground ground.asc --objects objects.ply
```

The same session opens in [Foxglove](https://foxglove.dev) for inspection.
In code, `vg::io::SessionReader` plus `vg::io::play()` give the same playback
with a callback per message, and `vg::io::SessionMapper` feeds it into a
`vg::MapBuilder`.

To consume vg_core from another CMake project:

```cmake
add_subdirectory(vg_core)
target_link_libraries(my_app PRIVATE vg_core::vg_core)
```

Tests are off by default when vg_core is not the top-level project
(`-DVG_CORE_BUILD_TESTS=ON` to force them).

## Development tools (Python ≥ 3.10)

```sh
python -m venv .venv && source .venv/bin/activate
pip install -e "tools[dev]"

vg build            # configure + build (debug preset)
vg test             # build and run C++ tests
vg format [--check] # clang-format C++ sources
pytest tools        # test the tools themselves
vg schemas          # regenerate src/io/schema_descriptors.cpp after editing schemas/
                    # (needs: pip install -e "tools[schemas]")
```

## License

vg_core is licensed under the [Apache License 2.0](LICENSE).
