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
| `apps/` | Command-line apps (`vg_replay`, `vg_map`, `vg_mcap_repair`), run as `vg replay`, `vg map`, `vg mcap repair` |
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

## Sun map

`vg::compute_sun_map()` works out how much direct sun each ground cell gets
over a period, such as a whole year. At each time step it computes where the
sun is for the map's place on the Earth (NOAA solar position algorithm) and
casts a ray from every ground cell toward it through the 3D voxel map;
cells whose ray escapes are in sun.

```cpp
// Where the map is: latitude/longitude from the GPS fixes, and the world
// frame's heading from lining up the GPS track with the pose track.
const auto geo = vg::estimate_geo_reference(gps_fixes, poses);
const auto config = vg::sun_map_config_for_year(2026);  // hourly steps
const vg::SunMap sun = vg::compute_sun_map(split.ground, builder.volume().occupied_voxels(),
                                           voxel_size, *geo, config);
sun.sun_hours;       // hours of direct sun per ground cell, same layout as the ground map
sun.irradiation;     // clear-sky direct energy per cell, kWh/m^2 (slope and sun angle)
sun.daylight_hours;  // the most any cell could get
```

It assumes a clear sky and open space beyond the mapped area. The heading
from GPS is only as good as the walk was long compared with the GPS error;
`GeoReference::heading_sigma` says how well it is known. A year at hourly steps
over a 10 m x 10 m map at 5 cm cells takes about 15 s on 4 cores (release
build).

## Replaying a recording: the map package

`vg replay` turns a capture session ([format](docs/capture-format.md)) into a
**map package**: a directory with the 3D voxel map and every map derived from
it.

```sh
vg replay session.mcap garden/               # 6 in cells, 3 in voxels
vg replay session.mcap garden/ --rate 1      # at recorded speed, printing progress
vg replay session.mcap garden/ --resolution 0.1 --year 2027 --sun-step 30
```

```
garden/
  map.vgm          the 3D map (TSDF voxel volume + where it is on the Earth)
  voxels.ply       its occupied voxels as cubes (MeshLab, Blender, CloudCompare)
  surface.asc      height of the highest surface per cell (m)
  ground.asc       height of the bare ground per cell, holes under objects patched (m)
  objects.ply      everything on or over the ground
  sun_hours.asc    hours of direct sun per ground cell over the year
  sun_energy.asc   clear-sky direct solar energy per ground cell (kWh/m^2)
  labels.ply       voxels colored by semantic class (when the recording has labels)
  semantic.asc     semantic class seen from above per cell (when it has labels)
  trajectory.csv   the camera's path while mapping (t, x, y, z)
  config.yaml      the settings it was made with: edit, then re-run stages with vg map
  manifest.json    what each file is, and the settings each stage last ran with
  *.png            previews of the .asc maps
```

The 3D map is the core: everything else is derived from `map.vgm`, never from
the recording. To change how a map is derived, re-run just that stage on the
saved map with `vg map`, which takes seconds:

```sh
vg map garden/ ground sun --max-slope 0.6    # stricter ground, then its sun map
vg map garden/ sun --x-bearing 15            # fix the map's heading
vg map garden/ all --cell 0.1                # every 2D map at 10 cm
```

The stages are `voxels`, `surface`, `ground` (also writes `objects.ply`),
`sun`, which reads `ground.asc`, so re-run `sun` after `ground`, and the
diagnostic `layers`. In code they're `vg::MapPackage`
([map_package.hpp](include/vg_core/map_package.hpp)), and the build stage is
`vg::io::build_map_package()`.

### Settings

Every tunable parameter of the pipeline is a setting in YAML:
[config/mapping.yaml](config/mapping.yaml) lists them all, with their defaults
and what they do. A settings file only needs what it changes:

```yaml
# garden.yaml
build:
  depth:
    max_depth: 3.0
ground:
  max_slope: 0.6
pipeline: [voxels, surface, ground, sun, layers]
```

```sh
vg replay session.mcap garden/ --config garden.yaml   # files apply in order...
vg replay session.mcap garden/ --set sun.year=2027    # ...then --set and flags
vg config show --config garden.yaml                   # the settings that result
vg config check garden.yaml                           # catch typos and bad values
```

Common settings also have short flags (`--max-slope 0.6`; `vg replay --help`
lists them). Unknown keys are errors, with the nearest real one suggested.

Each package keeps the settings it was made with in `config.yaml`, with
automatic values (`auto`) filled in. `vg map` starts from it, so a re-run stage
keeps its earlier settings: edit a section of `garden/config.yaml` and run
`vg map garden/ ground`. `map` and `build` settings describe the saved 3D map,
so `vg map` refuses to change them; rebuild with
`vg replay session.mcap garden/ --config garden/config.yaml`.

The settings live in `vg_core_config` (yaml-cpp), kept out of `vg_core` so the
mapping library stays free of it. To add one, bind its field in
[mapping_config.cpp](src/config/mapping_config.cpp) and regenerate the
defaults file: `vg config defaults > config/mapping.yaml` (a test checks it's
current).

### What the build does

`map.resolution` (default 6 in, 0.1524 m) is the cell size of the 2D maps; the
voxels are half that (`build.voxel_size`) so thin things like stems and posts
still show up. Phone depth is noisy, so the build cleans it up:

- **Depth filtering** (`build.depth`): readings beyond 3.5 m, on depth edges
  where they smear between two surfaces, and of surfaces seen nearly edge-on
  are dropped.
- **Weighted fusion** (`build.fusion`): each reading counts by its
  confidence, 1/range² and viewing angle, and a voxel needs a minimum fused
  weight to be mapped (`voxels.min_weight`, re-tunable with `vg map`). The
  default suits the depth: 0.05 for raw depth with confidence (most of
  ARCore's raw readings are low confidence, so weigh little), 0.5 for
  smoothed depth without it.
- **Free-space carving** (`build.fusion.carve`): space a frame sees through is
  marked empty, so a blob from a bad reading fades out.
- **Tracking jumps** (`build.pose_jumps`): when ARCore snaps its pose to
  correct drift, the rest of the walk is moved back into line; otherwise the
  ground before and after a snap maps as two ghost layers.
- **Specks** (`voxels.min_blob_volume`): groups of voxels on their own smaller
  than 0.01 m³ are left out of every map.

A ~1.5 GB, 5 minute, 35 m recording takes about 20 s (release build).

### Semantic labels

When a recording carries semantic labels (`/cam/<name>/depth/labels`, a class
per depth pixel from a segmentation model; see the
[capture format](docs/capture-format.md)), fusion counts per voxel the classes
it was seen as: ground, plant, tree, structure, sky or dynamic
([semantics.hpp](include/vg_core/semantics.hpp)). Votes from many frames settle
what single-frame labels get wrong, and stages ask the map by class.

- Readings of `build.semantics.drop_classes` (default people/animals and sky)
  are left out of the map altogether.
- The `voxels` stage also writes `labels.ply`, the voxels colored by class.
- The `semantic` stage writes `semantic.asc`: per cell, the class of the
  highest labeled voxel (1 ground ... 6 dynamic), i.e. what's seen from above.
- `semantics.min_votes` is how many votes a voxel's class needs (fewer leave it
  unknown); `vg mcap stats` shows the share of each class in a recording.

Recordings without labels map as before, with the `semantic` stage skipped.

To check a labeled recording:

```sh
vg mcap stats rec.vgcap.mcap        # share of each class, and how many frames have labels
vg labels perf rec.labels.csv       # the phone's inference time, latency, heat (CSV from the app)
vg labels frames rec.vgcap.mcap out/ --at 20 60 120   # PNGs: image | labels over it | depth over it
```

`vg labels frames` is the alignment check: the labels and depth are drawn over
the camera image they came from, so their edges should land on the image's
(trunks, fences, table tops). The depth covers the whole camera image, stretched
to its aspect ratio (`--fov stretch`, as the recorded depth intrinsics say);
`--fov crop` draws the other possibility, a centered crop, for comparison.

> **TODO before shipping the app: re-evaluate the segmentation model.**
> The first model (an off-the-shelf ADE20K model plus a class-mapping table)
> was chosen to get started, not for a product. Before shipping:
>
> - **Licenses:** ADE20K's terms and some popular weights (e.g. NVIDIA's
>   SegFormer) are for research / non-commercial use only. Ship commercially
>   usable weights and training data.
> - **Accuracy on real yards:** measure where it's wrong (mulch vs. soil vs.
>   low plants, raised beds, decks, furniture). If it matters, fine-tune a
>   mobile model on our own frames, labeled by hand or automatically by a
>   large open-vocabulary model run offline.
> - **On-phone cost:** inference time, label frame rate, and heat and battery
>   over a full mapping walk.

`.asc` files are ESRI ASCII grids: plain text, a six-line header (size,
lower-left corner, cell size, no-data value) and then one value per cell, top
row first. QGIS and GDAL open them directly; `vg heatmap` and `vg preview`
render them. Coordinates are the map's local frame in meters;
`manifest.json`'s `geo_reference` places it on the Earth.

The sun stage simulates the current year by default (`--year`, `--sun-step`
in minutes). If the session has no GPS, or the walk was too short to tell
which way the map faces, pass `--lat`/`--lon` and `--x-bearing` (compass
bearing of the map's +x axis); without a location the sun maps are skipped.

When a map comes out rough, two diagnostics tell noisy depth from tracking
error:

```sh
vg mcap stats session.mcap    # pose jumps, tracking changes, depth range / edge / confidence stats
vg map garden/ layers         # layers.asc: where a second surface sits over the first (ghosts)
```

Pose jumps (ARCore snapping its pose by several cm in one frame) double the
surfaces seen before and after them. Lots of far or edge depth makes
floating blobs; ghost layers mostly at the map's edges point to far depth
rather than tracking.

A recording the app never closed (killed, crashed, out of space) has no index,
and its last chunk may be cut off. `vg_replay` still reads it, but Foxglove and
other tools may not. `vg mcap repair` writes a new, indexed copy holding
everything up to the last complete chunk; the original is left alone:

```sh
vg mcap repair session.mcap                   # writes session.repaired.mcap
vg mcap repair session.mcap fixed.mcap        # or name the output (--force to replace it)
```

The same session opens in [Foxglove](https://foxglove.dev) for inspection.
In code, `vg::io::SessionReader` plus `vg::io::play()` give the same playback
with a callback per message, and `vg::io::SessionMapper` feeds it into a
`vg::MapBuilder`.

To drive an app's live pipeline from a recording, as if it were happening now,
`vg::io::SessionPlayer` hands out one message at a time on the recorded
schedule, and can be paused or sped up while it plays:

```cpp
vg::io::SessionPlayer player("session.mcap");  // real time
// On a playback thread:
while (auto message = player.next()) {
  feed_live_pipeline(*message);
}
// From the UI, at any time:
player.set_paused(true);
player.set_rate(4.0);
show_progress(player.position(), player.duration());
```

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
vg heatmap in.asc out.png [--scale 4] [--min V --max V]  # render a grid as a heatmap PNG
vg preview dir/     # render every .asc in a directory as a PNG (semantic.asc by class)
vg labels perf rec.labels.csv    # the phone's segmentation speed and heat
vg labels frames rec.mcap out/   # labels and depth over the camera image, per frame
                    # (needs: pip install -e "tools[analysis]")
vg replay ...       # run a C++ app from apps/ (here vg_replay), building it first
vg map ...          # vg_map
vg mcap repair ...  # vg_mcap_repair
```

Every C++ app is a `vg` subcommand, so there's no need to find its binary:
`apps/vg_<name>.cpp` is built as the target `vg_<name>` and runs as `vg <name>`,
with underscores splitting the name into words (`apps/vg_mcap_repair.cpp` is
`vg mcap repair`). Adding the file is all a new app needs: CMake and `vg` both
pick it up, and its first line, `// vg_<name>: what it does`, is its
`vg --help` entry. Arguments go to the app unchanged, and it runs in the
current directory, so `vg` works on recordings anywhere. Before each run the
app is rebuilt (incrementally) in the release build, configuring it if needed,
since a debug build of the mapping is many times slower; `--vg-preset NAME`
picks another build and `--vg-no-build` skips the rebuild. After `vg replay`
and `vg map`, the package's maps are rendered as PNG previews.

## License

vg_core is licensed under the [Apache License 2.0](LICENSE).
