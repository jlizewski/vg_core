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
  ground_patched.asc  1 where ground.asc was patched rather than seen (diagnostic)
  objects.ply      everything on or over the ground
  terrain.asc      smooth contour of the ground, from the seen ground and the walk (m)
  features.asc     height of the surface above the terrain: beds, mounds, plants (m)
  sky.vgs          where sky was seen, and blocked beyond depth (when labeled)
  canopy.asc       lowest canopy over the terrain, beyond depth's reach (m)
  canopy_top.asc   highest canopy (an upper bound), canopy.ply its cells
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
`terrain` (reads `ground.asc`, `surface.asc` and `trajectory.csv`), `sky`
(reads `sky.vgs` and `terrain.asc`), `sun`,
which reads `ground.asc`, so re-run `sun` after `ground`, and the diagnostic
`layers`. In code they're `vg::MapPackage`
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
pipeline: [voxels, surface, ground, terrain, sun, layers]
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
- The `ground` stage uses the labels alongside its slope rules
  (`ground.labels`): a column whose bottom voxel is labeled plant, tree,
  structure or dynamic isn't ground, so low plants and the feet of walls and
  fences stop reading as raised ground, and the ground there is patched from
  the lawn around it. The ground everything else is judged against is the
  region with the most columns labeled ground, not just the largest one (a big
  deck can't take its place). Each object in `objects.ply` gets the class most
  of its voxels have (in the manifest). `ground_patched.asc` shows which cells
  were patched rather than seen.
- `ground.labels.rescue` (off by default) also keeps small, isolated patches
  labeled ground. On a real yard it took in surfaces a meter up, since a deck
  or porch floor labeled ground can't be told from ground rising behind an
  obstacle.

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

### Segmentation everywhere, and on this computer

Segmentation itself is in vg_core, so the phone, a future iOS app, a ROS2 node
and the desktop all label alike: `vg::Segmenter`
([segmentation.hpp](include/vg_core/segmentation.hpp)) turns a camera image
upright from the camera's pose, runs a model, maps its classes to vg's and
shrinks the result to the depth image. Only running the model differs per
platform, behind `vg::InferenceBackend`: on the desktop it's ONNX Runtime
(`vg_core_onnx`, [onnx_backend.hpp](include/vg_core/onnx_backend.hpp); the
`VG_CORE_BUILD_ONNX` CMake option, on by default in a desktop checkout, fetches a
pinned release).

That lets a recording be relabeled on this computer, e.g. with another model or
class mapping, and the results compared without walking the yard again:

```sh
vg models export deeplabv3_mnv2_ade20k deeplabv3_mnv2_ade20k.tflite   # once: models/*.onnx
vg models export segformer_b0_ade20k        # from its PyTorch weights on Hugging Face
vg segment rec.mcap rec.segformer.mcap --model segformer_b0_ade20k
vg segment rec.mcap rec.desktop.mcap        # the same recording, relabeled here
vg labels compare rec.mcap rec.desktop.mcap # agreement, per-class IoU, confusion
vg replay rec.desktop.mcap pkg/             # and what the labels do to the maps
```

`vg models export` checks that the ONNX model picks the `.tflite`'s class on
random input (100% for DeepLabV3), so desktop labels stand in for the phone's.
On the Oct 16 recording the desktop's and the phone's labels agree on 81% of
pixels per frame (JPEG versus the raw camera image, the phone GPU's lower
precision), but their maps agree: DeepLabV3's labels flip on about a fifth of
pixels at the slightest change, which the per-voxel votes smooth out.

So per-pixel agreement has a noise floor around 80% even for the same model,
and the better measure of a model is **label consistency**: with labels, `vg
replay` logs (and the manifest records) the share of each frame's labels that
agree with the map's class where they land, per class. It needs no ground truth:
steady labels agree with the map, flipping ones don't. DeepLabV3 on the Oct 16
walk: 85% overall; ground 95%, structure 88%, tree 52%, plant 44%.

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

### Terrain

`ground.asc` is the ground where it was seen, patched in between. The
`terrain` stage fits a smooth contour to it (`terrain.asc`, nodes every
`terrain.node_spacing`, 0.5 m) and adds a second kind of evidence: the walk.
The phone is carried at a steady height (on a measured walk, 1.56 m over the
seen ground, +-18 cm), so the camera's path less that height is the ground
underfoot. One point is rough, but many pin the ground down where it's hidden
(overgrowth) and show up "ground" that is really foliage:

- The phone's height is solved for (`terrain.path.phone_height: auto`), from
  where the path crosses seen ground.
- The fit starts from the walk, then brings in the seen ground with robust
  weights: ground well above the fit (`terrain.above_scale`) counts for little
  where the walk disagrees with it. A node's area of seen ground counts as one
  observation, so a patch of foliage can't outvote the walk by its size.
- `features.asc` is `surface.asc` less the terrain: what stands on the land.
- The manifest records the phone height and how many ground cells were
  discounted (most of them near plants).

`vg path stats garden/` shows how the walk relates to the ground: how steady the
phone's height is, whether it reads lower near plants, and the longest stretches
walked without seen ground underfoot.

### Sky and canopy

Phone depth reaches about 3.5 m, but a canopy or roofline 6-15 m up still
shades the garden. The semantic labels see it: a pixel labeled sky is a ray
open all the way up, and a pixel looking up that isn't sky, with no depth
reading near by, is blocked somewhere beyond depth's reach
([sky.hpp](include/vg_core/sky.hpp)).

- While replaying, the build follows those rays through coarse cells
  (`build.sky.cell_size`, 0.5 m) and counts, per cell, the frames that saw sky
  through it and those that saw it blocked: `sky.vgs`.
- The `sky` stage reads it out: a cell many frames saw blocked
  (`sky.min_blocked`) and (almost) none saw sky through (`sky.max_clear`) is
  canopy. `canopy.asc` is its lowest height over the terrain per cell,
  `canopy.ply` its cells. Seen from below, what's above the canopy is never
  seen, so `canopy_top.asc` is only an upper bound.
- One walk sees little of the sky (even on a "look up" walk, 13% of frames
  were above level). A site combines every walk's evidence in its frame
  (`current/canopy.*`): on seven walks of one yard, the canopy from six
  predicted which of the seventh's upward pixels were sky better than a
  horizon profile, most under the canopy (sky IoU 30% -> 53%) and near the
  house (62% -> 74%).

## Sites: many walks of one place

A walk is one map in its own frame (the tracker's origin and heading are
arbitrary each time). A site lines up many walks of one place, so a garden can
be mapped over days and seasons, all on the phone: no cloud service.

```sh
vg site add garden-site walk1/ walk2/ walk3/   # register each into the site, remake its maps
vg site list garden-site                       # the visits, their alignment and fit
vg site maps garden-site                       # remake current/ only
```

- The first walk added sets the site's frame. Each later one is registered
  against every walk before it ([registration.hpp](include/vg_core/registration.hpp)):
  a heading-and-shift search matching the outlines of what stands on the
  terrain (`features.asc`), then a fine fit (ICP) on the voxels standing over
  the terrain. Walks only differ by a turn about vertical and a shift, since
  both know which way is down.
- GPS (`site.use_gps`) narrows the heading search and breaks near ties
  between alignments that fit alike, by how well each agrees with the walks'
  GPS headings and positions; it never overrules a clearly better fit.
  Without GPS all headings are searched (slower). Measured on real walks:
  - Long walks of the yard line up with or without GPS, also a month apart.
  - A sweep of one thing (a shed, the house, the canopy) fits several ways
    from geometry alone, so without GPS it's refused as ambiguous; with GPS
    they lined up.
  - Each walk is matched against the whole site so far, so adding short walks
    after a long one works best; a wrongly placed walk would mislead later
    ones, which is why doubtful fits are refused rather than guessed.
- A walk that doesn't line up convincingly isn't added, and the log says why:
  too little overlap, too few points within 0.3 m (`site.min_inliers`), or
  another alignment fitting nearly as well (`site.min_rival_ratio`). `--force`
  adds it anyway.
- Expect about half a meter and a couple of degrees of disagreement between
  walks: each walk drifts a little on its own, which one rigid alignment can't
  remove.
- `site.csv` lists the visits, oldest first (by the date in the recording's
  name), with each walk's transform into the site frame and its fit.
  `current/` holds the site's maps in its frame, each cell from the latest
  walk that saw it (`surface`, `ground`, `terrain`, `features`, `semantic`),
  plus `visit.asc` (which walk each cell came from) and `changes.asc` (the
  surface's change since the walk before: growth, and misalignment along
  walls).
- Settings: the `site` section of [config/mapping.yaml](config/mapping.yaml).
  Packages missing their terrain maps get them first.

### The dashboard

`vg dashboard --recordings DIR --sites DIR` opens a page (on this computer
only, http://127.0.0.1:8765) for the whole workflow, the way the app will run
it: recordings -> walks -> a site -> its maps.

- **Recordings**: each one with its tag (from a renamed video beside it,
  `shed-recording-....mp4`) and the walks made from it; replay one into a
  site as a walk, or get its stats.
- **Sites**: the visits lined up in the site's frame with their fit, walks
  not yet added (add them, long walks first), and any map of the site or of
  one walk, with every walk's path over it and the value under the cursor.
- **Jobs**: replays and site builds run one at a time, with their logs.

A site is a folder under `--sites` with its walks in `walks/`, so the whole
site can be moved or copied. It's pure Python, nothing to install.

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
vg labels compare a.mcap b.mcap  # how two labelings of a recording differ
                    # (both need: pip install -e "tools[analysis]")
vg path stats garden/            # the phone's height over the ground along the walk
vg site add site/ walk/          # vg_site: line up walks of one place (see Sites)
vg dashboard --recordings DIR --sites DIR  # browser dashboard (see Sites)
vg models export NAME m.tflite   # a segmentation model as ONNX for vg segment
                    # (needs: pip install -e "tools[models]", large: TensorFlow)
vg segment rec.mcap out.mcap     # vg_segment: relabel a recording on this computer
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
