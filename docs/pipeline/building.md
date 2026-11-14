# Building the 3D map

The build turns a stream of depth images and poses into a 3D map: a TSDF
volume. Everything else is derived from it.

Code: [map_builder.hpp](../../include/vg_core/map_builder.hpp),
[tsdf_volume.hpp](../../include/vg_core/tsdf_volume.hpp),
[depth_filter.hpp](../../include/vg_core/depth_filter.hpp),
[pose_jumps.hpp](../../include/vg_core/pose_jumps.hpp),
[depth_spool.hpp](../../include/vg_core/depth_spool.hpp),
[live_height_map.hpp](../../include/vg_core/live_height_map.hpp), and for
replaying a recording [io/map_build.hpp](../../include/vg_core/io/map_build.hpp).
Settings: the `build` section of [config/mapping.yaml](../../config/mapping.yaml).

## Inputs

From the phone, per frame ([sensor_data.hpp](../../include/vg_core/sensor_data.hpp)):

- a **pose**: where the phone body is in the tracker's world frame;
- a **depth image**: distance per pixel along the camera axis, optionally
  with a **confidence** per pixel and a semantic **label** per pixel;
- the camera's **intrinsics** and its **extrinsics** (where it sits on the
  body).

GPS isn't used to build the map; it places the finished map on the Earth
(see [sun-and-sky.md](sun-and-sky.md#geo-reference)).

## Step by step

### 1. Undo the tracker's jumps

ARCore and ARKit correct their drift by snapping the pose several centimeters
in one frame. Mapped as is, everything after the snap sits offset from
everything before it: a ghost copy of the ground a few centimeters up.

`PoseJumpFilter` watches consecutive poses. A move faster than a hand-held
phone can go (2 m/s, 180°/s) and bigger than 3 cm or 3° is a jump. Across a
jump it assumes the motion carried on as just before, and shifts the rest of
the walk to follow on from there. The whole walk stays in the frame of its
start. `vg mcap stats` lists a recording's jumps.

### 2. Pair each depth frame with a pose

Depth and poses arrive at different times. `MapBuilder` pairs each depth
frame with the closest pose in time, either the last one before it or the
first one after it, as long as it's within 50 ms (`max_pose_age`). So a depth
frame waits until the next pose arrives; `flush()` settles the last one at
the end. Frames taken while the tracker reports limited tracking are skipped.

### 3. Clean the depth

Phone depth is worst far away, across depth edges, and on surfaces seen
edge-on. `depth_filter.hpp` drops:

- readings outside 0.1–3.5 m;
- **flying pixels**: readings on a depth edge, where depth smears between the
  near and the far surface (a neighbor differs by more than a set fraction of
  the depth);
- readings of surfaces seen more than a set angle from head-on.

Left in, these become floating blobs and ghost surfaces.

### 4. Fuse into the TSDF

The **TSDF** (truncated signed distance field) stores, per voxel, the
distance to the nearest surface along the camera ray: positive in front,
negative behind, clamped to the **truncation distance** (3 voxels by
default), normalized to [-1, 1]. Each voxel also has a **weight**: how much
evidence it has.

Each depth frame updates the voxels near the surfaces it sees:

1. **Allocate** every 8×8×8 block within the truncation band around each
   reading. Blocks live in a hash map keyed by block index (voxel hashing),
   so empty space costs nothing.
2. **Project** each voxel of those blocks into the depth image. The
   difference between the reading there and the voxel's own depth is the
   voxel's signed distance for this frame.
3. **Average** it into the voxel's running value, weighted. With
   `quality_weights` the reading's weight is its confidence × (1 m / depth)²
   × the cosine of its viewing angle, so near, confident, head-on readings
   count most. The total weight is capped (64) so the map can still change.

Averaging many frames cancels most of the noise in any one. Blocks are fused
in parallel, and the result is the same for any number of threads.

**Free-space carving** (`carve`) also updates the existing blocks a ray
passes through on its way to the surface, marking them empty. A blob mapped
from one bad reading fades out once later frames see through it. Carving
never allocates new blocks.

**Labels**: when a frame has semantic labels, each voxel near the surface
counts how many times it was seen as each class (label votes). Readings of
dropped classes (people, animals, sky by default) don't enter the map at all.
See [semantics.md](semantics.md).

### 5. Read the map out

- **Occupied voxels**: the voxels just behind each surface (negative
  distance, with an observed voxel in front). Voxels with less total weight
  than `min_weight` are left out as seen too little to trust. The default
  depends on the depth source: 0.05 for raw depth with confidence (ARCore raw
  depth is mostly low confidence, so each reading weighs little), 0.5 for
  smoothed depth without it.
- **Surface points**: where the distance crosses zero, interpolated between
  voxel centers.
- **Specks**, groups of voxels connected to nothing and smaller than 0.01 m³,
  are left out before the derived stages
  ([voxel_cleanup.hpp](../../include/vg_core/voxel_cleanup.hpp)).

## Sizes

- **2D cell:** 0.1524 m (6 in), `map.resolution`.
- **Voxel:** half the cell, about 7.6 cm, so stems and posts still show.
- **Block:** 8 voxels on a side, about 61 cm.

## Live on the phone

On the phone, the map is built as the user walks, for the on-screen view.
Fusing every frame live is more than a phone can sustain, so `MapBuilder`
can:

- **Fuse keyframes only**: a frame taken after the phone has moved
  `keyframe_translation` or turned `keyframe_rotation` since the last one
  fused. The rest are **deferred**.
- **Defer everything** while the mapping thread falls behind (`set_live(false)`).

Deferred frames go to a `DepthSpool`, in memory or in a file, with the pose
they were paired with. Depth is stored as whole millimeters, which is exact
for ARCore. After recording, `integrate_deferred(n)` fuses them in batches, so
the finished map holds every frame.

`update()`, called a few times a second, reports what changed since the last
call: height map cells (from a `LiveHeightMap`, kept per block) and voxel
blocks to redraw. Its cost depends on how much changed, not on the map's size.

## From a recording

`vg::io::build_map_package` (used by `vg replay`) replays a recording through
the same steps, then:

- estimates the geo-reference from the GPS fixes and poses;
- writes `map.vgm` (the TSDF and its metadata), `trajectory.csv` (the camera
  path, used by the terrain stage) and `sky.vgs` (sky evidence, when the
  recording is labeled);
- scores label consistency when labeled ([semantics.md](semantics.md#label-quality));
- starts the manifest.

A five-minute, 35 m, 1.5 GB recording builds in about 20 s (release build).

## When a map comes out rough

| Symptom | Likely cause | Check |
| --- | --- | --- |
| Doubled ground or walls | Tracker jumps | `vg mcap stats` (jumps), `vg map <pkg> layers` |
| Floating blobs | Far or edge depth | `vg mcap stats` (depth by range, edges) |
| Ghosts mostly at the map's edges | Far depth rather than tracking | `layers.asc` |
| Holes | Not enough views, or `min_weight` too high | `vg map <pkg> voxels` with a lower `voxels.min_weight` |
