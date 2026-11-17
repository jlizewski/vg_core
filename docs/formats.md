# File formats

Every file vg_core reads or writes, apart from recordings, which have their
own spec in [capture-format.md](capture-format.md).

Coordinates in all of these are the map's (or site's) local frame in meters,
z up. `manifest.json`'s `geo_reference` places a package on the Earth.

## Map package

One walk's directory. Written by `vg replay` (the build) and `vg map` (the
stages). Code: [map_package.hpp](../include/vg_core/map_package.hpp).

| File | Format | Units | Written by | Contents |
| --- | --- | --- | --- | --- |
| `map.vgm` | binary | – | build | The 3D map: TSDF volume and metadata |
| `trajectory.csv` | CSV | s, m | build | Camera path: `t, x, y, z` |
| `sky.vgs` | binary | – | build (labeled) | Sky evidence per coarse cell |
| `voxels.ply` | PLY | m | voxels | Occupied voxels as cubes |
| `labels.ply` | PLY | m | voxels (labeled) | Same, colored by class |
| `surface.asc` | ASCII grid | m | surface | Highest surface per cell |
| `ground.asc` | ASCII grid | m | ground | Bare ground per cell, patched |
| `ground_patched.asc` | ASCII grid | 0/1 | ground | 1 where patched rather than seen |
| `ground_state.asc` | ASCII grid | state | ground | What's known of the ground: 1 measured, 2 under an object (patched), 3 small hole (patched), 4 hole too large to patch; no data where not seen |
| `ground_weight.asc` | ASCII grid | weight | ground | Fused weight of the measured ground: how much it was seen |
| `ground_object.asc` | ASCII grid | id | ground | The `objects.csv` id of the largest object over the cell |
| `objects.ply` | PLY | m | ground | Everything on or over the ground |
| `objects.csv` | CSV | m, m² | ground | Each object: `id, voxels, grounded, class, class_share, bottom_above_ground_m, top_above_ground_m`, bounds `min_x … max_z`, `footprint_m2` |
| `terrain.asc` | ASCII grid | m | terrain | Smooth contour of the land |
| `features.asc` | ASCII grid | m | terrain | Surface above the terrain |
| `canopy.asc` | ASCII grid | m | sky | Lowest canopy over the terrain |
| `canopy_top.asc` | ASCII grid | m | sky | Highest canopy (upper bound) |
| `canopy.ply` | PLY | m | sky | Canopy cells as cubes |
| `sun_hours.asc` | ASCII grid | h | sun | Hours of direct sun over the period |
| `sun_energy.asc` | ASCII grid | kWh/m² | sun | Clear-sky direct energy |
| `semantic.asc` | ASCII grid | class | semantic | Class seen from above |
| `layers.asc` | ASCII grid | m | layers | Gap to a second surface (diagnostic) |
| `config.yaml` | YAML | – | build, stages | The settings the package was made with |
| `manifest.json` | JSON | – | build, stages | What's here and how it was made |
| `*.png` | PNG | – | `vg preview` | Previews of the grids |

### ASCII grids (`.asc`)

ESRI ASCII grids: plain text, a six-line header (`ncols`, `nrows`,
`xllcorner`, `yllcorner`, `cellsize`, `NODATA_value`), then one value per
cell, **top row first**. In memory (`HeightMap`) rows run bottom up (row =
y), so the reader and writer flip them. QGIS and GDAL open them directly;
`vg heatmap` and `vg preview` render them as PNG. Code:
[height_map_io.hpp](../include/vg_core/height_map_io.hpp).

Plain text is bulky but readable anywhere and easy to check by eye. If size
becomes a problem on the phone, a binary grid with the same header is the
natural replacement.

### `map.vgm`

Binary, little-endian. Magic `VGMAP\0\0\0` and a format version (3 added
label votes; older versions still load), then the `TsdfConfig`,
the `MapMetadata` (session name, cell size, geo-reference, frames
integrated), then every allocated block (index, then 512 tsdf/weight pairs)
in sorted block order, so the same map always gives the same bytes. Label
votes follow when the map has them. Code:
[volume_io.hpp](../include/vg_core/volume_io.hpp).

### `sky.vgs`

Binary: magic, cell size, frame count, cell count, then per cell its index
(three `int32`) and its `clear` and `blocked` counts (`uint32`). Code:
[sky.hpp](../include/vg_core/sky.hpp).

### `manifest.json`

```json
{
  "format": "vg-map-package",
  "format_version": 1,
  "vg_core_version": "…",
  "session": "garden-20261109-101500.vgcap.mcap",
  "voxel_size_m": 0.0762,
  "cell_size_m": 0.1524,
  "frames_integrated": 4210,
  "coordinates": "local map frame in meters, z up; geo_reference places it",
  "geo_reference": {"latitude": …, "longitude": …, "altitude_m": …,
                    "x_axis_bearing_deg": …, "bearing_sigma_deg": …},
  "stages": {"build": {…settings…}, "ground": {…}, …},
  "files": [{"name": "ground.asc", "units": "m", "description": "…"}, …]
}
```

`stages` records the settings each stage last ran with and what it found
(e.g. the terrain's solved phone height), so every map can be traced to how
it was made. `geo_reference` is `null` without GPS.

## Site

A directory of many walks. Code: [site.hpp](../include/vg_core/site.hpp).
See [pipeline/sites.md](pipeline/sites.md).

### `site.csv`

A header row, then one row per visit, oldest first:

| Column | Meaning |
| --- | --- |
| package | Package directory, relative to the site when inside it |
| recorded | `YYYYMMDD-HHMMSS` from the recording's name, or empty |
| yaw_deg, x, y, z | Transform into the site frame (yaw in degrees, shifts in m) |
| used_gps, peak, rival_ratio, overlap, inliers_10cm, inliers_30cm, median_distance | The fit when added; empty for the first visit |

### `current/`

The same grids as a package, in the site frame, plus `visit.asc` (visit
number per cell, 1 = first) and `changes.asc` (surface change in m).

### `layers/ground/`

The ground layer ([ground_layer.hpp](../include/vg_core/ground_layer.hpp)),
written by `vg site ground`. Grids are on one site grid at the first walk's
cell size.

| File | Format | Contents |
| --- | --- | --- |
| `ground.asc` | ASCII grid, m | Ground height; no data where outside or unknown |
| `state.asc` | ASCII grid | 1 measured, 2 filled, 3 covered, 4 unknown; no data outside |
| `provenance.asc` | ASCII grid | 1 measured, 2 inferred, 3 edited, 4 confirmed; no data where none |
| `observations.asc` | ASCII grid | How many walks measured the cell |
| `source.asc` | ASCII grid | The `evidence.csv` id counting most toward the height |
| `spread.asc` | ASCII grid, m | How far the farthest walk is from the height: where walks disagree |
| `evidence.csv` | CSV | The walks used: `id, package, recorded, yaw_deg, x, y, z`, and how each walk's ground was aligned: `align_cells` (shared with the walks before it), `align_offset_m` at `align_center_x/y`, `align_slope_x/y` (rise per m), `align_before_m` and `align_after_m` (median distance to the walks before it) |
| `edits.csv` | CSV | The user's edits: `id, time, kind, polygon, note`; kind `ground` or `not_ground`, polygon `x y;x y;...` in the site frame (m), the note is the rest of the line. Read, never rewritten |
| `layer.json` | JSON | How it was made: combine rule and alignment settings, cell size, walks, edits applied, cells per state |

## Planned

- **Garden model** storage inside the site directory:
  [design/garden-model.md](design/garden-model.md#storage).
- **vg_data** release file (SQLite): [design/plant-data.md](design/plant-data.md#build-and-release).
