# Sun and sky

How vg_core works out where the map is on the Earth, where the sun is, how
much sun each spot gets, and what's overhead beyond the phone's depth range.

Code: [geo_reference.hpp](../../include/vg_core/geo_reference.hpp),
[solar.hpp](../../include/vg_core/solar.hpp),
[sun_map.hpp](../../include/vg_core/sun_map.hpp),
[sky.hpp](../../include/vg_core/sky.hpp). Settings: the `sun` and `sky`
sections of [config/mapping.yaml](../../config/mapping.yaml).

## Geo-reference

A walk's world frame is gravity aligned, but its origin and heading are
wherever the tracker happened to start. Sun calculations need to know which
way is north and where on the Earth the map is.

`estimate_geo_reference()` works it out from the GPS fixes and the poses
recorded together:

1. **Position**: the weighted mean of the fixes (each weighted by its
   reported accuracy) is the reference point.
2. **Pairing**: each fix is paired with the phone's position in the world
   frame at the same moment, interpolated between poses.
3. **Heading**: a weighted 2D Procrustes fit finds the rotation and shift that
   best line up the GPS track with the pose track.
4. **Uncertainty**: `heading_sigma` comes from how far the walk spread
   compared with the GPS error, inflated if the fixes scatter more than they
   claim.

Phone GPS is off by meters, so the heading is only good when the walk is long
compared with that error. A few meters of walking can leave it tens of
degrees off. `heading_sigma` reports this, and it is infinite when the
heading couldn't be observed at all. The manifest stores it as
`bearing_sigma_deg`.

When GPS is missing or the heading is poor, the sun stage takes the location
and the bearing of the map's +x axis by hand (`--lat`, `--lon`,
`--x-bearing`). Sites don't yet compute sun maps; when they do, the site's
heading can come from every walk's GPS together, which will be far better
than any one walk's (see the [roadmap](../roadmap.md)).

## Sun position

`sun_position()` uses the NOAA solar position algorithm (from Meeus,
*Astronomical Algorithms*): azimuth and elevation for a UTC time and a
latitude and longitude, accurate to about 0.01° for 1800–2100, with
atmospheric refraction near the horizon.

`clear_sky_direct_irradiance()` gives the direct sunlight on a surface facing
the sun, after the atmosphere: the Meinel model with the Kasten–Young air
mass. It's a clear-sky model: no clouds, haze or weather.

## Sun map

`compute_sun_map()`, the `sun` stage. Writes `sun_hours.asc` and
`sun_energy.asc` over the cells of `ground.asc`.

For each time step over the period (a year, hourly, by default), at the
step's midpoint:

1. Compute the sun's direction for the map's location.
2. Skip the step if the sun is below `min_elevation_deg`.
3. From each ground cell, at `sample_height` above the ground (one voxel, at
   least 4 cm, so the ray clears the ground's own bumps), cast a ray toward
   the sun through the occupied voxels.
4. If the ray escapes the map, the cell is in sun for that step: add the
   step's hours, and the step's energy, which is the direct irradiance times
   the cosine of the angle between the sun and the ground's slope at that
   cell (the slope from neighboring heights).

Everything mapped casts shadows: plants, beds, fences, the house, and the
slope of the ground itself.

For speed, the voxels are stored as vertical runs per column, and columns are
grouped into tiles that remember their tallest run, so a ray climbing above a
tile skips it whole. Rays walk the columns they cross in 2D. Rows of cells
are split across threads. A year at hourly steps over a 10 × 10 m map at
5 cm cells takes about 15 s on 4 cores.

Outputs:

- `sun_hours`: hours of direct sun per cell.
- `irradiation` (`sun_energy.asc`): clear-sky direct energy in kWh/m².
- `daylight_hours`: hours the sun was up, the most any cell could get. Sun
  hours over daylight hours is the share of possible sun.

### Assumptions, and what they mean for accuracy

| Assumption | Effect | Plan |
| --- | --- | --- |
| Clear sky | Overstates real sun, more in cloudy climates | Combine with cloud climatology or forecasts ([roadmap](../roadmap.md)) |
| Space beyond the map is open | A tall tree or building just outside the walk doesn't shade | The canopy from sky evidence fills part of this gap (below) |
| Direct light only | Diffuse skylight, which plants also use, is ignored | Add sky-view fraction from the same rays |
| One total for the period | A bed's sun in June differs from December | Monthly and growing-season sums ([roadmap](../roadmap.md)) |
| Geometry as of the walk | Deciduous trees shade in summer, not winter | Seasonal canopy from walks in different months, via sites |

The garden model will report these with each value (see
[design/garden-model.md](../design/garden-model.md#accuracy)).

## Sky and canopy

Phone depth reaches about 3.5 m, but a tree canopy or a roofline 6–15 m up
still shades the garden. Semantic labels see it even where depth can't.

### Sky evidence (during the build)

For a labeled depth frame, every few pixels (`pixel_stride`, 3):

- A pixel labeled **sky** is a ray open all the way up. Every coarse cell
  (`cell_size`, 0.5 m) it crosses, out to `range` (40 m), gets a **clear**
  count.
- A pixel looking up (at least 3° above level) that **isn't** sky and has no
  depth reading within 3.5 m is blocked somewhere beyond depth's reach. The
  cells it crosses past that distance get a **blocked** count.

Counts are kept per frame, so a cell knows how many frames saw sky through it
and how many saw it blocked. Saved as `sky.vgs`.

### Canopy (the sky stage)

`make_canopy()`: a cell is **occupied** if at least `min_blocked` (20)
frames saw it blocked and at most `max_clear` (1) saw sky through it
(labels are sometimes wrong). Occupied cells lower than 2 m over the terrain
are dropped, since depth maps what's that low. Together the occupied cells
are a visual hull of the canopy: everything consistent with what was seen.

Outputs: `canopy.asc` (the lowest occupied cell over the terrain per column,
which is what was measured from below), `canopy_top.asc` (the highest, only
an upper bound since no ray reaches above the canopy), and `canopy.ply`.

One walk sees little sky: even on a "look up" walk, 13% of frames were above
level. A site combines every walk's evidence in its own frame
(`current/canopy.*`), which is where the canopy becomes useful. On seven
walks of one yard, the canopy from six predicted which of the seventh's
upward pixels were sky better than a horizon profile: sky IoU from 30% to 53%
under the canopy, and 62% to 74% near the house.

The canopy isn't yet used as an occluder by the sun stage; see the
[roadmap](../roadmap.md).
